package us.ihmc.scs2.session.log;

import com.github.luben.zstd.Zstd;
import us.ihmc.robotDataLogger.LogIndex;
import us.ihmc.robotDataLogger.logger.LogCompressionType;
import us.ihmc.tools.compression.SnappyUtils;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

/**
 * A read-only, random-access view of a log's data file dedicated to {@link #readVariableValueBitsAt}, with its own
 * {@link FileChannel} and batch-decompression cursor kept entirely separate from a {@link LogDataReader}'s own live
 * sequential-playback cursor.
 * <p>
 * Backfilling a lazily-created buffer's history (see {@code LogSession#getHistoricalValueBits}) needs to seek around
 * freely without disturbing live playback. Sharing one {@link LogDataReader}'s channel between both uses meant every
 * backfill had to save the live cursor, do its own seeking, then restore the live cursor afterward - and for a
 * compressed/batched log, that restore always lands in a different batch than the one backfilling had just
 * decompressed (the live cursor is off wherever playback happens to be, not near the backfilled range), evicting it
 * immediately. That made repeatedly re-visiting the same region - e.g. scrubbing a chart back and forth - reload the
 * same batch on every single call. A reader with nothing else to restore only reloads when the requested batch
 * actually changes.
 * </p>
 */
class LogRandomAccessValueReader implements AutoCloseable
{
   private final FileInputStream fileInputStream;
   private final FileChannel channel;

   private final boolean compressed;
   private final LogCompressionType compressionType;
   private final LogIndex logIndex;
   private final int batchSize;
   private final int singleTickSize;

   private final ByteBuffer compressedBuffer;
   private final ByteBuffer batchBuffer;
   /** Scratch for the uncompressed path, which reads its 8 bytes straight off the channel. */
   private final ByteBuffer singleValue = ByteBuffer.allocate(Long.BYTES);

   private int index;
   private int currentBatchTickCount;
   /** The batch index currently decompressed into {@link #batchBuffer}, or {@code -1} if none. */
   private int loadedBatchIndex = -1;

   LogRandomAccessValueReader(File logDataFile, boolean compressed, LogCompressionType compressionType, LogIndex logIndex, int batchSize, int singleTickSize)
         throws IOException
   {
      this.compressed = compressed;
      this.compressionType = compressionType;
      this.logIndex = logIndex;
      this.batchSize = compressed ? batchSize : 1;
      this.singleTickSize = singleTickSize;

      fileInputStream = new FileInputStream(logDataFile);
      channel = fileInputStream.getChannel();

      if (compressed)
      {
         int rawBatchBytes = singleTickSize * this.batchSize;
         compressedBuffer = ByteBuffer.allocate(switch (compressionType)
                                                {
                                                   case ZSTD -> (int) Zstd.compressBound(rawBatchBytes);
                                                   default -> SnappyUtils.maxCompressedLength(rawBatchBytes);
                                                });
         batchBuffer = ByteBuffer.allocate(rawBatchBytes);
      }
      else
      {
         compressedBuffer = null;
         batchBuffer = null;
      }
   }

   /**
    * Reads {@code variableIndex}'s raw value out of the record at {@code position}, without decoding the record's
    * other variables - see {@link LogDataReader#readVariableValueBitsAt} for the live-cursor equivalent this mirrors.
    *
    * @return the raw value bits, or {@code 0} if {@code position} lies past the end of the log's recorded data.
    */
   long readVariableValueBitsAt(int position, int variableIndex)
   {
      try
      {
         // Byte offset of this one variable within the tick record. The leading long is the timestamp, which is why
         // the variables start at index 1 - see LogDataReader.readAndProcessALogLineReturnTrueIfDone.
         int byteOffsetInTick = (1 + variableIndex) * Long.BYTES;

         if (compressed)
         {
            if (!loadBatchFor(position))
               return 0L;

            int tickOffset = position % batchSize;
            // Read the 8 bytes straight out of the decompressed batch. Copying the whole tick into logLine first
            // (as the sequential-playback path does, because it decodes every variable) would mean a memcpy of
            // singleTickSize - 8 bytes per variable, so hundreds of KB on a log with tens of thousands of
            // variables - per single value read, and backfilling a chart reads thousands of them.
            return batchBuffer.getLong(tickOffset * singleTickSize + byteOffsetInTick);
         }
         else
         {
            // Same idea uncompressed: seek straight to the value rather than reading the whole record.
            channel.position((long) position * (long) singleTickSize + byteOffsetInTick);
            singleValue.clear();
            if (channel.read(singleValue) != Long.BYTES)
               return 0L;
            return singleValue.getLong(0);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Makes sure the batch holding {@code position} is the one decompressed into {@link #batchBuffer}.
    *
    * @return {@code false} if that tick isn't in the log, in which case the reader's state is left untouched.
    */
   private boolean loadBatchFor(int position) throws IOException
   {
      int batchIndex = position / batchSize;
      int tickOffset = position % batchSize;

      if (batchIndex != loadedBatchIndex)
      { // Not the batch currently decompressed into batchBuffer - go get it.
         if (batchIndex < 0 || batchIndex >= logIndex.dataOffsets.length)
            return false;

         channel.position(logIndex.dataOffsets[batchIndex]);
         currentBatchTickCount = 0;
         index = batchIndex;

         if (!readNextBatch())
            return false;
      }

      // Decompressing is what tells us how many ticks the batch actually holds, so this has to be checked after it
      // is loaded rather than against batchSize up front: the final batch is usually short.
      return tickOffset < currentBatchTickCount;
   }

   private boolean readNextBatch() throws IOException
   {
      if (index >= logIndex.getNumberOfEntries())
         return false;

      int batchBeingLoaded = index;

      int size = logIndex.compressedSizes[index];
      compressedBuffer.clear();
      compressedBuffer.limit(size);

      int read = channel.read(compressedBuffer);
      if (read != size)
         throw new RuntimeException("Expected read of " + size + ", got " + read + ". TODO: Implement loop for reading the full log line.");

      compressedBuffer.flip();
      batchBuffer.clear();

      switch (compressionType)
      {
         case ZSTD ->
         {
            long result = Zstd.decompressByteArray(batchBuffer.array(),
                                                   0,
                                                   batchBuffer.capacity(),
                                                   compressedBuffer.array(),
                                                   compressedBuffer.position(),
                                                   compressedBuffer.remaining());
            if (Zstd.isError(result))
               throw new RuntimeException("Zstd decompression failed: " + Zstd.getErrorName(result));
            batchBuffer.position((int) result);
         }
         default ->
         {
            try
            {
               SnappyUtils.uncompress(compressedBuffer, batchBuffer);
            }
            catch (Exception e)
            {
               throw new RuntimeException(e);
            }
         }
      }

      currentBatchTickCount = batchBuffer.position() / singleTickSize;
      loadedBatchIndex = batchBeingLoaded;
      index++;
      return true;
   }

   @Override
   public void close() throws IOException
   {
      fileInputStream.close();
   }
}

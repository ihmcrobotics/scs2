package us.ihmc.scs2.session.log;

import com.github.luben.zstd.Zstd;
import us.ihmc.robotDataLogger.LogIndex;
import us.ihmc.robotDataLogger.logger.LogCompressionType;
import us.ihmc.tools.compression.SnappyUtils;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
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
   private final ByteBuffer logLine;
   private final LongBuffer logLongArray;

   private int index;
   private int tickIndexInBatch;
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

      logLine = ByteBuffer.allocate(singleTickSize);
      logLongArray = logLine.asLongBuffer();
   }

   /**
    * Reads {@code variableIndex}'s raw value out of the record at {@code position}, without decoding the record's
    * other variables - see {@link LogDataReader#readVariableValueBitsAt} for the live-cursor equivalent this mirrors.
    */
   long readVariableValueBitsAt(int position, int variableIndex)
   {
      try
      {
         positionChannel(position);
         readLogLine();
         return logLongArray.get(1 + variableIndex);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void positionChannel(int position) throws IOException
   {
      if (compressed)
      {
         int batchIndex = position / batchSize;
         int tickOffset = position % batchSize;

         if (batchIndex == loadedBatchIndex)
         {
            // Already decompressed - just move the in-batch cursor, no re-read/re-decompress needed.
            tickIndexInBatch = tickOffset;
            return;
         }

         currentBatchTickCount = 0;
         tickIndexInBatch = 0;
         index = batchIndex;

         if (batchIndex < logIndex.dataOffsets.length)
            channel.position(logIndex.dataOffsets[batchIndex]);

         if (tickOffset > 0)
         {
            readNextBatch();
            tickIndexInBatch = tickOffset;
         }
      }
      else
      {
         channel.position((long) position * (long) logLine.capacity());
      }
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
      tickIndexInBatch = 0;
      loadedBatchIndex = batchBeingLoaded;
      index++;
      return true;
   }

   private boolean readLogLine() throws IOException
   {
      logLine.clear();
      logLongArray.clear();

      if (compressed)
      {
         if (tickIndexInBatch >= currentBatchTickCount)
         {
            if (!readNextBatch())
               return false;
         }

         int tickStart = tickIndexInBatch * singleTickSize;
         batchBuffer.limit(tickStart + singleTickSize);
         batchBuffer.position(tickStart);
         logLine.put(batchBuffer);
         tickIndexInBatch++;

         return true;
      }
      else
      {
         int read = channel.read(logLine);
         if (read < 0)
            return false;
         if (read != logLine.capacity())
            throw new RuntimeException("Expected read of " + logLine.capacity() + ", got " + read + ". TODO: Implement loop for reading the full log line.");
         return true;
      }
   }

   @Override
   public void close() throws IOException
   {
      fileInputStream.close();
   }
}

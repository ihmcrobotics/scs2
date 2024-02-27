package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;

import java.nio.ByteBuffer;
import java.util.Objects;

/**
 * Chunk records each contain a batch of Schema, Channel, and Message records.
 * The batch of records contained in a chunk may be compressed or uncompressed.
 * All messages in the chunk must reference channels recorded earlier in the file (in a previous chunk or earlier in the current chunk).
 *
 * <p>
 * MCAP files can have Schema, Channel, and Message records written directly to the data section, or they can be written into Chunk records to facilitate
 * indexing and compression.
 * For MCAPs that include Chunk Index records in the summary section, all Message records should be written into Chunk records.
 * Why?
 * The presence of Chunk Index records in the summary section indicates to readers that the MCAP is indexed, and they can use those records to look up messages
 * by log time or topic.
 * However, Message records outside of chunks cannot be indexed, and may not be found by readers using the index.
 * </p>
 *
 * @see <a href="https://mcap.dev/spec#chunk-op0x06">MCAP Chunk</a>
 */
public interface Chunk extends MCAPElement
{
   static Chunk load(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      return new ChunkDataInputBacked(dataInput, elementPosition, elementLength);
   }

   /**
    * Earliest message log_time in the chunk. Zero if the chunk has no messages.
    */
   long messageStartTime();

   /**
    * Latest message log_time in the chunk. Zero if the chunk has no messages.
    */
   long messageEndTime();

   /**
    * Uncompressed size of the records field.
    */
   long recordsUncompressedLength();

   /**
    * CRC-32 checksum of uncompressed `records` field. A value of zero indicates that CRC validation
    * should not be performed.
    */
   long uncompressedCRC32();

   /**
    * compression algorithm. i.e. zstd, lz4, "". An empty string indicates no compression. Refer to
    * well-known compression formats.
    */
   Compression compression();

   /**
    * Length of the records in bytes.
    */
   long recordsCompressedLength();

   default ByteBuffer getRecordsCompressedBuffer()
   {
      return getRecordsCompressedBuffer(false);
   }

   ByteBuffer getRecordsCompressedBuffer(boolean directBuffer);

   default ByteBuffer getRecordsUncompressedBuffer()
   {
      return getRecordsUncompressedBuffer(false);
   }

   ByteBuffer getRecordsUncompressedBuffer(boolean directBuffer);

   /**
    * The decompressed records.
    */
   Records records();

   @Override
   default String toString(int indent)
   {
      String out = getClass().getSimpleName() + ":";
      out += "\n\t-messageStartTime = " + messageStartTime();
      out += "\n\t-messageEndTime = " + messageEndTime();
      out += "\n\t-compression = " + compression();
      out += "\n\t-recordsUncompressedLength = " + recordsUncompressedLength();
      out += "\n\t-uncompressedCrc32 = " + uncompressedCRC32();
      out += "\n\t-records = " + (records() == null ? "null" : "\n" + EuclidCoreIOTools.getCollectionString("\n", records(), e -> e.toString(indent + 1)));
      return MCAPElement.indent(out, indent);
   }

   @Override
   default boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof Chunk other)
      {
         if (messageStartTime() != other.messageStartTime())
            return false;
         if (messageEndTime() != other.messageEndTime())
            return false;
         if (recordsUncompressedLength() != other.recordsUncompressedLength())
            return false;
         if (uncompressedCRC32() != other.uncompressedCRC32())
            return false;
         if (compression() != other.compression())
            return false;
         if (recordsCompressedLength() != other.recordsCompressedLength())
            return false;
         return Objects.equals(records(), other.records());
      }

      return false;
   }

   default Chunk crop(long startTimestamp, long endTimestamp)
   {
      long croppedStartTime = MathTools.clamp(messageStartTime(), startTimestamp, endTimestamp);
      long croppedEndTime = MathTools.clamp(messageEndTime(), startTimestamp, endTimestamp);
      Records croppedRecords = records().crop(croppedStartTime, croppedEndTime);
      // There may be no records when testing a chunk that is before the start timestamp.
      // We still want to test it in case there stuff like schemas, channels, and other time-insensitive data.
      if (croppedRecords.isEmpty())
         return null;

      MutableChunk croppedChunk = new MutableChunk();
      croppedChunk.setMessageStartTime(croppedStartTime);
      croppedChunk.setMessageEndTime(croppedEndTime);
      croppedChunk.setRecords(croppedRecords);
      croppedChunk.setCompression(compression());

      return croppedChunk;
   }
}

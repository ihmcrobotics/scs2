package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class ChunkDataInputBacked implements Chunk
{
   private final MCAPDataInput dataInput;
   private final long elementLength;
   /**
    * Earliest message log_time in the chunk. Zero if the chunk has no messages.
    */
   private final long messageStartTime;
   /**
    * Latest message log_time in the chunk. Zero if the chunk has no messages.
    */
   private final long messageEndTime;
   /**
    * Uncompressed size of the records field.
    */
   private final long recordsUncompressedLength;
   /**
    * CRC32 checksum of uncompressed records field. A value of zero indicates that CRC validation
    * should not be performed.
    */
   private final long uncompressedCRC32;
   /**
    * compression algorithm. i.e. zstd, lz4, "". An empty string indicates no compression. Refer to
    * well-known compression formats.
    */
   private final Compression compression;
   /**
    * Offset position of the records in either in the {@code  ByteBuffer} or {@code FileChannel},
    * depending on how this chunk was created.
    */
   private final long recordsOffset;
   /**
    * Length of the records in bytes.
    */
   private final long recordsCompressedLength;
   /**
    * The decompressed records.
    */
   private WeakReference<Records> recordsRef;
   private WeakReference<ByteBuffer> recordsUncompressedBufferRef;

   ChunkDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;
      this.elementLength = elementLength;

      dataInput.position(elementPosition);
      messageStartTime = dataInput.getLong();
      messageEndTime = dataInput.getLong();
      recordsUncompressedLength = MCAP.checkPositiveLong(dataInput.getLong(), "recordsUncompressedLength");
      uncompressedCRC32 = dataInput.getUnsignedInt();
      compression = Compression.fromString(dataInput.getString());
      recordsCompressedLength = MCAP.checkPositiveLong(dataInput.getLong(), "recordsCompressedLength");
      recordsOffset = dataInput.position();
      MCAP.checkLength(getElementLength(), 4 * Long.BYTES + Integer.BYTES + compression.getLength() + recordsCompressedLength);
   }

   @Override
   public long getElementLength()
   {
      return elementLength;
   }

   @Override
   public long messageStartTime()
   {
      return messageStartTime;
   }

   @Override
   public long messageEndTime()
   {
      return messageEndTime;
   }

   @Override
   public long recordsUncompressedLength()
   {
      return recordsUncompressedLength;
   }

   /**
    * CRC-32 checksum of uncompressed `records` field. A value of zero indicates that CRC validation
    * should not be performed.
    */
   @Override
   public long uncompressedCRC32()
   {
      return uncompressedCRC32;
   }

   @Override
   public Compression compression()
   {
      return compression;
   }

   public long recordsOffset()
   {
      return recordsOffset;
   }

   @Override
   public long recordsCompressedLength()
   {
      return recordsCompressedLength;
   }

   @Override
   public ByteBuffer getRecordsCompressedBuffer(boolean directBuffer)
   {
      return dataInput.getByteBuffer(recordsOffset, (int) recordsCompressedLength, directBuffer);
   }

   @Override
   public ByteBuffer getRecordsUncompressedBuffer(boolean directBuffer)
   {
      ByteBuffer recordsUncompressedBuffer = recordsUncompressedBufferRef == null ? null : recordsUncompressedBufferRef.get();

      if (recordsUncompressedBuffer == null)
      {
         recordsUncompressedBuffer = dataInput.getDecompressedByteBuffer(recordsOffset,
                                                                         (int) recordsCompressedLength,
                                                                         (int) recordsUncompressedLength,
                                                                         compression,
                                                                         directBuffer);
         recordsUncompressedBufferRef = new WeakReference<>(recordsUncompressedBuffer);
      }

      return recordsUncompressedBuffer.duplicate().order(ByteOrder.LITTLE_ENDIAN);
   }

   @Override
   public Records records()
   {
      Records records = recordsRef == null ? null : recordsRef.get();

      if (records != null)
         return records;

      if (compression == Compression.NONE)
      {
         records = Records.load(dataInput, recordsOffset, (int) recordsCompressedLength);
      }
      else
      {
         records = Records.load(MCAPDataInput.wrap(getRecordsUncompressedBuffer()), 0, (int) recordsUncompressedLength);
      }

      recordsRef = new WeakReference<>(records);
      return records;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Chunk other && Chunk.super.equals(other);
   }
}

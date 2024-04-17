package us.ihmc.scs2.session.mcap.specs.records;

import com.github.luben.zstd.ZstdCompressCtx;
import us.ihmc.scs2.session.mcap.encoding.LZ4FrameEncoder;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.output.MCAPByteBufferDataOutput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Collection;
import java.util.Objects;

public class MutableChunk implements Chunk
{
   private Compression compression = Compression.LZ4;

   private Records records;
   private long lastRecordsCRC32 = -1L;
   private ByteBuffer recordsCompressedData;

   public void setCompression(Compression compression)
   {
      this.compression = compression;
   }

   public void setRecords(Collection<? extends Record> records)
   {
      this.records = new Records(records);
   }

   @Override
   public long messageStartTime()
   {
      return records.getMessageStartTime();
   }

   @Override
   public long messageEndTime()
   {
      return records.getMessageEndTime();
   }

   @Override
   public long recordsUncompressedLength()
   {
      return records.getElementLength();
   }

   @Override
   public long uncompressedCRC32()
   {
      return records.getCRC32();
   }

   @Override
   public Compression compression()
   {
      return compression;
   }

   @Override
   public long recordsCompressedLength()
   {
      return getRecordsCompressedBuffer().remaining();
   }

   @Override
   public Records records()
   {
      return records;
   }

   @Override
   public long getElementLength()
   {
      getRecordsCompressedBuffer(); // Make sure the compressed data is available.
      return 4 * Long.BYTES + Integer.BYTES + compression.getLength() + recordsCompressedLength();
   }

   @Override
   public ByteBuffer getRecordsCompressedBuffer(boolean directBuffer)
   {
      long newRecordsCRC32 = uncompressedCRC32();

      if (recordsCompressedData == null || lastRecordsCRC32 != newRecordsCRC32)
      {
         lastRecordsCRC32 = newRecordsCRC32;
         Objects.requireNonNull(compression, "The compression has not been set yet.");
         Objects.requireNonNull(records, "The records have not been set yet.");
         
         ByteBuffer uncompressedBuffer = getRecordsUncompressedBuffer(compression == Compression.ZSTD);

         // Eclipse seems to be struggling to compile the following when formulated as a switch-yield statement.
         if (compression == Compression.NONE)
         {
            recordsCompressedData = uncompressedBuffer;
         }
         else if (compression == Compression.LZ4)
         {
            LZ4FrameEncoder lz4FrameEncoder = new LZ4FrameEncoder();
            recordsCompressedData = lz4FrameEncoder.encode(uncompressedBuffer, null);
         }
         else if (compression == Compression.ZSTD)
         {
            try (ZstdCompressCtx zstdCompressCtx = new ZstdCompressCtx())
            {
               recordsCompressedData = zstdCompressCtx.compress(uncompressedBuffer);
            }
         }
         else
         {
            throw new UnsupportedOperationException("Unsupported compression: " + compression);
         }

         recordsCompressedData.order(ByteOrder.LITTLE_ENDIAN);
      }

      return recordsCompressedData.duplicate();
   }

   @Override
   public ByteBuffer getRecordsUncompressedBuffer(boolean directBuffer)
   {
      MCAPByteBufferDataOutput recordsOutput = new MCAPByteBufferDataOutput((int) records.getElementLength(), 2, directBuffer);
      records.forEach(element -> element.write(recordsOutput));
      recordsOutput.close();
      return recordsOutput.getBuffer();
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putLong(messageStartTime());
      dataOutput.putLong(messageEndTime());
      dataOutput.putLong(recordsUncompressedLength());
      dataOutput.putUnsignedInt(uncompressedCRC32());
      dataOutput.putString(compression.getName());
      ByteBuffer recordsCompressedBuffer = getRecordsCompressedBuffer();
      dataOutput.putLong(recordsCompressedBuffer.remaining());
      dataOutput.putByteBuffer(recordsCompressedBuffer);
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addLong(messageStartTime());
      crc32.addLong(messageEndTime());
      crc32.addLong(recordsUncompressedLength());
      crc32.addUnsignedInt(uncompressedCRC32());
      crc32.addString(compression.getName());
      ByteBuffer recordsCompressedBuffer = getRecordsCompressedBuffer();
      crc32.addLong(recordsCompressedBuffer.remaining());
      crc32.addByteBuffer(recordsCompressedBuffer);
      return crc32;
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

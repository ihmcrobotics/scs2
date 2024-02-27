package us.ihmc.scs2.session.mcap.specs.records;

import com.github.luben.zstd.ZstdCompressCtx;
import us.ihmc.scs2.session.mcap.encoding.LZ4FrameEncoder;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.output.MCAPByteBufferDataOutput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Objects;

public class MutableChunk implements Chunk
{
   private long messageStartTime;
   private long messageEndTime;
   private Compression compression = Compression.LZ4;

   private Records records;
   private long recordsCRC32 = -1L;
   private ByteBuffer recordsCompressedData;

   public void setMessageStartTime(long messageStartTime)
   {
      this.messageStartTime = messageStartTime;
   }

   public void setMessageEndTime(long messageEndTime)
   {
      this.messageEndTime = messageEndTime;
   }

   public void setCompression(Compression compression)
   {
      this.compression = compression;
   }

   public void setRecords(Records records)
   {
      this.records = records;
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
      return records.getElementLength();
   }

   @Override
   public long uncompressedCRC32()
   {
      return records == null ? 0 : records.updateCRC(null).getValue();
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

      if (recordsCompressedData == null || recordsCRC32 != newRecordsCRC32)
      {
         recordsCRC32 = newRecordsCRC32;
         Objects.requireNonNull(compression, "The compression has not been set yet.");
         Objects.requireNonNull(records, "The records have not been set yet.");

         ByteBuffer uncompressedBuffer = getRecordsUncompressedBuffer(compression == Compression.ZSTD);

         recordsCompressedData = switch (compression)
         {
            case NONE:
            {
               yield uncompressedBuffer;
            }
            case LZ4:
            {
               LZ4FrameEncoder lz4FrameEncoder = new LZ4FrameEncoder();
               yield lz4FrameEncoder.encode(uncompressedBuffer, null);
            }
            case ZSTD:
            {
               try (ZstdCompressCtx zstdCompressCtx = new ZstdCompressCtx())
               {
                  yield zstdCompressCtx.compress(uncompressedBuffer);
               }
            }
         };
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
      dataOutput.putLong(messageStartTime);
      dataOutput.putLong(messageEndTime);
      dataOutput.putLong(records.getElementLength());
      dataOutput.putUnsignedInt(records.updateCRC(null).getValue());
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
      crc32.addLong(messageStartTime);
      crc32.addLong(messageEndTime);
      crc32.addLong(records.getElementLength());
      crc32.addUnsignedInt(records.updateCRC(null).getValue());
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

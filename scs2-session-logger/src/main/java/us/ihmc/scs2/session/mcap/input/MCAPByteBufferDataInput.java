package us.ihmc.scs2.session.mcap.input;

import com.github.luben.zstd.ZstdDecompressCtx;
import us.ihmc.scs2.session.mcap.LZ4FrameDecoder;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class MCAPByteBufferDataInput implements MCAPDataInput
{
   private final ByteBuffer buffer;

   MCAPByteBufferDataInput(ByteBuffer buffer)
   {
      this.buffer = buffer;
      buffer.order(ByteOrder.LITTLE_ENDIAN);
   }

   @Override
   public void position(long position)
   {
      buffer.position((int) position);
   }

   @Override
   public long position()
   {
      return buffer.position();
   }

   @Override
   public long size()
   {
      return buffer.limit();
   }

   @Override
   public long getLong()
   {
      return buffer.getLong();
   }

   @Override
   public int getInt()
   {
      return buffer.getInt();
   }

   @Override
   public short getShort()
   {
      return buffer.getShort();
   }

   @Override
   public byte getByte()
   {
      return buffer.get();
   }

   @Override
   public void getBytes(byte[] bytes)
   {
      buffer.get(bytes);
   }

   @Override
   public byte[] getBytes(long offset, int length)
   {
      byte[] bytes = new byte[length];
      buffer.get((int) offset, bytes);
      return bytes;
   }

   @Override
   public ByteBuffer getByteBuffer(long offset, int length, boolean direct)
   {
      ByteBuffer out = direct ? ByteBuffer.allocateDirect(length) : ByteBuffer.allocate(length);
      out.put(0, buffer, (int) offset, length);
      out.order(ByteOrder.LITTLE_ENDIAN);
      return out;
   }

   @Override
   public ByteBuffer getDecompressedByteBuffer(long offset, int compressedLength, int uncompressedLength, Compression compression, boolean direct)
   {
      ByteBuffer compressedBuffer = getByteBuffer(offset, compressedLength, false);
      ByteBuffer decompressedBuffer;

      if (compression == Compression.LZ4)
      {
         LZ4FrameDecoder lz4FrameDecoder = new LZ4FrameDecoder();
         decompressedBuffer = direct ? ByteBuffer.allocateDirect(uncompressedLength) : ByteBuffer.allocate(uncompressedLength);
         lz4FrameDecoder.decode(buffer, (int) offset, compressedLength, decompressedBuffer, 0);
      }
      else if (compression == Compression.ZSTD)
      {
         try (ZstdDecompressCtx zstdDecompressCtx = new ZstdDecompressCtx())
         {
            int previousPosition = buffer.position();
            int previousLimit = buffer.limit();
            buffer.limit((int) (offset + compressedLength));
            buffer.position((int) offset);
            decompressedBuffer = zstdDecompressCtx.decompress(compressedBuffer, uncompressedLength);
            buffer.position(previousPosition);
            buffer.limit(previousLimit);
         }
      }
      else
      {
         throw new IllegalArgumentException("Unsupported compression: " + compression);
      }

      decompressedBuffer.order(ByteOrder.LITTLE_ENDIAN);
      return decompressedBuffer;
   }

   @Override
   public String toString()
   {
      return "MCAPByteBufferDataInput{" + "buffer=" + buffer + '}';
   }
}

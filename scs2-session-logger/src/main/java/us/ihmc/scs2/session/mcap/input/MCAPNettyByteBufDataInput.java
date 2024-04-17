package us.ihmc.scs2.session.mcap.input;

import com.github.luben.zstd.ZstdDecompressCtx;
import io.netty.buffer.ByteBuf;
import us.ihmc.scs2.session.mcap.encoding.LZ4FrameDecoder;
import us.ihmc.scs2.session.mcap.specs.records.Compression;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class MCAPNettyByteBufDataInput implements MCAPDataInput
{
   private final ByteBuf buffer;
   private long position;

   public MCAPNettyByteBufDataInput(ByteBuf buffer)
   {
      this.buffer = buffer;
   }

   @Override
   public void position(long position)
   {
      this.position = position;
   }

   @Override
   public long position()
   {
      return position;
   }

   @Override
   public long size()
   {
      return buffer.readableBytes();
   }

   @Override
   public long getLong()
   {
      long value = buffer.getLongLE((int) position);
      position += Long.BYTES;
      return value;
   }

   @Override
   public int getInt()
   {
      int value = buffer.getIntLE((int) position);
      position += Integer.BYTES;
      return value;
   }

   @Override
   public short getShort()
   {
      short value = buffer.getShortLE((int) position);
      position += Short.BYTES;
      return value;
   }

   @Override
   public byte getByte()
   {
      byte value = buffer.getByte((int) position);
      position++;
      return value;
   }

   @Override
   public void getBytes(byte[] bytes)
   {
      buffer.getBytes((int) position, bytes);
      position += bytes.length;
   }

   @Override
   public byte[] getBytes(long offset, int length)
   {
      byte[] bytes = new byte[length];
      buffer.getBytes((int) offset, bytes);
      return bytes;
   }

   @Override
   public ByteBuffer getByteBuffer(long offset, int length, boolean direct)
   {
      ByteBuffer out = direct ? ByteBuffer.allocateDirect(length) : ByteBuffer.allocate(length);
      buffer.getBytes((int) offset, out);
      out.position(0);
      out.limit(length);
      out.order(ByteOrder.LITTLE_ENDIAN);
      return out;
   }

   @Override
   public ByteBuffer getDecompressedByteBuffer(long offset, int compressedLength, int uncompressedLength, Compression compression, boolean direct)
   {
      ByteBuffer decompressedBuffer;

      if (compression == Compression.LZ4)
      {
         LZ4FrameDecoder lz4FrameDecoder = new LZ4FrameDecoder();
         decompressedBuffer = direct ? ByteBuffer.allocateDirect(uncompressedLength) : ByteBuffer.allocate(uncompressedLength);
         lz4FrameDecoder.decode(buffer.nioBuffer(), (int) offset, compressedLength, decompressedBuffer, 0);
      }
      else if (compression == Compression.ZSTD)
      {
         try (ZstdDecompressCtx zstdDecompressCtx = new ZstdDecompressCtx())
         {
            ByteBuffer compressedBuffer = buffer.nioBuffer((int) offset, compressedLength);
            decompressedBuffer = zstdDecompressCtx.decompress(compressedBuffer, uncompressedLength);
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

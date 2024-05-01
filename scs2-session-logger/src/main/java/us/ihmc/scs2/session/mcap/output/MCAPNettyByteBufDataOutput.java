package us.ihmc.scs2.session.mcap.output;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;

import java.nio.ByteBuffer;

public class MCAPNettyByteBufDataOutput implements MCAPDataOutput
{
   private static final boolean DEFAULT_DIRECT_BUFFER = false;

   private final ByteBuf buffer;

   public MCAPNettyByteBufDataOutput()
   {
      this(DEFAULT_DIRECT_BUFFER);
   }

   public MCAPNettyByteBufDataOutput(boolean directBuffer)
   {
      buffer = directBuffer ? Unpooled.directBuffer() : Unpooled.buffer();
   }

   public MCAPNettyByteBufDataOutput(ByteBuf buffer)
   {
      this.buffer = buffer;
   }

   @Override
   public long position()
   {
      return buffer.writerIndex();
   }

   @Override
   public void putLong(long value)
   {
      buffer.ensureWritable(Long.BYTES);
      buffer.writeLongLE(value);
   }

   @Override
   public void putInt(int value)
   {
      buffer.ensureWritable(Integer.BYTES);
      buffer.writeIntLE(value);
   }

   @Override
   public void putShort(short value)
   {
      buffer.ensureWritable(Short.BYTES);
      buffer.writeShortLE(value);
   }

   @Override
   public void putByte(byte value)
   {
      buffer.ensureWritable(Byte.BYTES);
      buffer.writeByte(value);
   }

   @Override
   public void putBytes(byte[] bytes, int offset, int length)
   {
      buffer.ensureWritable(length);
      buffer.writeBytes(bytes, offset, length);
   }

   @Override
   public void putByteBuffer(ByteBuffer byteBuffer)
   {
      buffer.ensureWritable(byteBuffer.remaining());
      buffer.writeBytes(byteBuffer);
   }

   @Override
   public void close()
   {

   }

   public ByteBuf getBuffer()
   {
      return buffer;
   }
}

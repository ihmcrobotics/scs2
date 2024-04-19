package us.ihmc.scs2.session.mcap.input;

import io.netty.buffer.ByteBuf;
import us.ihmc.scs2.session.mcap.specs.records.Compression;

import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

public interface MCAPDataInput
{

   void position(long position);

   long position();

   default void skip(long length)
   {
      position(position() + length);
   }

   long size();

   long getLong();

   int getInt();

   default long getUnsignedInt()
   {
      return Integer.toUnsignedLong(getInt());
   }

   short getShort();

   default int getUnsignedShort()
   {
      return Short.toUnsignedInt(getShort());
   }

   byte getByte();

   default int getUnsignedByte()
   {
      return Byte.toUnsignedInt(getByte());
   }

   void getBytes(byte[] bytes);

   default byte[] getBytes(int length)
   {
      byte[] bytes = new byte[length];
      getBytes(bytes);
      return bytes;
   }

   byte[] getBytes(long offset, int length);

   default String getString()
   {
      return new String(getBytes((int) getUnsignedInt()));
   }

   ByteBuffer getByteBuffer(long offset, int length, boolean direct);

   ByteBuffer getDecompressedByteBuffer(long offset, int compressedLength, int uncompressedLength, Compression compression, boolean direct);

   static MCAPDataInput wrap(FileChannel fileChannel)
   {
      return new MCAPBufferedFileChannelInput(fileChannel);
   }

   static MCAPDataInput wrap(ByteBuffer buffer)
   {
      return new MCAPByteBufferDataInput(buffer);
   }

   static MCAPDataInput wrap(ByteBuf buffer)
   {
      return new MCAPNettyByteBufDataInput(buffer);
   }
}

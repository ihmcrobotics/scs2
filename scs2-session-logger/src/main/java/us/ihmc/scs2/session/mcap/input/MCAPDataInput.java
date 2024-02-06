package us.ihmc.scs2.session.mcap.input;

import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

public interface MCAPDataInput
{
   enum Compression
   {
      NONE, LZ4, ZSTD;

      public static Compression fromString(String name)
      {
         return switch (name.trim().toLowerCase())
         {
            case "none", "" -> NONE;
            case "lz4" -> LZ4;
            case "zstd" -> ZSTD;
            default -> throw new IllegalArgumentException("Unsupported compression: " + name);
         };
      }
   }

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
}

package us.ihmc.scs2.session.mcap.output;

import us.ihmc.scs2.session.mcap.specs.records.MCAPElement;

import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.Collection;

public interface MCAPDataOutput
{
   long position();

   void putLong(long value);

   void putInt(int value);

   default void putUnsignedInt(long value)
   {
      putInt((int) value);
   }

   void putShort(short value);

   default void putUnsignedShort(int value)
   {
      putShort((short) value);
   }

   void putByte(byte value);

   default void putUnsignedByte(int value)
   {
      putByte((byte) value);
   }

   default void putBytes(byte[] bytes)
   {
      putBytes(bytes, 0, bytes.length);
   }

   void putBytes(byte[] bytes, int offset, int length);

   default void putString(String string)
   {
      putUnsignedInt(string.length());
      for (int i = 0; i < string.length(); i++)
         putByte((byte) string.charAt(i));
   }

   void putByteBuffer(ByteBuffer byteBuffer);

   default <T extends MCAPElement> void putCollection(Collection<T> collection)
   {
      putUnsignedInt(collection.stream().mapToLong(MCAPElement::getElementLength).sum());
      collection.forEach(element -> element.write(this));
   }

   default void putInputStream(InputStream inputStream)
   {
      byte[] buffer = new byte[8192];
      int bytesRead;
      try
      {
         while ((bytesRead = inputStream.read(buffer)) != -1)
            putBytes(buffer, 0, bytesRead);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   void close();

   static MCAPDataOutput wrap(FileChannel fileChannel)
   {
      return new MCAPBufferedFileChannelOutput(fileChannel);
   }
}

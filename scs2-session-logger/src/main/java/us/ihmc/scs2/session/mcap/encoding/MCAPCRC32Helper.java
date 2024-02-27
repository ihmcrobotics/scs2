package us.ihmc.scs2.session.mcap.encoding;

import us.ihmc.scs2.session.mcap.specs.records.MCAPElement;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Collection;
import java.util.zip.CRC32;

public class MCAPCRC32Helper
{
   private final CRC32 crc32 = new CRC32();
   private final ByteBuffer buffer = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);

   public MCAPCRC32Helper()
   {
   }

   public void reset()
   {
      crc32.reset();
   }

   public void addLong(long value)
   {
      buffer.clear();
      buffer.putLong(value);
      buffer.flip();
      addByteBuffer(buffer);
   }

   public void addInt(int value)
   {
      buffer.clear();
      buffer.putInt(value);
      buffer.flip();
      addByteBuffer(buffer);
   }

   public void addUnsignedInt(long value)
   {
      addInt((int) value);
   }

   public void addShort(short value)
   {
      buffer.clear();
      buffer.putShort(value);
      buffer.flip();
      addByteBuffer(buffer);
   }

   public void addUnsignedShort(int value)
   {
      addShort((short) value);
   }

   public void addByte(byte value)
   {
      crc32.update(value);
   }

   public void addUnsignedByte(int value)
   {
      addByte((byte) value);
   }

   public void addBytes(byte[] bytes)
   {
      crc32.update(bytes);
   }

   public void addBytes(byte[] bytes, int offset, int length)
   {
      crc32.update(bytes, offset, length);
   }

   public void addByteBuffer(ByteBuffer byteBuffer)
   {
      crc32.update(byteBuffer);
   }

   public void addString(String value)
   {
      byte[] bytes = value.getBytes();
      addUnsignedInt(bytes.length);
      addBytes(bytes);
   }

   public <T extends MCAPElement> void addCollection(Collection<T> collection)
   {
      addUnsignedInt(collection.stream().mapToLong(MCAPElement::getElementLength).sum());
      collection.forEach(element -> element.updateCRC(this));
   }

   public long getValue()
   {
      return crc32.getValue();
   }
}

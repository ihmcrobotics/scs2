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

   public MCAPCRC32Helper reset()
   {
      crc32.reset();
      return this;
   }

   public MCAPCRC32Helper addLong(long value)
   {
      buffer.clear();
      buffer.putLong(value);
      buffer.flip();
      return addByteBuffer(buffer);
   }

   public MCAPCRC32Helper addInt(int value)
   {
      buffer.clear();
      buffer.putInt(value);
      buffer.flip();
      return addByteBuffer(buffer);
   }

   public MCAPCRC32Helper addUnsignedInt(long value)
   {
      return addInt((int) value);
   }

   public MCAPCRC32Helper addShort(short value)
   {
      buffer.clear();
      buffer.putShort(value);
      buffer.flip();
      return addByteBuffer(buffer);
   }

   public MCAPCRC32Helper addUnsignedShort(int value)
   {
      return addShort((short) value);
   }

   public MCAPCRC32Helper addByte(byte value)
   {
      crc32.update(value);
      return this;
   }

   public MCAPCRC32Helper addUnsignedByte(int value)
   {
      return addByte((byte) value);
   }

   public MCAPCRC32Helper addBytes(byte[] bytes)
   {
      crc32.update(bytes);
      return this;
   }

   public MCAPCRC32Helper addBytes(byte[] bytes, int offset, int length)
   {
      crc32.update(bytes, offset, length);
      return this;
   }

   public MCAPCRC32Helper addByteBuffer(ByteBuffer byteBuffer)
   {
      crc32.update(byteBuffer);
      return this;
   }

   public MCAPCRC32Helper addString(String value)
   {
      byte[] bytes = value.getBytes();
      addUnsignedInt(bytes.length);
      return addBytes(bytes);
   }

   public <T extends MCAPElement> MCAPCRC32Helper addCollection(Collection<T> collection)
   {
      addUnsignedInt(collection.stream().mapToLong(MCAPElement::getElementLength).sum());
      return addHeadlessCollection(collection);
   }

   public <T extends MCAPElement> MCAPCRC32Helper addHeadlessCollection(Collection<T> collection)
   {
      collection.forEach(element -> element.updateCRC(this));
      return this;
   }

   public long getValue()
   {
      return crc32.getValue();
   }
}

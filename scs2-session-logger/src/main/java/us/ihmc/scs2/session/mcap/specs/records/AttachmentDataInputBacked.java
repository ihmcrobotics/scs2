package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;

class AttachmentDataInputBacked implements Attachment
{
   private final MCAPDataInput dataInput;
   private final long logTime;
   private final long createTime;
   private final String name;
   private final String mediaType;
   private final long dataLength;
   private final long dataOffset;
   private WeakReference<byte[]> dataRef;
   private final long crc32;
   private final long crc32InputStart;
   private final int crc32InputLength;
   private WeakReference<ByteBuffer> crc32InputRef;

   AttachmentDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;

      dataInput.position(elementPosition);
      crc32InputStart = elementPosition;
      logTime = MCAP.checkPositiveLong(dataInput.getLong(), "logTime");
      createTime = MCAP.checkPositiveLong(dataInput.getLong(), "createTime");
      name = dataInput.getString();
      mediaType = dataInput.getString();
      dataLength = MCAP.checkPositiveLong(dataInput.getLong(), "lengthData");
      dataOffset = dataInput.position();
      dataInput.skip(dataLength);
      crc32InputLength = (int) (dataInput.position() - elementPosition);
      crc32 = dataInput.getUnsignedInt();
      MCAP.checkLength(elementLength, getElementLength());
   }

   /**
    * Retrieves the bytes to use for recomputing the CRC32.
    */
   public ByteBuffer crc32Input()
   {
      ByteBuffer crc32Input = this.crc32InputRef == null ? null : this.crc32InputRef.get();

      if (crc32Input == null)
      {
         crc32Input = dataInput.getByteBuffer(crc32InputStart, crc32InputLength, false);
         crc32InputRef = new WeakReference<>(crc32Input);
      }

      return crc32Input;
   }

   @Override
   public long logTime()
   {
      return logTime;
   }

   @Override
   public long createTime()
   {
      return createTime;
   }

   @Override
   public String name()
   {
      return name;
   }

   @Override
   public String mediaType()
   {
      return mediaType;
   }

   public long dataOffset()
   {
      return dataOffset;
   }

   @Override
   public long dataLength()
   {
      return dataLength;
   }

   @Override
   public byte[] data()
   {
      byte[] data = this.dataRef == null ? null : this.dataRef.get();

      if (data == null)
      {
         data = dataInput.getBytes(dataOffset, (int) dataLength);
         dataRef = new WeakReference<>(data);
      }
      return data;
   }

   @Override
   public long crc32()
   {
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
      return object instanceof Attachment attachment && Attachment.super.equals(attachment);
   }
}

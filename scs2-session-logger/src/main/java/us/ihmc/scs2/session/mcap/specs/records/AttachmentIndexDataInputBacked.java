package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;

class AttachmentIndexDataInputBacked implements AttachmentIndex
{
   private final MCAPDataInput dataInput;
   private final long attachmentOffset;
   private final long attachmentLength;
   private final long logTime;
   private final long createTime;
   private final long dataLength;
   private final String name;
   private final String mediaType;

   private WeakReference<Record> attachmentRef;

   AttachmentIndexDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;

      dataInput.position(elementPosition);
      attachmentOffset = MCAP.checkPositiveLong(dataInput.getLong(), "attachmentOffset");
      attachmentLength = MCAP.checkPositiveLong(dataInput.getLong(), "attachmentLength");
      logTime = MCAP.checkPositiveLong(dataInput.getLong(), "logTime");
      createTime = MCAP.checkPositiveLong(dataInput.getLong(), "createTime");
      dataLength = MCAP.checkPositiveLong(dataInput.getLong(), "dataSize");
      name = dataInput.getString();
      mediaType = dataInput.getString();
      MCAP.checkLength(elementLength, getElementLength());
   }

   @Override
   public Record attachment()
   {
      Record attachment = attachmentRef == null ? null : attachmentRef.get();

      if (attachment == null)
      {
         attachment = new RecordDataInputBacked(dataInput, attachmentOffset);
         attachmentRef = new WeakReference<>(attachment);
      }

      return attachment;
   }

   @Override
   public long attachmentOffset()
   {
      return attachmentOffset;
   }

   @Override
   public long attachmentLength()
   {
      return attachmentLength;
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
   public long dataLength()
   {
      return dataLength;
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

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof AttachmentIndex other && AttachmentIndex.super.equals(other);
   }
}

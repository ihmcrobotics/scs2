package us.ihmc.scs2.session.mcap.specs.records;

public class MutableAttachmentIndex implements AttachmentIndex
{
   private long attachmentOffset;
   private long attachmentLength;
   private long logTime;
   private long createTime;
   private long dataLength;
   private String name;
   private String mediaType;
   private Record attachment;

   @Override
   public Record attachment()
   {
      return attachment;
   }

   public void setAttachment(Record attachment)
   {
      this.attachment = attachment;
      Attachment attachmentBody = attachment.body();
      attachmentLength = attachment.getElementLength();
      logTime = attachmentBody.logTime();
      createTime = attachmentBody.createTime();
      dataLength = attachmentBody.dataLength();
      name = attachmentBody.name();
      mediaType = attachmentBody.mediaType();
   }

   @Override
   public long attachmentOffset()
   {
      return attachmentOffset;
   }

   public void setAttachmentOffset(long attachmentOffset)
   {
      this.attachmentOffset = attachmentOffset;
   }

   @Override
   public long attachmentLength()
   {
      return attachmentLength;
   }

   public void setAttachmentLength(long attachmentLength)
   {
      this.attachmentLength = attachmentLength;
   }

   @Override
   public long logTime()
   {
      return logTime;
   }

   public void setLogTime(long logTime)
   {
      this.logTime = logTime;
   }

   @Override
   public long createTime()
   {
      return createTime;
   }

   public void setCreateTime(long createTime)
   {
      this.createTime = createTime;
   }

   @Override
   public long dataLength()
   {
      return dataLength;
   }

   public void setDataLength(long dataLength)
   {
      this.dataLength = dataLength;
   }

   @Override
   public String name()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   @Override
   public String mediaType()
   {
      return mediaType;
   }

   public void setMediaType(String mediaType)
   {
      this.mediaType = mediaType;
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

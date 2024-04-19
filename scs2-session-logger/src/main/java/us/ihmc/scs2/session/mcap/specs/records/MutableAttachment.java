package us.ihmc.scs2.session.mcap.specs.records;

public class MutableAttachment implements Attachment
{
   private long logTime;
   private long createTime;
   private String name;
   private String mediaType;
   private long dataLength;

   private byte[] data;
   private long crc32;

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
   public long dataLength()
   {
      return dataLength;
   }

   public void setDataLength(long dataLength)
   {
      this.dataLength = dataLength;
   }

   @Override
   public byte[] data()
   {
      return data;
   }

   public void setData(byte[] data)
   {
      this.data = data;
   }

   @Override
   public long crc32()
   {
      return crc32;
   }

   public void setCRC32(long crc32)
   {
      this.crc32 = crc32;
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

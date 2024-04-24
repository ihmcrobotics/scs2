package us.ihmc.robotDataLogger.websocket.mcap;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.scs2.session.mcap.specs.records.Attachment;

public class WebsocketResourcesAttachment implements Attachment
{
   public static final int RESOURCES_LOG_TIME = -41;
   public static final int RESOURCES_CREATE_TIME = -40;
   public static final String RESOURCES_MEDIA_TYPE = "application/zip";
   public static final String RESOURCES_NAME_SUFFIX = "-resources.zip";

   private String name;
   private byte[] data;
   private long crc32;

   public WebsocketResourcesAttachment()
   {
   }

   public static WebsocketResourcesAttachment create(LogModelProvider logModelProvider)
   {
      if (logModelProvider == null || logModelProvider.getResourceZip() == null)
         return null;

      WebsocketResourcesAttachment attachment = new WebsocketResourcesAttachment();
      attachment.name = logModelProvider.getModelName() + RESOURCES_NAME_SUFFIX;
      attachment.data = logModelProvider.getResourceZip();
      attachment.crc32 = attachment.updateCRC(null).getValue();
      return attachment;
   }

   public static WebsocketResourcesAttachment toWebsocketResourcesAttachment(Attachment attachment)
   {
      if (attachment instanceof WebsocketResourcesAttachment)
      {
         return (WebsocketResourcesAttachment) attachment;
      }
      else if (attachment.name().endsWith(RESOURCES_NAME_SUFFIX) && attachment.mediaType().equals(RESOURCES_MEDIA_TYPE)
               && attachment.logTime() == RESOURCES_LOG_TIME && attachment.createTime() == RESOURCES_CREATE_TIME)
      {
         WebsocketResourcesAttachment resourcesAttachment = new WebsocketResourcesAttachment();
         resourcesAttachment.name = attachment.name();
         resourcesAttachment.data = attachment.data();
         resourcesAttachment.crc32 = attachment.crc32();
         return resourcesAttachment;
      }

      return null;
   }

   @Override
   public long logTime()
   {
      return RESOURCES_LOG_TIME;
   }

   @Override
   public long createTime()
   {
      return RESOURCES_CREATE_TIME;
   }

   @Override
   public String name()
   {
      return name;
   }

   @Override
   public String mediaType()
   {
      return RESOURCES_MEDIA_TYPE;
   }

   @Override
   public long dataLength()
   {
      return data.length;
   }

   @Override
   public byte[] data()
   {
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

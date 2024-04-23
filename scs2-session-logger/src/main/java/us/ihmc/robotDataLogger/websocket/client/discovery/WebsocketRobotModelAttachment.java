package us.ihmc.robotDataLogger.websocket.client.discovery;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.scs2.session.mcap.specs.records.Attachment;

public class WebsocketRobotModelAttachment implements Attachment
{
   public static final int ROBOT_MODEL_LOG_TIME = -31;
   public static final int ROBOT_MODEL_CREATE_TIME = -30;
   public static final String ROBOT_MODEL_PREFIX_MEDIA_TYPE = "model/";
   public static final String ROBOT_MODEL_URDF_MEDIA_TYPE = ROBOT_MODEL_PREFIX_MEDIA_TYPE + "urdf";
   public static final String ROBOT_MODEL_SDF_MEDIA_TYPE = ROBOT_MODEL_PREFIX_MEDIA_TYPE + "sdf";

   private String name;
   private String mediaType;
   private byte[] data;
   private long crc32;

   public WebsocketRobotModelAttachment()
   {

   }

   public static WebsocketRobotModelAttachment create(LogModelProvider logModelProvider)
   {
      if (logModelProvider == null || logModelProvider.getModel() == null)
         return null;

      WebsocketRobotModelAttachment attachment = new WebsocketRobotModelAttachment();
      attachment.name = logModelProvider.getModelName();
      if (logModelProvider.getLoader() == null || logModelProvider.getLoader().getSimpleName().toLowerCase().contains("urdf"))
         attachment.mediaType = ROBOT_MODEL_URDF_MEDIA_TYPE;
      else
         attachment.mediaType = ROBOT_MODEL_SDF_MEDIA_TYPE;
      attachment.data = logModelProvider.getModel();
      attachment.crc32 = attachment.updateCRC(null).getValue();
      return attachment;
   }

   public static WebsocketRobotModelAttachment toWebsocketRobotModelAttachment(Attachment attachment)
   {
      if (attachment instanceof WebsocketRobotModelAttachment)
      {
         return (WebsocketRobotModelAttachment) attachment;
      }
      else if (attachment.mediaType().startsWith(ROBOT_MODEL_PREFIX_MEDIA_TYPE) && attachment.logTime() == ROBOT_MODEL_LOG_TIME
               && attachment.createTime() == ROBOT_MODEL_CREATE_TIME)
      {
         WebsocketRobotModelAttachment robotModelAttachment = new WebsocketRobotModelAttachment();
         robotModelAttachment.name = attachment.name();
         robotModelAttachment.mediaType = attachment.mediaType();
         robotModelAttachment.data = attachment.data();
         robotModelAttachment.crc32 = attachment.crc32();
         return robotModelAttachment;
      }

      return null;
   }

   @Override
   public long logTime()
   {
      return ROBOT_MODEL_LOG_TIME;
   }

   @Override
   public long createTime()
   {
      return ROBOT_MODEL_CREATE_TIME;
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

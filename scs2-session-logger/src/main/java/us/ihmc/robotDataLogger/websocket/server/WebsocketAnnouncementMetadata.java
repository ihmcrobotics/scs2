package us.ihmc.robotDataLogger.websocket.server;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.scs2.session.mcap.specs.records.Metadata;
import us.ihmc.scs2.session.mcap.specs.records.MetadataMap;

import java.util.Objects;

public class WebsocketAnnouncementMetadata implements Metadata
{
   public static final String NAME = "announcement";

   public static final String METADATA_SERVER_NAME_KEY = "serverName";
   public static final String METADATA_SERVER_VERSION_KEY = "serverVersion";
   public static final String METADATA_IS_LOGGING_SESSION_KEY = "isLoggingSession";
   public static final String METADATA_HOST_NAME_KEY = "hostName";
   public static final String METADATA_PORT_KEY = "port";
   public static final String METADATA_HAS_ROBOT_MODEL_KEY = "hasRobotModel";
   public static final String METADATA_HAS_RESOURCES_KEY = "hasResources";

   private MetadataMap metadata;

   public WebsocketAnnouncementMetadata()
   {
   }

   public static WebsocketAnnouncementMetadata create(String serverName,
                                                      String serverVersion,
                                                      String hostName,
                                                      LogModelProvider logModelProvider,
                                                      DataServerSettings serverSettings)
   {
      WebsocketAnnouncementMetadata announcement = new WebsocketAnnouncementMetadata();
      announcement.metadata = new MetadataMap();
      announcement.metadata.put(METADATA_SERVER_NAME_KEY, serverName);
      announcement.metadata.put(METADATA_SERVER_VERSION_KEY, serverVersion);
      announcement.metadata.put(METADATA_IS_LOGGING_SESSION_KEY, Boolean.toString(serverSettings.isLogSession()));
      announcement.metadata.put(METADATA_HOST_NAME_KEY, hostName);
      announcement.metadata.put(METADATA_PORT_KEY, Integer.toString(serverSettings.getPort()));
      announcement.metadata.put(METADATA_HAS_ROBOT_MODEL_KEY, Boolean.toString(logModelProvider != null && logModelProvider.getModel() != null));
      announcement.metadata.put(METADATA_HAS_RESOURCES_KEY, Boolean.toString(logModelProvider != null && logModelProvider.getResourceZip() != null));
      return announcement;
   }

   public static WebsocketAnnouncementMetadata toWebsocketAnnouncementMetadata(Metadata metadata)
   {
      if (metadata instanceof WebsocketAnnouncementMetadata)
      {
         return (WebsocketAnnouncementMetadata) metadata;
      }
      else if (Objects.equals(metadata.name(), NAME))
      {
         WebsocketAnnouncementMetadata announcement = new WebsocketAnnouncementMetadata();
         announcement.metadata = metadata.metadata();
         return announcement;
      }

      return null;
   }

   public String getServerName()
   {
      return metadata.get(METADATA_SERVER_NAME_KEY);
   }

   public String getServerVersion()
   {
      return metadata.get(METADATA_SERVER_VERSION_KEY);
   }

   public String getHostName()
   {
      return metadata.get(METADATA_HOST_NAME_KEY);
   }

   public int getPort()
   {
      return Integer.parseInt(metadata.get(METADATA_PORT_KEY));
   }

   public boolean isLoggingSession()
   {
      return Boolean.parseBoolean(metadata.get(METADATA_IS_LOGGING_SESSION_KEY));
   }

   public boolean hasRobotModel()
   {
      return Boolean.parseBoolean(metadata.get(METADATA_HAS_ROBOT_MODEL_KEY));
   }

   public boolean hasResources()
   {
      return Boolean.parseBoolean(metadata.get(METADATA_HAS_RESOURCES_KEY));
   }

   @Override
   public String name()
   {
      return NAME;
   }

   @Override
   public MetadataMap metadata()
   {
      return metadata;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Metadata other && equals(other);
   }
}

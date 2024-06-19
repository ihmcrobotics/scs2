package us.ihmc.robotDataLogger;

import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.util.DaemonThreadFactory;
import us.ihmc.robotDataLogger.websocket.client.MCAPWebsocketDataConsumer;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPMCAPServerConnection;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.robotDataLogger.websocket.interfaces.ConnectionStateListener;
import us.ihmc.robotDataLogger.websocket.interfaces.DataServerCommandConsumer;
import us.ihmc.robotDataLogger.websocket.interfaces.MCAPRecordListener;
import us.ihmc.robotDataLogger.websocket.interfaces.MCAPStarterConsumer;
import us.ihmc.robotDataLogger.websocket.mcap.WebsocketMCAPStarter;
import us.ihmc.robotDataLogger.websocket.server.WebsocketAnnouncementMetadata;
import us.ihmc.robotDataLogger.websocket.server.WebsocketChannelStarterChunk;
import us.ihmc.robotDataLogger.websocket.server.WebsocketSchemaStarterChunk;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.records.Channel;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Schema;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Consumer;

public class MCAPVariableClient
{
   public static final int DEFAULT_TIMEOUT = 25000; //ms

   private String serverName;

   // Command executor
   private final Executor commandExecutor = Executors.newSingleThreadExecutor(DaemonThreadFactory.getNamedDaemonThreadFactory(getClass().getSimpleName()));

   private MCAPWebsocketDataConsumer dataConsumer;

   // Callbacks
   private TimestampListener timestampListener;
   private MCAPStarterConsumer starterMCAPConsumer;
   private MCAPRecordListener recordConsumer;
   private ConnectionStateListener connectionStateListener;
   private DataServerCommandConsumer dataServerCommandConsumer;

   public MCAPVariableClient()
   {
   }

   public void setTimestampListener(TimestampListener timestampListener)
   {
      this.timestampListener = timestampListener;
   }

   public void setStarterMCAPConsumer(MCAPStarterConsumer starterMCAPConsumer)
   {
      this.starterMCAPConsumer = starterMCAPConsumer;
   }

   public void setRecordConsumer(MCAPRecordListener recordConsumer)
   {
      this.recordConsumer = recordConsumer;
   }

   public void setConnectionStateListener(ConnectionStateListener connectionStateListener)
   {
      this.connectionStateListener = connectionStateListener;
   }

   public void setDataServerCommandConsumer(DataServerCommandConsumer dataServerCommandConsumer)
   {
      this.dataServerCommandConsumer = dataServerCommandConsumer;
   }

   /**
    * Start a logger connecting to a specified host.
    *
    * @param host
    * @param port
    */
   public void start(String host, int port)
   {
      try
      {
         HTTPMCAPServerConnection connection = HTTPMCAPServerConnection.connect(host, port);
         start(DEFAULT_TIMEOUT, connection);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Start the logger re-using an already existing HTTPDataServerConnection This method is used by the
    * logger and the GUI popup to avoid an extra connection. This saves some object allocations on the
    * server side
    *
    * @param timeout    Timeout for requesting resources
    * @param connection An existing HTTPDataServerConnection
    * @throws IOException
    */
   public synchronized void start(int timeout, HTTPMCAPServerConnection connection) throws IOException
   {
      if (dataConsumer != null)
      {
         throw new RuntimeException("Client already started");
      }

      WebsocketMCAPStarter mcapStarter = connection.getMCAPStarter();

      dataConsumer = new MCAPWebsocketDataConsumer(connection, timeout);
      WebsocketAnnouncementMetadata announcementMetadata = mcapStarter.announcementMetadata();
      serverName = announcementMetadata.getServerName();

      if (announcementMetadata.hasResources())
         mcapStarter.setResourcesAttachment(dataConsumer.getResourceAttachment());

      receivedStarterMCAP(mcapStarter);

      if (dataConsumer.isSessionActive())
         throw new RuntimeException("Client already connected");
      if (dataConsumer.isClosed())
         throw new RuntimeException("Client has closed completely");

      dataConsumer.setTimestampListener(this::receivedTimestamp);
      dataConsumer.setSingleRecordConsumer(this::receivedRecord);
      dataConsumer.setDataServerCommandConsumer(this::receivedCommand);
      dataConsumer.setConnectionStateListener(new ConnectionStateListener()
      {
         @Override
         public void connected()
         {
            if (connectionStateListener != null)
               connectionStateListener.connected();
         }

         @Override
         public void connectionClosed()
         {
            if (connectionStateListener != null)
               connectionStateListener.connectionClosed();
         }
      });

      dataConsumer.startSession();
   }

   /**
    * Stops the client completely. The participant leaves the domain and a reconnect is not possible.
    */
   public synchronized void stop()
   {
      if (dataConsumer == null)
      {
         throw new RuntimeException("Session not started");
      }
      dataConsumer.close();
      dataConsumer = null;
   }

   public void sendClearLogRequest()
   {
      try
      {
         dataConsumer.sendCommand(DataServerCommand.CLEAR_LOG, 0);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void receivedTimestamp(long timestamp)
   {
      if (timestampListener != null)
      {
         timestampListener.receivedTimestampOnly(timestamp);
      }
   }

   private void receivedStarterMCAP(WebsocketMCAPStarter mcap)
   {
      if (starterMCAPConsumer != null)
      {
         starterMCAPConsumer.accept(mcap);
      }
   }

   public void sendRecord(Record record)
   {
      if (dataConsumer == null)
         throw new RuntimeException("Session not started");

      dataConsumer.sendRecord(record);
   }

   private void receivedRecord(long timestamp, Record record)
   {
      if (recordConsumer != null)
      {
         recordConsumer.accept(timestamp, record);
      }
   }

   private void receivedCommand(DataServerCommand command, int argument)
   {
      if (dataServerCommandConsumer != null)
      {
         dataServerCommandConsumer.receivedCommand(command, argument);
      }
   }

   /**
    * Reconnect to the same session. This will work as long as the IP, port, controller name and
    * complete variable registry match.
    *
    * @throws IOException
    */
   public synchronized boolean reconnect() throws IOException
   {
      if (dataConsumer == null)
      {
         throw new RuntimeException("Session not started");
      }

      return dataConsumer.reconnect();
   }

   /**
    * Disconnect and cleanup
    */
   public void disconnect()
   {
      if (dataConsumer != null)
      {
         dataConsumer.disconnectSession();
      }
   }

   public boolean isConnected()
   {
      return dataConsumer != null && dataConsumer.isSessionActive();
   }

   public String getServerName()
   {
      return serverName;
   }

   public static class YoRegistryManager implements MCAPStarterConsumer, MCAPRecordListener
   {
      private final YoRegistry rootRegistry;
      private final List<YoVariable> channelIDToVariable = new ArrayList<>();
      private final List<Consumer<CDRDeserializer>> channelIDToVariableUpdater = new ArrayList<>();

      public YoRegistryManager(YoRegistry rootRegistry)
      {
         this.rootRegistry = rootRegistry;
      }

      @Override
      public void accept(WebsocketMCAPStarter newMCAP)
      {
         List<Class<? extends YoVariable>> variableClasses = new ArrayList<>();

         WebsocketSchemaStarterChunk schemaStarterChunk = newMCAP.schemaStarterChunk();
         for (Record record : schemaStarterChunk.records())
         {
            Schema schema = record.body();
            Class<? extends YoVariable> variableClass = MCAPBuilder.getVariableTypeFromSchemaName(schema.name());

            if (variableClass == null)
               continue;

            while (variableClasses.size() < schema.id())
               variableClasses.add(null);
            variableClasses.set(schema.id() - 1, variableClass);
         }

         WebsocketChannelStarterChunk channelStarterChunk = newMCAP.channelStarterChunk();
         for (Record record : channelStarterChunk.records())
         {
            Channel channel = record.body();
            int schemaID = channel.schemaId();

            if (schemaID < 0 || schemaID >= variableClasses.size())
               continue;

            Class<? extends YoVariable> variableClass = variableClasses.get(schemaID);
            if (variableClass == null)
               continue;

            String fullname = channel.topic().replace('/', YoTools.NAMESPACE_SEPERATOR);
            // TODO Need to handle enums
            YoVariable yoVariable = SharedMemoryTools.ensureYoVariableExists(rootRegistry, fullname, variableClass);
            if (yoVariable == null)
               continue;

            int channelId = channel.id();
            while (channelIDToVariable.size() <= channelId)
               channelIDToVariable.add(null);
            channelIDToVariable.set(channelId, yoVariable);

            while (channelIDToVariableUpdater.size() <= channelId)
               channelIDToVariableUpdater.add(null);

            if (yoVariable instanceof YoDouble yoDouble)
               channelIDToVariableUpdater.set(channelId, deserializer -> yoDouble.set(deserializer.read_float64()));
            else if (yoVariable instanceof YoBoolean yoBoolean)
               channelIDToVariableUpdater.set(channelId, deserializer -> yoBoolean.set(deserializer.read_bool()));
            else if (yoVariable instanceof YoInteger yoInteger)
               channelIDToVariableUpdater.set(channelId, deserializer -> yoInteger.set(deserializer.read_int32()));
            else if (yoVariable instanceof YoLong yoLong)
               channelIDToVariableUpdater.set(channelId, deserializer -> yoLong.set(deserializer.read_int64()));
            else if (yoVariable instanceof YoEnum<?> yoEnum)
               channelIDToVariableUpdater.set(channelId, deserializer -> yoEnum.set(deserializer.read_uint8()));
         }
      }

      public static YoVariable createYoVariable(YoRegistry rootRegistry, String variableFullname, Schema schema)
      {
         YoVariable variable = rootRegistry.findVariable(variableFullname);
         if (variable != null)
            return variable;
         YoNamespace yoNamespace = new YoNamespace(variableFullname);
         YoRegistry parentRegistry = SharedMemoryTools.ensurePathExists(rootRegistry, yoNamespace.getParent());

         String name = schema.name();
         if (name.equals("YoDouble"))
            return new YoDouble(yoNamespace.getShortName(), parentRegistry);
         if (name.equals("YoInteger"))
            return new YoInteger(yoNamespace.getShortName(), parentRegistry);
         if (name.equals("YoLong"))
            return new YoLong(yoNamespace.getShortName(), parentRegistry);
         if (name.equals("YoBoolean"))
            return new YoBoolean(yoNamespace.getShortName(), parentRegistry);
         if (name.startsWith("YoEnum"))
         {
            String schemaContent = new String(schema.dataArray(), 0, (int) schema.dataLength());
            int enumSize = Integer.parseInt(schemaContent.substring("uint8 SIZE=".length(), schemaContent.indexOf('\n')));
            String[] enumConstants = new String[enumSize];
            schemaContent = schemaContent.substring(schemaContent.indexOf('\n') + 1);
            for (int i = 0; i < enumSize; i++)
            {
               String enumConstantName = schemaContent.substring("uint8 ".length(), schemaContent.indexOf('=')).trim();
               int enumConstantOrdinal = Integer.parseInt(schemaContent.substring(schemaContent.indexOf('=') + 1, schemaContent.indexOf('\n')).trim());
               enumConstants[enumConstantOrdinal] = enumConstantName;
               schemaContent = schemaContent.substring(schemaContent.indexOf('\n') + 1);
            }
            boolean allowNull = Boolean.parseBoolean(schemaContent.substring("bool ALLOW_NULL=".length(), schemaContent.indexOf('\n')).trim());
            return new YoEnum<>(yoNamespace.getShortName(), "", parentRegistry, allowNull, enumConstants);
         }
         return null;
      }

      private final CDRDeserializer deserializer = new CDRDeserializer();

      @Override
      public void accept(long timestamp, Record record)
      {
         if (record.op() == Opcode.MESSAGE)
         {
            Message message = record.body();
            deserializer.initialize(message.messageBuffer());
            int channelId = message.channelId();
            Consumer<CDRDeserializer> variableUpdater = channelIDToVariableUpdater.get(channelId);
            if (variableUpdater != null)
               variableUpdater.accept(deserializer);
         }
      }
   }
}

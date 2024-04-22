package us.ihmc.robotDataLogger;

import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.util.DaemonThreadFactory;
import us.ihmc.robotDataLogger.websocket.client.MCAPWebsocketDataConsumer;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPMCAPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.robotDataLogger.websocket.dataBuffers.ConnectionStateListener;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPRegistryConsumer.MCAPConsumer;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPRegistryConsumer.MCAPSingleRecordConsumer;
import us.ihmc.robotDataLogger.websocket.server.MCAPDataServerServerContent;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Attachment;
import us.ihmc.scs2.session.mcap.specs.records.Metadata;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.scs2.session.mcap.specs.records.Record;

import java.io.IOException;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/**
 * Main entry point to write a client to the data server
 *
 * @author Jesper Smith
 */
public class YoMCAPVariableClient
{
   public static final int DEFAULT_TIMEOUT = 25000; //ms

   private String serverName;

   // Command executor
   private final Executor commandExecutor = Executors.newSingleThreadExecutor(DaemonThreadFactory.getNamedDaemonThreadFactory(getClass().getSimpleName()));

   private MCAPWebsocketDataConsumer dataConsumer;

   // Callbacks
   private TimestampListener timestampListener;
   private MCAPConsumer starterMCAPConsumer;
   private MCAPSingleRecordConsumer recordConsumer;
   private ConnectionStateListener connectionStateListener;

   public YoMCAPVariableClient()
   {
   }

   public void setTimestampListener(TimestampListener timestampListener)
   {
      this.timestampListener = timestampListener;
   }

   public void setStarterMCAPConsumer(MCAPConsumer starterMCAPConsumer)
   {
      this.starterMCAPConsumer = starterMCAPConsumer;
   }

   public void setRecordConsumer(MCAPSingleRecordConsumer recordConsumer)
   {
      this.recordConsumer = recordConsumer;
   }

   public void setConnectionStateListener(ConnectionStateListener connectionStateListener)
   {
      this.connectionStateListener = connectionStateListener;
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
         HTTPMCAPDataServerConnection connection = HTTPMCAPDataServerConnection.connect(host, port);
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
   public synchronized void start(int timeout, HTTPMCAPDataServerConnection connection) throws IOException
   {
      if (dataConsumer != null)
      {
         throw new RuntimeException("Client already started");
      }

      MCAP mcapStarter = connection.getMCAPStarter();

      dataConsumer = new MCAPWebsocketDataConsumer(connection, timeout);
      serverName = ((Metadata) mcapStarter.findMetadata(MCAPDataServerServerContent.ANNOUNCEMENT_METADATA_NAME).get(0).body()).metadata().get("name");

      if (dataConsumer.isSessionActive())
         throw new RuntimeException("Client already connected");
      if (dataConsumer.isClosed())
         throw new RuntimeException("Client has closed completely");

      int insertionPoint = mcapStarter.records().indexOf(mcapStarter.dataEnd());
      Attachment resourceAttachment = dataConsumer.getResourceAttachment();
      if (resourceAttachment != null)
         mcapStarter.records().add(insertionPoint, new MutableRecord(resourceAttachment));
      receivedStarterMCAP(mcapStarter);
      dataConsumer.startSession(this::receivedTimestamp, this::receivedRecord, new ConnectionStateListener()
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

   private void receivedStarterMCAP(MCAP mcap)
   {
      if (starterMCAPConsumer != null)
      {
         starterMCAPConsumer.accept(mcap);
      }
   }

   private void receivedRecord(long timestamp, Record record)
   {
      if (recordConsumer != null)
      {
         recordConsumer.accept(timestamp, record);
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

   void receivedCommand(DataServerCommand command, int argument)
   {
      // FIXME
      //      commandExecutor.execute(() -> yoVariablesUpdatedListener.receivedCommand(command, argument));
   }

   public boolean isConnected()
   {
      return dataConsumer != null && dataConsumer.isSessionActive();
   }

   public String getServerName()
   {
      return serverName;
   }
}

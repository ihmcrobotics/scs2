package us.ihmc.robotDataLogger.websocket.client;

import io.netty.buffer.ByteBuf;
import io.netty.util.CharsetUtil;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.websocket.HTTPDataServerPaths;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.robotDataLogger.websocket.dataBuffers.ConnectionStateListener;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPRegistryConsumer.MCAPRecordConsumer;

import java.io.IOException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

public class MCAPWebsocketDataConsumer
{
   private final Object lock = new Object();
   private HTTPDataServerConnection connection;

   private MCAPWebsocketDataServerClient session;
   private boolean closed = false;

   private final int timeoutInMs;

   private TimestampListener timestampListener;
   private MCAPRecordConsumer recordConsumer;
   private ConnectionStateListener connectionStateListener;

   public MCAPWebsocketDataConsumer(HTTPDataServerConnection initialConnection, int timeoutInMs)
   {
      connection = initialConnection;
      this.timeoutInMs = timeoutInMs;
   }

   private ByteBuf getResource(String path) throws IOException
   {
      synchronized (lock)
      {
         if (!connection.isConnected())
         {
            throw new IOException("Not connected");
         }

         Future<ByteBuf> resourceFuture = connection.requestResource(path);

         try
         {
            return resourceFuture.get(timeoutInMs, TimeUnit.MILLISECONDS);
         }
         catch (Exception e)
         {
            throw new IOException(e);
         }
      }
   }

   public byte[] getModelFile() throws IOException
   {
      ByteBuf model = getResource(HTTPDataServerPaths.model);
      byte[] retVal = new byte[model.readableBytes()];
      model.readBytes(retVal);

      return retVal;
   }

   public byte[] getResourceZip() throws IOException
   {
      ByteBuf resourceZip = getResource(HTTPDataServerPaths.resources);
      byte[] retVal = new byte[resourceZip.readableBytes()];
      resourceZip.readBytes(retVal);
      return retVal;
   }

   public Handshake getHandshake() throws IOException
   {
      ByteBuf handshake = getResource(HTTPDataServerPaths.handshake);

      JSONSerializer<Handshake> serializer = new JSONSerializer<>(new HandshakePubSubType());
      return serializer.deserialize(handshake.toString(CharsetUtil.UTF_8));
   }

   public void startSession(TimestampListener timestampListener, MCAPRecordConsumer recordConsumer, ConnectionStateListener connectionStateListener)
         throws IOException
   {
      synchronized (lock)
      {
         if (!connection.isConnected())
         {
            throw new IOException("Not connected");
         }

         connection.take();
         this.timestampListener = timestampListener;
         this.recordConsumer = recordConsumer;
         this.connectionStateListener = connectionStateListener;

         session = new MCAPWebsocketDataServerClient(connection, timestampListener, recordConsumer, connectionStateListener, timeoutInMs);
      }
   }

   public boolean isSessionActive()
   {
      synchronized (lock)
      {
         if (session == null)
         {
            return false;
         }
         else
         {
            return session.isActive();
         }
      }
   }

   public void disconnectSession()
   {
      synchronized (lock)
      {

         if (session == null)
         {
            throw new RuntimeException("Session not started");
         }

         session.close();
      }
   }

   public void close()
   {
      synchronized (lock)
      {

         if (connection.isConnected())
         {
            connection.close();
         }

         if (session != null)
         {
            if (session.isActive())
            {
               session.close();
            }
         }

         closed = true;
      }
   }

   public boolean isClosed()
   {
      synchronized (lock)
      {
         return closed;
      }
   }

   public boolean reconnect() throws IOException
   {
      synchronized (lock)
      {
         if (session != null && session.isActive())
         {
            throw new RuntimeException("Session is still active");
         }

         try
         {
            HTTPDataServerDescription oldDescription = connection.getTarget();
            HTTPDataServerConnection newConnection = HTTPDataServerConnection.connect(oldDescription.getHost(), oldDescription.getPort());
            newConnection.close();

            Announcement announcement = newConnection.getAnnouncement();
            Announcement oldAnnouncement = connection.getAnnouncement();
            if (announcement.getReconnectKeyAsString().equals(oldAnnouncement.getReconnectKeyAsString()))
            {
               connection = newConnection;
               session = new MCAPWebsocketDataServerClient(connection, timestampListener, recordConsumer, connectionStateListener, timeoutInMs);
               return true;
            }
            else
            {
               return false;
            }
         }
         catch (IOException e)
         {
            System.err.println(e.getMessage());
            return false;
         }
      }
   }

   // FIXME Fix this implementation
   public void writeVariableChangeRequest(int identifier, double valueAsDouble)
   {
      synchronized (lock)
      {
         if (session != null && session.isActive())
         {
            session.writeVariableChangeRequest(identifier, valueAsDouble);
         }
      }
   }

   public void sendCommand(DataServerCommand command, int argument) throws IOException
   {
      synchronized (lock)
      {
         if (session != null && session.isActive())
         {
            session.sendCommand(command, argument);
         }
      }
   }
}

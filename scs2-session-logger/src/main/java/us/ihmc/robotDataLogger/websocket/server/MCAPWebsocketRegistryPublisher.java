package us.ihmc.robotDataLogger.websocket.server;

import io.netty.channel.EventLoopGroup;
import io.netty.util.concurrent.ScheduledFuture;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.robotDataLogger.dataBuffers.CustomLogDataPublisherType;
import us.ihmc.robotDataLogger.dataBuffers.LoggerDebugRegistry;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBuffer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.interfaces.BufferListenerInterface;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;

import java.util.concurrent.TimeUnit;

/**
 * Publishing thread for registry data This thread reads all variables on a realtime thread, puts
 * them in a ConcurrentRingBuffer and periodically sends them out on the websocket connection.
 *
 * @author Jesper Smith
 */
class MCAPWebsocketRegistryPublisher implements RegistryPublisher
{
   private static final int BUFFER_CAPACITY = 128;

   private long uid = 0;
   private final ConcurrentRingBuffer<RegistrySendBuffer> ringBuffer;

   private final WebsocketDataBroadcaster broadcaster;
   private final LoggerDebugRegistry loggerDebugRegistry;

   private final EventLoopGroup eventLoopGroup;

   private final VariableUpdateThread variableUpdateThread = new VariableUpdateThread();

   private final CustomLogDataPublisherType publisherType;
   private final SerializedPayload serializedPayload;

   private ScheduledFuture<?> scheduledFuture;

   private final int numberOfVariables;

   private final int bufferID;

   private final BufferListenerInterface bufferListener;

   public MCAPWebsocketRegistryPublisher(EventLoopGroup workerGroup,
                                         RegistrySendBufferBuilder builder,
                                         WebsocketDataBroadcaster broadcaster,
                                         int bufferID,
                                         BufferListenerInterface bufferListener)
   {
      this.broadcaster = broadcaster;

      ringBuffer = new ConcurrentRingBuffer<>(builder, BUFFER_CAPACITY);
      eventLoopGroup = workerGroup;

      loggerDebugRegistry = builder.getLoggerDebugRegistry();
      numberOfVariables = builder.getNumberOfVariables();

      this.bufferID = bufferID;

      publisherType = new CustomLogDataPublisherType(builder.getNumberOfVariables(), builder.getNumberOfJointStates());

      serializedPayload = new SerializedPayload(publisherType.getMaximumTypeSize());

      this.bufferListener = bufferListener;

      if (bufferListener != null)
      {
         bufferListener.addBuffer(bufferID, builder);
      }
   }

   public int getMaximumBufferSize()
   {
      return publisherType.getMaximumTypeSize();
   }

   /**
    * Starts the registry publisher and schedules it on the main eventLoopGroup
    */
   @Override
   public void start()
   {
      scheduledFuture = eventLoopGroup.scheduleAtFixedRate(variableUpdateThread, 0, 1, TimeUnit.MILLISECONDS);
   }

   @Override
   public void stop()
   {
      scheduledFuture.cancel(false);

      try
      {
         scheduledFuture.await(5, TimeUnit.SECONDS);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void update(long timestamp)
   {
      RegistrySendBuffer buffer = ringBuffer.next();
      if (buffer != null)
      {
         buffer.updateBufferFromVariables(timestamp, uid);
         ringBuffer.commit();
      }
      else
      {
         loggerDebugRegistry.circularBufferFull();
      }

      uid++;
   }

   private class VariableUpdateThread implements Runnable
   {
      private long previousUid = -1;

      private VariableUpdateThread()
      {

      }

      @Override
      public void run()
      {
         try
         {
            while (ringBuffer.poll())
            {
               RegistrySendBuffer buffer;

               if ((buffer = ringBuffer.read()) != null)
               {
                  serializedPayload.getData().clear();
                  publisherType.serialize(buffer, serializedPayload);
                  broadcaster.write(bufferID, buffer.getTimestamp(), serializedPayload.getData());

                  if (previousUid != -1)
                  {
                     if (buffer.getUid() != previousUid + 1)
                     {
                        loggerDebugRegistry.lostTickInCircularBuffer();
                     }
                  }
                  previousUid = buffer.getUid();

                  if (bufferListener != null)
                  {
                     bufferListener.updateBuffer(bufferID, buffer);
                  }
               }

               if (bufferListener != null)
               {
                  while ((buffer = ringBuffer.read()) != null)
                  {
                     bufferListener.updateBuffer(bufferID, buffer);
                  }
               }
               ringBuffer.flush();
            }
         }
         catch (Throwable e)
         {
            e.printStackTrace();
         }
      }
   }
}

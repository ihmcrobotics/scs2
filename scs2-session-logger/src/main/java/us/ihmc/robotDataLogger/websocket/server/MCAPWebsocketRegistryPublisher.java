package us.ihmc.robotDataLogger.websocket.server;

import io.netty.channel.EventLoopGroup;
import io.netty.util.concurrent.ScheduledFuture;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotDataLogger.dataBuffers.LoggerDebugRegistry;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;
import us.ihmc.robotDataLogger.websocket.server.dataBuffers.MCAPRegistrySendBuffer;
import us.ihmc.robotDataLogger.websocket.server.dataBuffers.MCAPRegistrySendBufferBuilder;

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

   private long sequenceID = 0;
   private final ConcurrentRingBuffer<MCAPRegistrySendBuffer> ringBuffer;

   private final MCAPWebsocketDataBroadcaster broadcaster;
   private final LoggerDebugRegistry loggerDebugRegistry;

   private final EventLoopGroup eventLoopGroup;

   private final VariableUpdateThread variableUpdateThread = new VariableUpdateThread();

   private ScheduledFuture<?> scheduledFuture;

   private final int bufferID;

   public MCAPWebsocketRegistryPublisher(EventLoopGroup workerGroup,
                                         MCAPRegistrySendBufferBuilder builder,
                                         MCAPWebsocketDataBroadcaster broadcaster,
                                         int bufferID)
   {
      this.broadcaster = broadcaster;

      ringBuffer = new ConcurrentRingBuffer<>(builder, BUFFER_CAPACITY);
      eventLoopGroup = workerGroup;

      loggerDebugRegistry = builder.getLoggerDebugRegistry();

      this.bufferID = bufferID;
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
      MCAPRegistrySendBuffer buffer = ringBuffer.next();
      if (buffer != null)
      {
         buffer.update(timestamp, sequenceID);
         ringBuffer.commit();
      }
      else
      {
         loggerDebugRegistry.circularBufferFull();
      }

      sequenceID++;
   }

   private class VariableUpdateThread implements Runnable
   {
      private long previousSequenceID = -1;

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
               MCAPRegistrySendBuffer buffer;

               if ((buffer = ringBuffer.read()) != null)
               {
                  broadcaster.write(bufferID, buffer.getTimestamp(), buffer.getBuffer());

                  if (previousSequenceID != -1 && buffer.getSequenceID() != previousSequenceID + 1)
                     loggerDebugRegistry.lostTickInCircularBuffer();

                  previousSequenceID = buffer.getSequenceID();
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

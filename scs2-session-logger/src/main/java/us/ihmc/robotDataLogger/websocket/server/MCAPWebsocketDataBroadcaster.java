package us.ihmc.robotDataLogger.websocket.server;

import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelFutureListener;
import us.ihmc.robotDataLogger.util.PaddedVolatileBoolean;
import us.ihmc.robotDataLogger.util.PaddedVolatileLong;
import us.ihmc.robotDataLogger.util.PaddedVolatileReference;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.concurrent.locks.LockSupport;

/**
 * Helper class that keep track of active connections and writes data to all connections. It also
 * has an internal thread for the timestamp publisher to avoid the overhead of having a thread per
 * connection for timestamp publishing
 *
 * @author Jesper Smith
 */
public class MCAPWebsocketDataBroadcaster implements ChannelFutureListener
{
   /**
    * How much time the timestamp publishing thread should wait between successive polling for a new
    * timestamp.
    */
   public static final int TIMESTAMP_PUBLISHING_SLEEP_NS = 500000;

   private final Object channelLock = new Object();
   private final TimestampPublishingThread timestampPublishingThread = new TimestampPublishingThread();

   // Implement channels as copy on write array to avoid blocking in the timestamp update thread

   private final PaddedVolatileReference<MCAPWebsocketDataServerFrameHandler[]> channels = new PaddedVolatileReference<>(new MCAPWebsocketDataServerFrameHandler[0]);

   private final PaddedVolatileBoolean active = new PaddedVolatileBoolean(true);
   private final PaddedVolatileLong newTimestamp = new PaddedVolatileLong(Long.MIN_VALUE);

   public MCAPWebsocketDataBroadcaster()
   {
      timestampPublishingThread.start();
   }

   public void addClient(MCAPWebsocketDataServerFrameHandler websocketLogFrameHandler)
   {
      synchronized (channelLock)
      {

         MCAPWebsocketDataServerFrameHandler[] newChannels = Arrays.copyOf(channels.get(), channels.get().length + 1);
         newChannels[newChannels.length - 1] = websocketLogFrameHandler;
         channels.set(newChannels);

         websocketLogFrameHandler.addCloseFutureListener(this);
      }
   }

   public void write(int bufferID, long timestamp, ByteBuffer frame) throws IOException
   {
      // Localize channels
      MCAPWebsocketDataServerFrameHandler[] localChannels = channels.get();

      for (int i = 0; i < localChannels.length; i++)
      {
         localChannels[i].write(bufferID, timestamp, frame);
      }
   }

   /**
    * Remove channel on completion
    */
   @Override
   public void operationComplete(ChannelFuture future) throws Exception
   {
      synchronized (channelLock)
      {
         Channel channel = future.channel();
         MCAPWebsocketDataServerFrameHandler[] oldChannels = channels.get();
         MCAPWebsocketDataServerFrameHandler[] newChannels = new MCAPWebsocketDataServerFrameHandler[oldChannels.length - 1];

         int newI = 0;
         for (int i = 0; i < oldChannels.length; i++)
         {
            if (oldChannels[i].channel() == channel)
            {
               oldChannels[i].release();
            }
            else
            {
               if (newI >= newChannels.length)
               {
                  // Channel not found, returning
                  return;
               }

               newChannels[newI] = oldChannels[i];
               ++newI;
            }
         }

         channels.set(newChannels);
      }
   }

   public void writeCommand(DataServerCommand command, int argument)
   {
      // Localize channels
      MCAPWebsocketDataServerFrameHandler[] localChannels = channels.get();

      for (int i = 0; i < localChannels.length; i++)
      {
         localChannels[i].writeCommand(command, argument);
      }
   }

   public void publishTimestamp(long timestamp)
   {
      newTimestamp.set(timestamp);
   }

   public void stop()
   {
      active.set(false);
   }

   /**
    * Internal thread that sends timestamps over a UDP connection. Polls for new timestamps at
    * approximately 10kHz
    *
    * @author Jesper Smith
    */
   private class TimestampPublishingThread extends Thread
   {

      private TimestampPublishingThread()
      {
         super(TimestampPublishingThread.class.getSimpleName());
      }

      @Override
      public void run()
      {
         long lastSendTimestamp = Long.MIN_VALUE;

         while (active.getBoolean())
         {
            // Localize variable so it doesn't change in this thread
            long newTimestampLocal = newTimestamp.getLong();

            if (lastSendTimestamp != newTimestampLocal)
            {

               // Localize variables
               MCAPWebsocketDataServerFrameHandler[] localChannels = channels.get();

               for (int i = 0; i < localChannels.length; i++)
               {
                  localChannels[i].publishTimestamp(newTimestampLocal);
               }

               lastSendTimestamp = newTimestampLocal;
            }

            LockSupport.parkNanos(TIMESTAMP_PUBLISHING_SLEEP_NS);
         }
      }
   }
}

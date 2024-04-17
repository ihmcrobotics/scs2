package us.ihmc.robotDataLogger.websocket.dataBuffers;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.scs2.session.mcap.specs.records.Record;

import java.util.concurrent.PriorityBlockingQueue;

public class MCAPRegistryConsumer extends Thread
{
   private final static int MAXIMUM_ELEMENTS = 4096;

   //   private final ConcurrentSkipListSet<MCAPRegistryReceiveBuffer> orderedBuffers = new ConcurrentSkipListSet<>();
   private final PriorityBlockingQueue<MCAPRegistryReceiveBuffer> orderedBuffers = new PriorityBlockingQueue<>();
   private volatile boolean running = true;

   private final MCAPRecordConsumer recordConsumer;
   private final ConnectionStateListener connectionStateListener;

   public MCAPRegistryConsumer(MCAPRecordConsumer recordConsumer, ConnectionStateListener connectionStateListener)
   {
      this.recordConsumer = recordConsumer;
      this.connectionStateListener = connectionStateListener;

      start();
   }

   @Override
   public void run()
   {
      while (running)
      {
         ThreadTools.sleep(1);

         while (!orderedBuffers.isEmpty())
         {
            try
            {
               handlePackets();
            }
            catch (InterruptedException e)
            {
               // Try next time
            }
         }
      }

      // Empty buffer
      while (!orderedBuffers.isEmpty())
      {
         try
         {
            handlePackets();
         }
         catch (InterruptedException e)
         {
            // Try next time
         }
      }

      connectionStateListener.connectionClosed();
   }

   public void stopImmediately()
   {
      running = false;
   }

   private void handlePackets() throws InterruptedException
   {
      MCAPRegistryReceiveBuffer buffer = orderedBuffers.take();
      recordConsumer.accept(buffer.getReceivedTimestamp(), buffer.getRecord());
   }

   public void onNewDataMessage(MCAPRegistryReceiveBuffer buffer)
   {
      if (orderedBuffers.size() < MAXIMUM_ELEMENTS)
         orderedBuffers.add(buffer);
      else
         System.out.println("Dropping packet");
   }

   public interface MCAPRecordConsumer
   {
      void accept(long timestamp, Record newRecord);
   }
}

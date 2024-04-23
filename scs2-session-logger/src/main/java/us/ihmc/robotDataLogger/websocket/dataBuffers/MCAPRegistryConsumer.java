package us.ihmc.robotDataLogger.websocket.dataBuffers;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotDataLogger.websocket.client.discovery.WebsocketMCAPStarter;
import us.ihmc.scs2.session.mcap.specs.records.Record;

import java.util.concurrent.PriorityBlockingQueue;

public class MCAPRegistryConsumer extends Thread
{
   private final static int MAXIMUM_ELEMENTS = 4096;

   private final PriorityBlockingQueue<MCAPRegistryReceiveBuffer> orderedBuffers = new PriorityBlockingQueue<>();
   private volatile boolean running = true;

   private final MCAPSingleRecordConsumer singleRecordConsumer;
   private final ConnectionStateListener connectionStateListener;

   public MCAPRegistryConsumer(MCAPSingleRecordConsumer singleRecordConsumer, ConnectionStateListener connectionStateListener)
   {
      this.singleRecordConsumer = singleRecordConsumer;
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
      singleRecordConsumer.accept(buffer.getReceivedTimestamp(), buffer.getRecord());
   }

   public void onNewDataMessage(MCAPRegistryReceiveBuffer buffer)
   {
      if (orderedBuffers.size() < MAXIMUM_ELEMENTS)
         orderedBuffers.add(buffer);
      else
         System.out.println("Dropping packet");
   }

   public interface MCAPConsumer
   {
      void accept(WebsocketMCAPStarter newMCAP);
   }

   public interface MCAPSingleRecordConsumer
   {
      void accept(long timestamp, Record newRecord);
   }
}

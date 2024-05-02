package us.ihmc.robotDataLogger.websocket.dataBuffers;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotDataLogger.websocket.interfaces.ConnectionStateListener;
import us.ihmc.robotDataLogger.websocket.interfaces.MCAPRecordListener;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.RecordDataInputBacked;

import java.util.concurrent.PriorityBlockingQueue;

public class MCAPDataReceiveScheduler extends Thread
{
   private final static int MAXIMUM_ELEMENTS = 4096;

   private final PriorityBlockingQueue<MCAPData> orderedMCAPData = new PriorityBlockingQueue<>();
   private volatile boolean running = true;

   private final MCAPRecordListener mcapRecordListener;
   private final ConnectionStateListener connectionStateListener;

   public MCAPDataReceiveScheduler(MCAPRecordListener mcapRecordListener, ConnectionStateListener connectionStateListener)
   {
      this.mcapRecordListener = mcapRecordListener;
      this.connectionStateListener = connectionStateListener;

      start();
   }

   @Override
   public void run()
   {
      while (running)
      {
         ThreadTools.sleep(1);

         while (!orderedMCAPData.isEmpty())
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
      while (!orderedMCAPData.isEmpty())
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
      MCAPData buffer = orderedMCAPData.take();
      mcapRecordListener.accept(buffer.getReceivedTimestamp(), buffer.getRecord());
   }

   public void onNewData(MCAPDataInput dataInput)
   {
      if (orderedMCAPData.size() < MAXIMUM_ELEMENTS)
         orderedMCAPData.add(new MCAPData(System.nanoTime(), dataInput));
      else
         System.out.println("Dropping packet");
   }

   private static class MCAPData implements Comparable<MCAPData>
   {
      private final long receivedTimestamp;
      private final Record record;

      public MCAPData(long receivedTimestamp, MCAPDataInput dataInput)
      {
         this.receivedTimestamp = receivedTimestamp;
         record = new RecordDataInputBacked(dataInput, 0);
      }

      public long getReceivedTimestamp()
      {
         return receivedTimestamp;
      }

      public Record getRecord()
      {
         return record;
      }

      @Override
      public int compareTo(MCAPData other)
      {
         if (other == null)
            return 1;
         long thisTimestamp = extractTimestamp(record);
         long otherTimestamp = extractTimestamp(other.getRecord());
         return Long.compare(thisTimestamp, otherTimestamp);
      }

      private static long extractTimestamp(Record record)
      {
         if (record == null || record.op() == null)
            return Long.MAX_VALUE;
         if (record.op() == Opcode.CHUNK)
            return ((Chunk) record.body()).messageStartTime();
         else if (record.op() == Opcode.MESSAGE)
            return ((Message) record.body()).publishTime();
         return Long.MAX_VALUE;
      }
   }
}

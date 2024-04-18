package us.ihmc.robotDataLogger.websocket.dataBuffers;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.RecordDataInputBacked;

public class MCAPRegistryReceiveBuffer implements Comparable<MCAPRegistryReceiveBuffer>
{
   private final long receivedTimestamp;
   private final Record record;

   public MCAPRegistryReceiveBuffer(long receivedTimestamp, MCAPDataInput dataInput)
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
   public int compareTo(MCAPRegistryReceiveBuffer other)
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

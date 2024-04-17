package us.ihmc.robotDataLogger.websocket.dataBuffers;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.RecordDataInputBacked;

public class MCAPRegistryReceiveBuffer
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
}

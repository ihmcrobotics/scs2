package us.ihmc.robotDataLogger.websocket.server;

import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.MutableChunk;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.scs2.session.mcap.specs.records.Record;

public class MCAPHandShakeBuilder
{
   private final MCAPBuilder mcapBuilder;

   private final MutableChunk handShakeChunk = new MutableChunk();
   private final Record handShakeRecord = new MutableRecord(handShakeChunk);

   public MCAPHandShakeBuilder(MCAPBuilder mcapBuilder)
   {
      this.mcapBuilder = mcapBuilder;
   }
}

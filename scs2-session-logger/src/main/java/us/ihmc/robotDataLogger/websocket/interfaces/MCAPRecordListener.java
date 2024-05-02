package us.ihmc.robotDataLogger.websocket.interfaces;

import us.ihmc.scs2.session.mcap.specs.records.Record;

public interface MCAPRecordListener
{
   void accept(long timestamp, Record newRecord);
}

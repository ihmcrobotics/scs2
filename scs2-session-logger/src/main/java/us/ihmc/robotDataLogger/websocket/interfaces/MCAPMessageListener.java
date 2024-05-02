package us.ihmc.robotDataLogger.websocket.interfaces;

import us.ihmc.scs2.session.mcap.specs.records.Message;

public interface MCAPMessageListener
{
   void onMessage(Message message);
}

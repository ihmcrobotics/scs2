package us.ihmc.robotDataLogger.websocket.interfaces;

import us.ihmc.robotDataLogger.websocket.mcap.WebsocketMCAPStarter;

public interface MCAPStarterConsumer
{
   void accept(WebsocketMCAPStarter newMCAP);
}

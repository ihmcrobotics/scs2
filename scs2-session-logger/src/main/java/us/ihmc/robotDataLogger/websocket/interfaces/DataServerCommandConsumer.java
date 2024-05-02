package us.ihmc.robotDataLogger.websocket.interfaces;

import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

public interface DataServerCommandConsumer
{
   void receivedCommand(DataServerCommand command, int argument);
}

package us.ihmc.robotDataLogger.websocket.interfaces;

public interface ConnectionStateListener
{
   void connected();

   void connectionClosed();
}

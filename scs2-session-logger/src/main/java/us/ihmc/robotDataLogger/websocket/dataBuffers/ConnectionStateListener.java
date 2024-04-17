package us.ihmc.robotDataLogger.websocket.dataBuffers;

public interface ConnectionStateListener
{
   void connected();

   void connectionClosed();
}

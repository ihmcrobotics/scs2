package us.ihmc.scs2.sessionVisualizer.jfx.session.remote;

import us.ihmc.robotDataLogger.interfaces.DataServerDiscoveryListener;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;

@FunctionalInterface
public interface FunctionalDataServerDiscoveryListener extends DataServerDiscoveryListener
{
   void connectionStateUpdated(HTTPDataServerConnection connection);

   @Override
   default void connected(HTTPDataServerConnection connection)
   {
      connectionStateUpdated(connection);
   }

   @Override
   default void disconnected(HTTPDataServerConnection connection)
   {
      connectionStateUpdated(connection);
   }
}

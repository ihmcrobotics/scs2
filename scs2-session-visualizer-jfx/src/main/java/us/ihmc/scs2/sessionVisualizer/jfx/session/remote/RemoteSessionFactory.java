package us.ihmc.scs2.sessionVisualizer.jfx.session.remote;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.scs2.session.remote.RemoteSession;
import us.ihmc.scs2.session.remote.SimpleYoVariablesUpdatedListener;

public class RemoteSessionFactory implements SimpleYoVariablesUpdatedListener
{
   static
   {
      // RegistryConsumer's adaptive jitter buffer (RFC-1889) is sized for the lossy UDP realtime logger,
      // where it can grow to thousands of frames (tens of seconds of latency). A live SCS2 viewer connects
      // over an ordered, reliable TCP websocket, so it needs essentially no reordering buffer. Without this,
      // the estimate saturates at its cap and pins the displayed delay at ~30s permanently. Default the cap
      // low for the viewer while still allowing an explicit -Dscs2.remote.maxJitterBufferSamples override.
      if (System.getProperty("scs2.remote.maxJitterBufferSamples") == null)
         System.setProperty("scs2.remote.maxJitterBufferSamples", "10");
   }

   private final ObjectProperty<RemoteSession> activeSessionProperty = new SimpleObjectProperty<>(this, "activeSession", null);

   public RemoteSessionFactory()
   {
   }

   @Override
   public void start(YoVariableClientInterface yoVariableClientInterface,
                     LogHandshake handshake,
                     YoVariableHandshakeParser handshakeParser,
                     DebugRegistry debugRegistry)
   {
      activeSessionProperty.set(new RemoteSession(yoVariableClientInterface, handshake, handshakeParser, debugRegistry));
   }

   @Override
   public void receivedTimestampAndData(long timestamp)
   {
      RemoteSession activeSession = activeSessionProperty.get();
      if (activeSession != null)
         activeSession.receivedTimestampAndData(timestamp);
   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
      RemoteSession activeSession = activeSessionProperty.get();
      if (activeSession != null)
         activeSession.receivedTimestampOnly(timestamp);
   }

   @Override
   public void receivedCommand(DataServerCommand command, int argument)
   {
      RemoteSession activeSession = activeSessionProperty.get();
      if (activeSession != null)
         activeSession.receivedCommand(command, argument);
   }

   public void unloadSession()
   {
      RemoteSession activeSession = activeSessionProperty.get();

      if (activeSession != null)
      {
         // FIXME Session management is a mess, shouldn't be the responsibility of the factory
         //         activeSession.shutdownSession();
         activeSessionProperty.set(null);
      }
   }

   public ReadOnlyObjectProperty<RemoteSession> activeSessionProperty()
   {
      return activeSessionProperty;
   }
}

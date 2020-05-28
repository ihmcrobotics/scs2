package us.ihmc.scs2;

import us.ihmc.messager.Messager;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.SessionVisualizerTopics;
import us.ihmc.scs2.websocket.JavalinManager;
import us.ihmc.scs2.websocket.JavalinWebsocketHandler;

public class SessionSimpleControlsControllerJS
{
   public SessionSimpleControlsControllerJS(Messager messager, SessionVisualizerTopics topics, JavalinManager javalinManager)
   {
      JavalinWebsocketHandler handler = javalinManager.webSocket("/maincontrols");
      handler.addOnMessage(ctx ->
      {
         System.out.println("Received message for /maincontrols: " + ctx.message());
         switch (ctx.message())
         {
            case "requestSimulate":
               messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.RUNNING);
               break;
            case "requestPlayback":
               messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PLAYBACK);
               break;
            case "requestPause":
               messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
               break;
         }
      });
   }
}

package us.ihmc.scs2;

import us.ihmc.scs2.session.Session;

public class SessionVisualizerJS
{
   private SessionVisualizerToolkitJS toolkit;
   private MultiSessionManagerJS multiSessionManager;

   public SessionVisualizerJS()
   {
      toolkit = new SessionVisualizerToolkitJS();
      toolkit.getJavalinManager().start(4567);
      multiSessionManager = new MultiSessionManagerJS(toolkit);
      new SessionSimpleControlsControllerJS(toolkit.getMessager(), toolkit.getTopics(), toolkit.getJavalinManager());
   }

   public void startSession(Session session)
   {
      startSession(session, null);
   }

   public void startSession(Session session, Runnable sessionLoadedCallback)
   {
      multiSessionManager.startSession(session, sessionLoadedCallback);
      toolkit.getJavalinManager().startBrowser(); // TODO Not a good place
   }

   public void stopSession()
   {
      multiSessionManager.stopSession();
   }

   public static void startSessionVisualizer(Session session)
   {
      SessionVisualizerJS sessionVisualizer = new SessionVisualizerJS();
      sessionVisualizer.startSession(session);
   }

   public static void main(String[] args)
   {
   }
}

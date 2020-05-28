package us.ihmc.scs2;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMessagerAPI;
import us.ihmc.scs2.session.SessionState;
import us.ihmc.scs2.session.YoSharedBufferMessagerAPI;
import us.ihmc.scs2.sessionVisualizer.SessionVisualizerMessagerAPI;
import us.ihmc.scs2.sessionVisualizer.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.managers.BackgroundExecutorManager;
import us.ihmc.scs2.sessionVisualizer.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.managers.YoManager;
import us.ihmc.scs2.websocket.JavalinManager;

public class SessionVisualizerToolkitJS
{
   private final Messager messager;
   private final SessionVisualizerTopics topics = new SessionVisualizerTopics();

   private final YoManager yoManager = new YoManager();
   // TODO Chart managers
   // TODO YoGraphic manager
   // TODO YoCompositeSearchManager
   // TODO Snapshot manager
   // TODO Key frame manager

   private final JavalinManager javalinManager = new JavalinManager();
   private final BackgroundExecutorManager backgroundExecutorManager = new BackgroundExecutorManager(4);
   private final EnvironmentManagerJS environmentManager = new EnvironmentManagerJS(javalinManager, backgroundExecutorManager);
   private final ReferenceFrameManager referenceFrameManager = new ReferenceFrameManager(backgroundExecutorManager);
   private final YoRobotJSManager yoRobotJSManager = new YoRobotJSManager(yoManager, javalinManager, referenceFrameManager, backgroundExecutorManager);

   public SessionVisualizerToolkitJS()
   {
      MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      apiFactory.createRootCategory("SCS2");
      apiFactory.includeMessagerAPIs(SessionMessagerAPI.API, YoSharedBufferMessagerAPI.API, SessionVisualizerMessagerAPI.API);
      messager = new BufferedMessager(apiFactory.getAPIAndCloseFactory());
      topics.setupTopics();
      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void startSession(Session session, Runnable sessionLoadedCallback)
   {
      yoManager.startSession(session);
      referenceFrameManager.startSession(session);
      yoRobotJSManager.startSession(session);
      environmentManager.startSession(session);

      session.setupWithMessager(messager);

      referenceFrameManager.refreshReferenceFramesNow();
      messager.submitMessage(topics.getSessionCurrentState(), SessionState.ACTIVE);
      if (sessionLoadedCallback != null)
         sessionLoadedCallback.run();

   }

   public void stopSession()
   {
      yoManager.stopSession();
      yoRobotJSManager.stopSession();
      referenceFrameManager.stopSession();
      environmentManager.stopSession();
   }

   public void stop()
   {
      backgroundExecutorManager.stop();
      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public Messager getMessager()
   {
      return messager;
   }

   public SessionVisualizerTopics getTopics()
   {
      return topics;
   }

   public YoManager getYoManager()
   {
      return yoManager;
   }

   public JavalinManager getJavalinManager()
   {
      return javalinManager;
   }

   public YoRobotJSManager getYoRobotJSManager()
   {
      return yoRobotJSManager;
   }

   public ReferenceFrame getWorldFrame()
   {
      return referenceFrameManager.getWorldFrame();
   }

   public ReferenceFrameManager getReferenceFrameManager()
   {
      return referenceFrameManager;
   }

   public EnvironmentManagerJS getEnvironmentManager()
   {
      return environmentManager;
   }

   public BackgroundExecutorManager getBackgroundExecutorManager()
   {
      return backgroundExecutorManager;
   }

   public boolean hasActiveSession()
   {
      // TODO Auto-generated method stub
      return false;
   }
}

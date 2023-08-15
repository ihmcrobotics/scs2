package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.SubScene;
import javafx.stage.Stage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMessagerAPI;
import us.ihmc.scs2.session.SessionState;
import us.ihmc.scs2.session.YoSharedBufferMessagerAPI;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionChangeListener;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerMessagerAPI;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.SCS2JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

public class SessionVisualizerToolkit extends ObservedAnimationTimer
{
   private final SCS2JavaFXMessager messager;
   private final SessionVisualizerTopics topics = new SessionVisualizerTopics();

   private final YoManager yoManager = new YoManager();
   private final ChartDataManager chartDataManager;
   private final ChartRenderManager chartRenderManager = new ChartRenderManager();
   private final YoGraphicFXManager yoGraphicFXManager;
   private final YoCompositeSearchManager yoCompositeSearchManager;
   private final SnapshotManager snapshotManager;
   private final VideoRecordingManager videoRecordingManager;
   private final KeyFrameManager keyFrameManager;
   private final CameraSensorsManager cameraSensorsManager;

   private final BackgroundExecutorManager backgroundExecutorManager = new BackgroundExecutorManager(4);
   private final ReferenceFrameManager referenceFrameManager = new ReferenceFrameManager(yoManager, backgroundExecutorManager);
   private final YoRobotFXManager yoRobotFXManager;
   private final EnvironmentManager environmentManager;
   private final SecondaryWindowManager secondaryWindowManager;
   private final SessionDataPreferenceManager sessionDataPreferenceManager;
   private final MultiViewport3DManager viewport3DManager;

   private final Stage mainWindow;
   private final SubScene mainScene3D;
   private final Group mainView3DRoot;

   private final ObjectProperty<Session> activeSessionProperty = new SimpleObjectProperty<>(this, "activeSession", null);
   private final ObservableList<RobotDefinition> sessionRobotDefinitions = FXCollections.observableArrayList();
   private final ObservableList<TerrainObjectDefinition> sessionTerrainObjectDefinitions = FXCollections.observableArrayList();
   private final ConcurrentLinkedQueue<SessionChangeListener> sessionChangeListeners = new ConcurrentLinkedQueue<>();

   public SessionVisualizerToolkit(Stage mainWindow, Group mainView3DRoot) throws Exception
   {
      this.mainWindow = mainWindow;
      this.mainView3DRoot = mainView3DRoot;

      MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      apiFactory.createRootCategory("SCS2");
      apiFactory.includeMessagerAPIs(SessionMessagerAPI.API, YoSharedBufferMessagerAPI.API, SessionVisualizerMessagerAPI.API);
      messager = new SCS2JavaFXMessager(apiFactory.getAPIAndCloseFactory());

      topics.setupTopics();
      messager.startMessager();

      snapshotManager = new SnapshotManager(mainWindow, messager, topics);
      chartDataManager = new ChartDataManager(messager, topics, yoManager, backgroundExecutorManager);
      yoGraphicFXManager = new YoGraphicFXManager(messager, topics, yoManager, backgroundExecutorManager, referenceFrameManager);
      yoCompositeSearchManager = new YoCompositeSearchManager(messager, topics, yoManager, backgroundExecutorManager);
      keyFrameManager = new KeyFrameManager(messager, topics);
      yoRobotFXManager = new YoRobotFXManager(messager, topics, yoManager, referenceFrameManager, backgroundExecutorManager);
      environmentManager = new EnvironmentManager(messager, topics, backgroundExecutorManager);

      viewport3DManager = new MultiViewport3DManager(mainView3DRoot, yoManager, yoCompositeSearchManager, referenceFrameManager);
      this.mainScene3D = viewport3DManager.getMainViewport().getSubScene();
      mainView3DRoot.getChildren().addAll(yoGraphicFXManager.getRootNode3D(), yoRobotFXManager.getRootNode(), environmentManager.getRootNode());
      environmentManager.addSkybox(viewport3DManager.getMainViewport().getCamera());

      messager.addFXTopicListener(topics.getCamera3DRequest(), viewport3DManager::submitRequest);

      videoRecordingManager = new VideoRecordingManager(mainScene3D, mainView3DRoot, topics, messager, backgroundExecutorManager);
      secondaryWindowManager = new SecondaryWindowManager(this);
      sessionDataPreferenceManager = new SessionDataPreferenceManager(messager, topics);
      cameraSensorsManager = new CameraSensorsManager(mainView3DRoot, messager, topics, yoRobotFXManager);

      activeSessionProperty.addListener((o, oldValue, newValue) ->
      {
         sessionRobotDefinitions.clear();
         sessionTerrainObjectDefinitions.clear();

         if (newValue == null)
            return;

         List<RobotDefinition> newRobotDefinitions = newValue.getRobotDefinitions();
         if (newRobotDefinitions != null && !newRobotDefinitions.isEmpty())
            sessionRobotDefinitions.setAll(newRobotDefinitions);

         List<TerrainObjectDefinition> newTerrainObjectDefinitions = newValue.getTerrainObjectDefinitions();
         if (newTerrainObjectDefinitions != null && !newTerrainObjectDefinitions.isEmpty())
            sessionTerrainObjectDefinitions.setAll(newTerrainObjectDefinitions);
      });
   }

   public void startSession(Session session, Runnable sessionLoadedCallback)
   {
      Session oldSession = activeSessionProperty.get();
      if (oldSession != null)
      {
         LogTools.warn("Session already in progress. Stop the current session before starting a new one.");
         return;
      }

      activeSessionProperty.set(session);

      session.setupWithMessager(messager);

      backgroundExecutorManager.executeInBackground(() ->
      {
         try
         {
            yoManager.startSession(session);
            yoRobotFXManager.startSession(session);
            environmentManager.startSession(session);
            referenceFrameManager.startSession(session);
            chartDataManager.startSession(session);
            chartRenderManager.startSession(session);
            yoCompositeSearchManager.startSession(session);
            keyFrameManager.startSession(session);
            secondaryWindowManager.startSession(session);

            while (!yoRobotFXManager.isSessionLoaded())
            {
               try
               {
                  Thread.sleep(100);
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
                  return;
               }
            }

            yoGraphicFXManager.startSession(session); // In case some graphics rely on the robot frames
            cameraSensorsManager.startSession(session);
            messager.submitMessage(topics.getSessionCurrentState(), SessionState.ACTIVE);
         }
         finally
         {
            if (sessionLoadedCallback != null)
               sessionLoadedCallback.run();
            sessionChangeListeners.forEach(listener -> listener.sessionChanged(oldSession, session));
         }
      });

      mainWindow.setTitle(session.getSessionName());
   }

   public void stopSession(boolean shutdownSession)
   {
      Session oldSession = activeSessionProperty.get();
      if (oldSession == null)
         return;

      activeSessionProperty.set(null);

      backgroundExecutorManager.stopSession();
      yoRobotFXManager.stopSession();
      chartDataManager.stopSession();
      chartRenderManager.stopSession();
      yoGraphicFXManager.stopSession();
      referenceFrameManager.stopSession();
      yoCompositeSearchManager.stopSession();
      environmentManager.stopSession();
      keyFrameManager.stopSession();
      secondaryWindowManager.stopSession();
      cameraSensorsManager.stopSession();
      yoManager.stopSession();

      mainWindow.setTitle(SessionVisualizer.NO_ACTIVE_SESSION_TITLE);

      if (shutdownSession)
         messager.submitMessage(topics.getSessionCurrentState(), SessionState.INACTIVE);
      sessionChangeListeners.forEach(listener -> listener.sessionChanged(oldSession, null));
   }

   public boolean hasActiveSession()
   {
      return activeSessionProperty.get() != null;
   }

   public Session getSession()
   {
      return activeSessionProperty.get();
   }

   public void addSessionChangedListener(SessionChangeListener listener)
   {
      sessionChangeListeners.add(listener);
   }

   public boolean removeSessionChangedListener(SessionChangeListener listener)
   {
      return sessionChangeListeners.remove(listener);
   }

   @Override
   public void handleImpl(long now)
   {
   }

   @Override
   public void stop()
   {
      super.stop();
      yoManager.stop();
      yoRobotFXManager.stop();
      yoGraphicFXManager.stop();
      backgroundExecutorManager.shutdown();
      environmentManager.dispose();
      messager.closeMessager();
      viewport3DManager.dispose();
      mainView3DRoot.getChildren().clear();
   }

   public SCS2JavaFXMessager getMessager()
   {
      return messager;
   }

   public SessionVisualizerTopics getTopics()
   {
      return topics;
   }

   public Stage getMainWindow()
   {
      return mainWindow;
   }

   public SubScene getMainScene3D()
   {
      return mainScene3D;
   }

   public Group getMainView3DRoot()
   {
      return mainView3DRoot;
   }

   public MultiViewport3DManager getViewport3DManager()
   {
      return viewport3DManager;
   }

   public YoManager getYoManager()
   {
      return yoManager;
   }

   public YoGraphicFXManager getYoGraphicFXManager()
   {
      return yoGraphicFXManager;
   }

   public YoGroupFX getYoGraphicFXRootGroup()
   {
      return yoGraphicFXManager.getRootGroup();
   }

   public YoGroupFX getYoGraphicFXSessionRootGroup()
   {
      return yoGraphicFXManager.getSessionRootGroup();
   }

   public YoRobotFXManager getYoRobotFXManager()
   {
      return yoRobotFXManager;
   }

   public ChartDataManager getChartDataManager()
   {
      return chartDataManager;
   }

   public ChartRenderManager getChartRenderManager()
   {
      return chartRenderManager;
   }

   public YoCompositeSearchManager getYoCompositeSearchManager()
   {
      return yoCompositeSearchManager;
   }

   public ReferenceFrame getWorldFrame()
   {
      return referenceFrameManager.getWorldFrame();
   }

   public ReferenceFrameManager getReferenceFrameManager()
   {
      return referenceFrameManager;
   }

   public EnvironmentManager getEnvironmentManager()
   {
      return environmentManager;
   }

   public KeyFrameManager getKeyFrameManager()
   {
      return keyFrameManager;
   }

   public SnapshotManager getSnapshotManager()
   {
      return snapshotManager;
   }

   public BackgroundExecutorManager getBackgroundExecutorManager()
   {
      return backgroundExecutorManager;
   }

   public VideoRecordingManager getVideoRecordingManager()
   {
      return videoRecordingManager;
   }

   public SecondaryWindowManager getWindowManager()
   {
      return secondaryWindowManager;
   }

   public SessionDataPreferenceManager getSessionDataPreferenceManager()
   {
      return sessionDataPreferenceManager;
   }

   public ObservableList<RobotDefinition> getSessionRobotDefinitions()
   {
      return sessionRobotDefinitions;
   }

   public ObservableList<TerrainObjectDefinition> getSessionTerrainObjectDefinitions()
   {
      return sessionTerrainObjectDefinitions;
   }
}

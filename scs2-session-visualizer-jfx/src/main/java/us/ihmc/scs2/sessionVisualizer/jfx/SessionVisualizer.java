package us.ihmc.scs2.sessionVisualizer.jfx;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.lang3.mutable.MutableObject;

import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.geometry.Rectangle2D;
import javafx.scene.Group;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.javaFXToolkit.cameraControllers.CameraZoomCalculator;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.MultiSessionManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.plotter.Plotter2D;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.BufferedJavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CameraTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXApplicationCreator;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class SessionVisualizer
{
   public static final String NO_ACTIVE_SESSION_TITLE = "No Active Session";

   static
   {
      DefinitionIOTools.loadResources();
      YoGraphicFXControllerTools.loadResources();
   }

   private final SessionVisualizerToolkit toolkit;
   private final MultiSessionManager multiSessionManager;

   private final Group view3DRoot;
   private final Plotter2D plotter2D = new Plotter2D();
   private final MainWindowController mainWindowController;
   private final FocusBasedCameraMouseEventHandler cameraController;
   private final BufferedJavaFXMessager messager;
   private final SessionVisualizerTopics topics;
   private final SessionVisualizerControlsImpl sessionVisualizerControls = new SessionVisualizerControlsImpl();
   private final List<Runnable> stopListeners = new ArrayList<>();

   private final Stage primaryStage;

   private boolean hasTerminated = false;

   public SessionVisualizer(Stage primaryStage) throws Exception
   {
      this.primaryStage = primaryStage;
      // Configuring listener first so this is the first one getting called. Allows to cancel the close request.
      primaryStage.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, this::stop);
      SessionVisualizerIOTools.addSCSIconToWindow(primaryStage);
      primaryStage.setTitle(NO_ACTIVE_SESSION_TITLE);

      View3DFactory view3DFactory = View3DFactory.createSubscene();
      view3DRoot = view3DFactory.getRoot();
      view3DFactory.addDefaultLighting();

      toolkit = new SessionVisualizerToolkit(primaryStage, view3DFactory.getSubScene(), view3DRoot);
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();

      FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.MAIN_WINDOW_URL);
      Parent mainPane = loader.load();
      mainWindowController = loader.getController();
      mainWindowController.initialize(new SessionVisualizerWindowToolkit(primaryStage, toolkit));

      view3DFactory.addNodeToView(toolkit.getYoRobotFXManager().getRootNode());
      view3DFactory.addNodeToView(toolkit.getEnvironmentManager().getRootNode());
      cameraController = view3DFactory.addCameraController(0.05, 2.0e5, true);
      CameraTools.setupNodeTrackingContextMenu(cameraController, view3DFactory.getSubScene());

      messager.registerJavaFXSyncedTopicListener(topics.getCameraTrackObject(), request ->
      {
         if (request.getNode() != null)
            cameraController.getNodeTracker().setNodeToTrack(request.getNode());
      });

      toolkit.getEnvironmentManager().addWorldCoordinateSystem(0.3);
      toolkit.getEnvironmentManager().addSkybox(view3DFactory.getSubScene());
      messager.registerJavaFXSyncedTopicListener(topics.getSessionVisualizerCloseRequest(), m -> stop());

      mainWindowController.setupPlotter2D(plotter2D);

      view3DFactory.addNodeToView(toolkit.getYoGraphicFXManager().getRootNode3D());
      mainWindowController.setupViewport3D(view3DFactory.getSubSceneWrappedInsidePane());

      Scene mainScene = new Scene(mainPane);
      toolkit.getSnapshotManager().registerRecordable(mainScene);
      primaryStage.setScene(mainScene);
      multiSessionManager = new MultiSessionManager(toolkit, mainWindowController);

      mainWindowController.start();
      toolkit.start();
      initializeStageWithPrimaryScreen();
      primaryStage.show();
      // Seems that on Ubuntu the changes done to the window position/size are not processed properly until the window is showing.
      initializeStageWithPrimaryScreen();
   }

   public void initializeStageWithPrimaryScreen()
   {
      initializeStageWithScreen(0.75, Screen.getPrimary(), primaryStage);
   }

   public static void initializeStageWithScreen(double sizeRatio, Screen screen, Stage stage)
   {
      Rectangle2D bounds = screen.getVisualBounds();

      double width = sizeRatio * bounds.getWidth();
      double height = sizeRatio * bounds.getHeight();
      stage.setWidth(width);
      stage.setHeight(height);
      double centerX = bounds.getMinX() + (bounds.getWidth() - width) * 0.5;
      double centerY = bounds.getMinY() + (bounds.getHeight() - height) * 1.0 / 3.0;
      stage.setX(centerX);
      stage.setY(centerY);
   }

   public void startSession(Session session)
   {
      Runnable sessionLoadedCallback = () -> sessionVisualizerControls.visualizerReadyLatch.countDown();
      multiSessionManager.startSession(session, sessionLoadedCallback);
   }

   public void stop()
   {
      stop(null);
   }

   public void stop(WindowEvent event)
   {
      boolean saveConfiguration = false;

      if (toolkit.hasActiveSession())
      {
         Alert alert = new Alert(AlertType.CONFIRMATION, "Do you want to save the default configuration?", ButtonType.YES, ButtonType.NO, ButtonType.CANCEL);
         SessionVisualizerIOTools.addSCSIconToDialog(alert);

         Optional<ButtonType> result = alert.showAndWait();
         if (!result.isPresent() || result.get() == ButtonType.CANCEL)
         {
            if (event != null)
               event.consume();
            return;
         }

         saveConfiguration = result.get() == ButtonType.YES;
      }

      stopNow(saveConfiguration);
   }

   private void stopNow(boolean saveConfiguration)
   {
      if (hasTerminated)
         return;

      hasTerminated = true;

      LogTools.info("Simulation GUI is going down.");
      try
      {
         cameraController.dispose();
         multiSessionManager.stopSession(saveConfiguration);
         multiSessionManager.shutdown();
         mainWindowController.stop();
         toolkit.stop();
         if (primaryStage.isShowing())
            primaryStage.close();
         primaryStage.setScene(null);
         view3DRoot.getChildren().clear();

         stopListeners.forEach(Runnable::run);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      finally
      {
         sessionVisualizerControls.visualizerShutdownLatch.countDown();
      }
   }

   public AnchorPane getSceneAnchorPane()
   {
      return mainWindowController.getSceneAnchorPane();
   }

   public SessionVisualizerToolkit getToolkit()
   {
      return toolkit;
   }

   public static void main(String[] args)
   {
      SessionVisualizerControls controls = startSessionVisualizer(null, true);
      // When running as remote visualizer, some non-daemon threads are not cleaned up properly.
      controls.addVisualizerShutdownListener(() -> System.exit(0));
   }

   public static SessionVisualizerControls startSessionVisualizer()
   {
      return startSessionVisualizer(null);
   }

   public static SessionVisualizerControls startSessionVisualizer(Session session)
   {
      return startSessionVisualizer(session, null);
   }

   public static SessionVisualizerControls startSessionVisualizer(Session session, Boolean javaFXThreadImplicitExit)
   {
      if (javaFXThreadImplicitExit != null && Platform.isImplicitExit() != javaFXThreadImplicitExit)
         Platform.setImplicitExit(javaFXThreadImplicitExit);

      MutableObject<SessionVisualizerControls> sessionVisualizerControls = new MutableObject<>();

      JavaFXApplicationCreator.spawnJavaFXMainApplication();

      JavaFXMissingTools.runAndWait(SessionVisualizer.class, () ->
      {
         try
         {
            SessionVisualizer sessionVisualizer = new SessionVisualizer(new Stage());
            sessionVisualizerControls.setValue(sessionVisualizer.sessionVisualizerControls);
            if (session != null)
               sessionVisualizer.startSession(session);
            JavaFXApplicationCreator.attachStopListener(sessionVisualizer::stop);
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      });

      return sessionVisualizerControls.getValue();
   }

   private class SessionVisualizerControlsImpl implements SessionVisualizerControls
   {
      private final CountDownLatch visualizerReadyLatch = new CountDownLatch(1);
      private final CountDownLatch visualizerShutdownLatch = new CountDownLatch(1);

      public SessionVisualizerControlsImpl()
      {
      }

      @Override
      public void waitUntilFullyUp()
      {
         try
         {
            visualizerReadyLatch.await();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }

      @Override
      public void waitUntilDown()
      {
         try
         {
            visualizerShutdownLatch.await();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }

      @Override
      public void setCameraOrientation(double latitude, double longitude, double roll)
      {
         checkVisualizerRunning();
         waitUntilFullyUp();
         JavaFXMissingTools.runLater(getClass(), () -> cameraController.getRotationCalculator().setRotation(latitude, longitude, roll));
      }

      @Override
      public void setCameraPosition(double x, double y, double z)
      {
         checkVisualizerRunning();
         waitUntilFullyUp();
         JavaFXMissingTools.runLater(getClass(), () -> cameraController.changeCameraPosition(x, y, z));
      }

      @Override
      public void setCameraFocusPosition(double x, double y, double z)
      {
         checkVisualizerRunning();
         waitUntilFullyUp();
         JavaFXMissingTools.runLater(getClass(), () -> cameraController.changeFocusPosition(x, y, z, false));
      }

      @Override
      public void setCameraZoom(double distanceFromFocus)
      {
         checkVisualizerRunning();
         waitUntilFullyUp();
         JavaFXMissingTools.runLater(getClass(), () ->
         {
            CameraZoomCalculator zoomCalculator = cameraController.getZoomCalculator();
            if (zoomCalculator.isInvertZoomDirection())
               zoomCalculator.setZoom(-distanceFromFocus);
            else
               zoomCalculator.setZoom(distanceFromFocus);
         });
      }

      @Override
      public void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
      {
         checkVisualizerRunning();
         waitUntilFullyUp();
         messager.submitMessage(topics.getCameraTrackObject(), new CameraObjectTrackingRequest(robotName, rigidBodyName));
      }

      @Override
      public void addStaticVisual(VisualDefinition visualDefinition)
      {
         checkVisualizerRunning();
         waitUntilFullyUp();
         toolkit.getEnvironmentManager().addStaticVisual(visualDefinition);
      }

      @Override
      public void addYoGraphic(String namespace, YoGraphicDefinition yoGraphicDefinition)
      {
         String[] subNames = namespace.split(YoGraphicTools.SEPARATOR);
         if (subNames == null || subNames.length == 0)
            addYoGraphic(yoGraphicDefinition);

         for (int i = subNames.length - 1; i >= 0; i--)
         {
            yoGraphicDefinition = new YoGraphicGroupDefinition(subNames[i], yoGraphicDefinition);
         }

         addYoGraphic(yoGraphicDefinition);
      }

      @Override
      public void addYoGraphic(YoGraphicDefinition yoGraphicDefinition)
      {
         checkVisualizerRunning();
         messager.submitMessage(topics.getAddYoGraphicRequest(), yoGraphicDefinition);
      }

      @Override
      public void exportVideo(SceneVideoRecordingRequest request)
      {
         checkVisualizerRunning();
         messager.submitMessage(topics.getSceneVideoRecordingRequest(), request);
      }

      @Override
      public void shutdown()
      {
         JavaFXMissingTools.runAndWait(getClass(), () -> stop());
      }

      @Override
      public void shutdownNow()
      {
         JavaFXMissingTools.runAndWait(getClass(), () -> stopNow(false));
      }

      @Override
      public void addVisualizerShutdownListener(Runnable listener)
      {
         checkVisualizerRunning();
         stopListeners.add(listener);
      }

      private void checkVisualizerRunning()
      {
         if (hasTerminated)
            throw new IllegalOperationException("Unable to perform operation, visualizer has terminated.");
      }
   }
}

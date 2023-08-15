package us.ihmc.scs2.sessionVisualizer.jfx;

import java.net.URL;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.geometry.Rectangle2D;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import javafx.util.Pair;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.camera.YoLevelOrbitalCoordinateDefinition;
import us.ihmc.scs2.definition.camera.YoOrbitalCoordinateDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoEntry.YoEntryListDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardType;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionDataFilterParameters;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.Camera3DRequest.CameraControlRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.Camera3DRequest.FocalPointRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.MultiSessionManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoBooleanProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoEnumAsStringProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoIntegerProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoLongProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXApplicationCreator;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.SCS2JavaFXMessager;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class SessionVisualizer
{
   private static final boolean SHOW_WORLD_FRAME = SessionPropertiesHelper.loadBooleanPropertyOrEnvironment("scs2.session.gui.worldframe.show",
                                                                                                            "SCS2_GUI_WORLDFRAME_SHOW",
                                                                                                            true);
   public static final String NO_ACTIVE_SESSION_TITLE = "No Active Session";

   static
   {
      DefinitionIOTools.loadResources();
      YoGraphicFXControllerTools.loadResources();
   }

   private final boolean shutdownSessionOnClose;

   private final SessionVisualizerToolkit toolkit;
   private final MultiSessionManager multiSessionManager;

   private final MainWindowController mainWindowController;
   private final SCS2JavaFXMessager messager;
   private final SessionVisualizerTopics topics;
   private final SessionVisualizerControlsImpl sessionVisualizerControls = new SessionVisualizerControlsImpl();
   private final List<Runnable> stopListeners = new ArrayList<>();

   private final Stage primaryStage;

   private boolean hasTerminated = false;

   public SessionVisualizer(Stage primaryStage, boolean shutdownSessionOnClose, Session initialSession) throws Exception
   {
      this.primaryStage = primaryStage;
      this.shutdownSessionOnClose = shutdownSessionOnClose;
      // Configuring listener first so this is the first one getting called. Allows to cancel the close request.
      primaryStage.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, this::stop);
      SessionVisualizerIOTools.addSCSIconToWindow(primaryStage);
      primaryStage.setTitle(NO_ACTIVE_SESSION_TITLE);

      Scene3DBuilder scene3DBuilder = new Scene3DBuilder();
      scene3DBuilder.addDefaultLighting();

      toolkit = new SessionVisualizerToolkit(primaryStage, scene3DBuilder.getRoot());
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();

      FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.MAIN_WINDOW_URL);
      Parent mainPane = loader.load();
      mainWindowController = loader.getController();
      mainWindowController.initialize(new SessionVisualizerWindowToolkit(primaryStage, toolkit));

      if (SHOW_WORLD_FRAME)
         toolkit.getEnvironmentManager().addWorldCoordinateSystem(0.3);
      messager.addFXTopicListener(topics.getSessionVisualizerCloseRequest(), m -> stop());

      // This is workaround to get the lights working when doing snapshots.
      Group clonedLightGroup = new Group();
      Scene3DBuilder.setupLigthCloneList(clonedLightGroup.getChildren(), scene3DBuilder.getAllLights());
      StackPane mainPaneWithLights = new StackPane(mainPane);
      mainPaneWithLights.getChildren().add(clonedLightGroup);
      mainPaneWithLights.getStylesheets().setAll(mainPane.getStylesheets());
      Scene mainScene = new Scene(mainPaneWithLights);
      toolkit.getSnapshotManager().registerRecordable(mainScene);
      primaryStage.setScene(mainScene);
      multiSessionManager = new MultiSessionManager(toolkit, mainWindowController);

      mainWindowController.start();
      toolkit.start();
      initializeStageWithPrimaryScreen();
      primaryStage.show();
      // TODO Seems that on Ubuntu the changes done to the window position/size are not processed properly until the window is showing.
      // This may be related to the bug reported when using GTK3: https://github.com/javafxports/openjdk-jfx/pull/446, might be fixed in later version.
      initializeStageWithPrimaryScreen();

      if (initialSession != null)
      {
         Runnable sessionLoadedCallback = () -> sessionVisualizerControls.visualizerReadyLatch.countDown();
         multiSessionManager.startSession(initialSession, sessionLoadedCallback);
      }
      else
      {
         sessionVisualizerControls.visualizerReadyLatch.countDown();
      }
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
         alert.initOwner(primaryStage);
         JavaFXMissingTools.centerDialogInOwner(alert);

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

      try
      {
         multiSessionManager.stopSession(saveConfiguration, shutdownSessionOnClose);
         multiSessionManager.shutdown();
         mainWindowController.stop();
         toolkit.stop();
         if (primaryStage.isShowing())
            primaryStage.close();
         primaryStage.setScene(null);

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
      return startSessionVisualizer(session, javaFXThreadImplicitExit, true);
   }

   public static SessionVisualizerControls startSessionVisualizer(Session session, Boolean javaFXThreadImplicitExit, boolean shutdownSessionOnClose)
   {
      SessionVisualizer sessionVisualizer = startSessionVisualizerExpert(session, javaFXThreadImplicitExit, shutdownSessionOnClose);
      if (sessionVisualizer != null)
         return sessionVisualizer.getSessionVisualizerControls();
      else
         return null;
   }

   public static SessionVisualizer startSessionVisualizerExpert(Session session, Boolean javaFXThreadImplicitExit)
   {
      return startSessionVisualizerExpert(session, javaFXThreadImplicitExit, true);
   }

   public static SessionVisualizer startSessionVisualizerExpert(Session session, Boolean javaFXThreadImplicitExit, boolean shutdownSessionOnClose)
   {
      JavaFXApplicationCreator.spawnJavaFXMainApplication();

      return JavaFXMissingTools.runAndWait(SessionVisualizer.class, () ->
      {
         try
         {
            SessionVisualizer sessionVisualizer = new SessionVisualizer(new Stage(), shutdownSessionOnClose, session);

            JavaFXApplicationCreator.attachStopListener(sessionVisualizer::stop);

            if (javaFXThreadImplicitExit != null && Platform.isImplicitExit() != javaFXThreadImplicitExit)
               Platform.setImplicitExit(javaFXThreadImplicitExit);

            return sessionVisualizer;
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      });
   }

   private class SessionVisualizerControlsImpl implements SessionVisualizerControls
   {
      private final CountDownLatch visualizerReadyLatch = new CountDownLatch(1);
      private final CountDownLatch visualizerShutdownLatch = new CountDownLatch(1);

      public SessionVisualizerControlsImpl()
      {
      }

      @Override
      public void waitUntilVisualizerFullyUp()
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
      public void waitUntilVisualizerDown()
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
      public void setCameraOrientation(double latitude, double longitude)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         toolkit.getViewport3DManager().getMainViewport().setCameraOrientation(latitude, longitude, 0);
      }

      @Override
      public void setCameraPosition(double x, double y, double z)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         toolkit.getViewport3DManager().getMainViewport().setCameraPosition(x, y, z);
      }

      @Override
      public void setCameraFocalPosition(double x, double y, double z)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         toolkit.getViewport3DManager().getMainViewport().setCameraFocalPosition(x, y, z);
      }

      @Override
      public void setCameraZoom(double distanceFromFocus)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         toolkit.getViewport3DManager().getMainViewport().setCameraZoom(distanceFromFocus);
      }

      @Override
      public void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         submitMessage(getTopics().getCamera3DRequest(), new Camera3DRequest(FocalPointRequest.trackRobot(robotName, rigidBodyName)));
      }

      @Override
      public void requestCameraFocalPositionTracking(YoTuple3DDefinition coordinatesToTrack)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         submitMessage(getTopics().getCamera3DRequest(), new Camera3DRequest(FocalPointRequest.trackCoordinates(coordinatesToTrack)));
      }

      @Override
      public void requestCameraPositionTracking(YoTuple3DDefinition cameraCoordinates)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         submitMessage(getTopics().getCamera3DRequest(), new Camera3DRequest(CameraControlRequest.trackPosition(cameraCoordinates)));
      }

      @Override
      public void requestCameraOrbitTracking(YoOrbitalCoordinateDefinition cameraCoordinates)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         submitMessage(getTopics().getCamera3DRequest(), new Camera3DRequest(CameraControlRequest.trackOrbit(cameraCoordinates)));
      }

      @Override
      public void requestCameraLevelOrbitTracking(YoLevelOrbitalCoordinateDefinition cameraCoordinates)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         submitMessage(getTopics().getCamera3DRequest(), new Camera3DRequest(CameraControlRequest.trackLevelOrbit(cameraCoordinates)));
      }

      @Override
      public void showOverheadPlotter2D(boolean show)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         submitMessage(getTopics().getShowOverheadPlotter(), show);
      }

      @Override
      public void requestPlotter2DCoordinateTracking(String xVariableName, String yVariableName, String frameName)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         if (xVariableName == null)
            xVariableName = Double.toString(0.0);
         if (yVariableName == null)
            yVariableName = Double.toString(0.0);
         if (frameName == null)
            frameName = ReferenceFrameManager.WORLD_FRAME;
         submitMessage(getTopics().getPlotter2DTrackCoordinateRequest(), new YoTuple2DDefinition(xVariableName, yVariableName, frameName));
      }

      @Override
      public void addStaticVisual(VisualDefinition visualDefinition)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         toolkit.getEnvironmentManager().addStaticVisual(visualDefinition);
      }

      @Override
      public void removeStaticVisual(VisualDefinition visualDefinition)
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         toolkit.getEnvironmentManager().removeStaticVisual(visualDefinition);
      }

      @Override
      public void removeYoGraphic(String name)
      {
         submitMessage(getTopics().getRemoveYoGraphicRequest(), name);
      }

      @Override
      public void setYoGraphicVisible(String name, boolean visible)
      {
         submitMessage(getTopics().getSetYoGraphicVisibleRequest(), new Pair<>(name, visible));
      }

      @Override
      public void addYoGraphic(YoGraphicDefinition yoGraphicDefinition)
      {
         submitMessage(getTopics().getAddYoGraphicRequest(), yoGraphicDefinition);
      }

      @Override
      public void addYoEntry(String groupName, Collection<String> variableNames)
      {
         submitMessage(getTopics().getYoEntryListAdd(), YoEntryListDefinition.newYoVariableEntryList(groupName, variableNames));
      }

      @Override
      public void clearAllSliderboards()
      {
         submitMessage(getTopics().getYoMultiSliderboardClearAll(), true);
      }

      @Override
      public void setSliderboards(YoSliderboardListDefinition sliderboardListDefinition)
      {
         submitMessage(getTopics().getYoMultiSliderboardSet(), sliderboardListDefinition);
      }

      @Override
      public void setSliderboard(YoSliderboardDefinition sliderboardConfiguration)
      {
         submitMessage(getTopics().getYoSliderboardSet(), sliderboardConfiguration);
      }

      @Override
      public void removeSliderboard(String sliderboardName, YoSliderboardType sliderboardType)
      {
         submitMessage(getTopics().getYoSliderboardRemove(), new Pair<>(sliderboardName, sliderboardType));
      }

      @Override
      public void setSliderboardButton(String sliderboardName, YoSliderboardType sliderboardType, YoButtonDefinition buttonDefinition)
      {
         submitMessage(getTopics().getYoSliderboardSetButton(),
                       new ImmutableTriple<>(sliderboardName, sliderboardType, buttonDefinition));
      }

      @Override
      public void clearSliderboardButton(String sliderboardName, YoSliderboardType sliderboardType, int buttonIndex)
      {
         submitMessage(getTopics().getYoSliderboardClearButton(),
                       new ImmutableTriple<>(sliderboardName, sliderboardType, buttonIndex));
      }

      @Override
      public void setSliderboardKnob(String sliderboardName, YoSliderboardType sliderboardType, YoKnobDefinition knobDefinition)
      {
         submitMessage(getTopics().getYoSliderboardSetKnob(),
                       new ImmutableTriple<>(sliderboardName, sliderboardType, knobDefinition));
      }

      @Override
      public void clearSliderboardKnob(String sliderboardName, YoSliderboardType sliderboardType, int knobIndex)
      {
         submitMessage(getTopics().getYoSliderboardClearKnob(),
                       new ImmutableTriple<>(sliderboardName, sliderboardType, knobIndex));
      }

      @Override
      public void setSliderboardSlider(String sliderboardName, YoSliderboardType sliderboardType, YoSliderDefinition sliderDefinition)
      {
         submitMessage(getTopics().getYoSliderboardSetSlider(),
                       new ImmutableTriple<>(sliderboardName, sliderboardType, sliderDefinition));
      }

      @Override
      public void clearSliderboardSlider(String sliderboardName, YoSliderboardType sliderboardType, int sliderIndex)
      {
         submitMessage(getTopics().getYoSliderboardClearSlider(),
                       new ImmutableTriple<>(sliderboardName, sliderboardType, sliderIndex));
      }

      @Override
      public void addSessionDataFilterParameters(SessionDataFilterParameters filterParameters)
      {
         submitMessage(getTopics().getSessionDataFilterParametersAddRequest(), filterParameters);
      }

      @Override
      public void exportVideo(SceneVideoRecordingRequest request)
      {
         checkVisualizerRunning();
         checkSessionThreadRunning();

         CountDownLatch latch = new CountDownLatch(1);
         Runnable callback = request.getRecordingEndedCallback();
         request.setRecordingEndedCallback(() ->
         {
            latch.countDown();
            if (callback != null)
               callback.run();
         });
         messager.submitMessage(topics.getSceneVideoRecordingRequest(), request);

         try
         {
            latch.await();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }

      /** {@inheritDoc} */
      @Override
      public void disableGUIControls()
      {
         submitMessage(getTopics().getDisableUserControls(), true);
      }

      /** {@inheritDoc} */
      @Override
      public void enableGUIControls()
      {
         submitMessage(getTopics().getDisableUserControls(), false);
      }

      /**
       * Gets the messager's topics.
       * <p>
       * The visualizer relies on the {@link Messager} framework to communicate requests.
       * </p>
       *
       * @return the topics this visualizer uses.
       */
      SessionVisualizerTopics getTopics()
      {
         return topics;
      }

      /**
       * Submits a message.
       *
       * @param <T>            the type of the message content imposed by the selected topic.
       * @param topic          the topic to with the message is to be submitted.
       * @param messageContent the content of the message.
       */
      <T> void submitMessage(Topic<T> topic, T messageContent)
      {
         checkVisualizerRunning();
         messager.submitMessage(topic, messageContent);
      }

      @Override
      public Window getPrimaryGUIWindow()
      {
         checkVisualizerRunning();
         waitUntilVisualizerFullyUp();
         return primaryStage;
      }

      @Override
      public void addCustomGUIControl(Node control)
      {
         checkVisualizerRunning();
         mainWindowController.getUserSidePaneController().addControl(control);
      }

      @Override
      public boolean removeCustomGUIControl(Node control)
      {
         checkVisualizerRunning();
         return mainWindowController.getUserSidePaneController().removeControl(control);
      }

      @Override
      public void loadCustomGUIPane(String name, URL fxmlResource)
      {
         checkVisualizerRunning();
         mainWindowController.getUserSidePaneController().loadCustomPane(name, fxmlResource);
      }

      @Override
      public void addCustomGUIPane(String name, Pane pane)
      {
         checkVisualizerRunning();
         mainWindowController.getUserSidePaneController().addCustomPane(name, pane);
      }

      @Override
      public boolean removeCustomGUIPane(String name)
      {
         checkVisualizerRunning();
         return mainWindowController.getUserSidePaneController().removeCustomPane(name);
      }

      @Override
      public YoDoubleProperty newYoDoubleProperty(String variableName)
      {
         checkVisualizerRunning();
         return toolkit.getYoManager().newYoDoubleProperty(variableName);
      }

      @Override
      public YoIntegerProperty newYoIntegerProperty(String variableName)
      {
         checkVisualizerRunning();
         return toolkit.getYoManager().newYoIntegerProperty(variableName);
      }

      @Override
      public YoBooleanProperty newYoBooleanProperty(String variableName)
      {
         checkVisualizerRunning();
         return toolkit.getYoManager().newYoBooleanProperty(variableName);
      }

      @Override
      public YoLongProperty newYoLongProperty(String variableName)
      {
         checkVisualizerRunning();
         return toolkit.getYoManager().newYoLongProperty(variableName);
      }

      @Override
      public <E extends Enum<E>> YoEnumAsStringProperty<E> newYoEnumProperty(String variableName)
      {
         checkVisualizerRunning();
         return toolkit.getYoManager().newYoEnumProperty(variableName);
      }

      @Override
      public void addSessionChangedListener(SessionChangeListener listener)
      {
         toolkit.addSessionChangedListener(listener);
      }

      @Override
      public boolean removeSessionChangedListener(SessionChangeListener listener)
      {
         return toolkit.removeSessionChangedListener(listener);
      }

      @Override
      public void requestVisualizerShutdown()
      {
         JavaFXMissingTools.runAndWait(getClass(), () -> stop());
      }

      @Override
      public void shutdownSession()
      {
         JavaFXMissingTools.runAndWait(getClass(), () -> stopNow(false));
      }

      @Override
      public void addVisualizerShutdownListener(Runnable listener)
      {
         checkVisualizerRunning();
         stopListeners.add(listener);
      }

      @Override
      public boolean isVisualizerShutdown()
      {
         return hasTerminated;
      }

      private void checkVisualizerRunning()
      {
         if (hasTerminated)
            throw new IllegalOperationException("Unable to perform operation, visualizer has terminated.");
      }

      private void checkSessionThreadRunning()
      {
         if (toolkit.getSession() == null)
            throw new IllegalOperationException("No active session.");
         if (!toolkit.getSession().hasSessionStarted())
            throw new IllegalOperationException("Session thread is not running.");
      }
   }

   public SessionVisualizerControls getSessionVisualizerControls()
   {
      return sessionVisualizerControls;
   }
}

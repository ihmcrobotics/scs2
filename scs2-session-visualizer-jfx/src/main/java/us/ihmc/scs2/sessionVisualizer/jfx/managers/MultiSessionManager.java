package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.zip.ZipEntry;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.SynchronizeHint;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionIOTools;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.MainWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.SCSGuiConfiguration;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.entry.YoEntryTabPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionInfoController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.LogSessionManagerController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.remote.RemoteSessionManagerController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.SCS2JavaFXMessager;

public class MultiSessionManager
{
   private static final boolean LOAD_SESSION_SYNCHRONOUS = SessionPropertiesHelper.loadBooleanPropertyOrEnvironment("scs2.session.gui.loadconfig.synchronous",
                                                                                                                    "SCS2_GUI_LOAD_SESSION_SYNCHRONOUS",
                                                                                                                    true);
   private static final boolean LOAD_SESSION_TIME = SessionPropertiesHelper.loadBooleanProperty("scs2.session.gui.loadconfig.time", false);
   private static final boolean LOAD_MAIN_WINDOW_CONFIGURATION = SessionPropertiesHelper.loadBooleanProperty("scs2.session.gui.mainwindow.loadconfig", true);

   private final SessionVisualizerToolkit toolkit;
   private final MainWindowController mainWindowController;

   private final Map<Class<? extends SessionControlsController>, SessionControlsController> inactiveControllerMap = new HashMap<>();
   private final ObjectProperty<SessionControlsController> activeController = new SimpleObjectProperty<>(this, "activeSessionControls", null);
   // TODO This activeSession is not setup properly, when starting a sim it remains null.
   private final ObjectProperty<Session> activeSession = new SimpleObjectProperty<>(this, "activeSession", null);

   private boolean isFirstSession = true;

   public MultiSessionManager(SessionVisualizerToolkit toolkit, MainWindowController mainWindowController)
   {
      this.toolkit = toolkit;
      this.mainWindowController = mainWindowController;

      activeSession.addListener((o, oldValue, newValue) ->
      {
         JavaFXMissingTools.runAndWait(getClass(), () ->
         {
            if (toolkit.hasActiveSession())
            {
               Alert alert = new Alert(AlertType.CONFIRMATION, "Do you want to save the default configuration?", ButtonType.YES, ButtonType.NO);
               alert.initOwner(toolkit.getMainWindow());
               JavaFXMissingTools.centerDialogInOwner(alert);

               SessionVisualizerIOTools.addSCSIconToDialog(alert);
               Optional<ButtonType> result = alert.showAndWait();
               stopSession(result.isPresent() && result.get() == ButtonType.YES, true);
               if (oldValue != null)
                  oldValue.shutdownSession();
            }
         });

         if (newValue != null)
         {
            startSession(newValue, () ->
            {
               if (activeController.get() != null)
                  activeController.get().notifySessionLoaded();
            });
         }
      });

      SessionVisualizerTopics topics = toolkit.getTopics();
      JavaFXMessager messager = toolkit.getMessager();
      messager.addTopicListener(topics.getStartNewSessionRequest(), m -> activeSession.set(m));
      messager.addFXTopicListener(topics.getRemoteSessionControlsRequest(), m -> openRemoteSessionControls());
      messager.addFXTopicListener(topics.getLogSessionControlsRequest(), m -> openLogSessionControls());
      messager.addFXTopicListener(topics.getSessionVisualizerConfigurationLoadRequest(), m -> loadSessionConfiguration(m));
      messager.addFXTopicListener(topics.getSessionVisualizerConfigurationSaveRequest(), m -> saveSessionConfiguration(m));
      messager.addFXTopicListener(topics.getSessionVisualizerDefaultConfigurationLoadRequest(), m -> loadSessionDefaultConfiguration(toolkit.getSession()));
      messager.addFXTopicListener(topics.getSessionVisualizerDefaultConfigurationSaveRequest(), m -> saveSessionDefaultConfiguration());
   }

   public void startSession(Session session, Runnable sessionLoadedCallback)
   {
      Runnable callback = () ->
      {
         try
         {
            loadSessionDefaultConfiguration(session);
         }
         finally
         {
            if (sessionLoadedCallback != null)
               sessionLoadedCallback.run();
            isFirstSession = false;
         }
      };
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         toolkit.startSession(session, callback);
         mainWindowController.startSession();
      });
   }

   public void stopSession(boolean saveConfiguration, boolean shutdownSession)
   {
      if (!toolkit.hasActiveSession())
         return;

      if (saveConfiguration)
         saveSessionDefaultConfiguration();
      toolkit.stopSession(shutdownSession);
      mainWindowController.stopSession();
      inactiveControllerMap.values().forEach(SessionControlsController::unloadSession);
   }

   public void openRemoteSessionControls()
   {
      openSessionControls(RemoteSessionManagerController.class, SessionVisualizerIOTools.REMOTE_SESSION_MANAGER_PANE_FXML_URL);
   }

   public void openLogSessionControls()
   {
      openSessionControls(LogSessionManagerController.class, SessionVisualizerIOTools.LOG_SESSION_MANAGER_PANE_FXML_URL);
   }

   private void openSessionControls(Class<? extends SessionControlsController> controllerType, URL fxml)
   {
      SessionControlsController activeSessionControls = activeController.get();

      if (activeSessionControls != null)
      {
         if (!controllerType.isInstance(activeSessionControls))
         {
            closeSessionControls(activeSessionControls);
         }
         else
         {
            activeSessionControls.bringUp();
            return;
         }
      }

      SessionControlsController controller = inactiveControllerMap.remove(controllerType);

      if (controller == null)
      {
         try
         {
            FXMLLoader loader = new FXMLLoader(fxml);
            loader.load();
            controller = loader.getController();
            controller.initialize(toolkit);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      SessionInfoController sessionInfoController = controller.getSessionInfoController();

      if (sessionInfoController != null)
      {
         Pane infoPane = sessionInfoController.getMainPane();
         mainWindowController.getSceneAnchorPane().getChildren().add(infoPane);
         AnchorPane.setLeftAnchor(infoPane, 5.0);
         AnchorPane.setBottomAnchor(infoPane, 5.0);
      }

      activeController.set(controller);
      controller.bringUp();
   }

   private void closeSessionControls(SessionControlsController controller)
   {
      SessionInfoController sessionInfo = controller.getSessionInfoController();
      if (sessionInfo != null)
         mainWindowController.getSceneAnchorPane().getChildren().remove(sessionInfo.getMainPane());

      controller.getStage().close();

      if (activeController.get() != null)
         inactiveControllerMap.put(activeController.get().getClass(), activeController.get());

      if (activeController.get() == controller)
         activeController.set(null);
   }

   private String robotName;
   private String sessionName;

   public void loadSessionDefaultConfiguration(Session session)
   {
      if (session.getRobotDefinitions().isEmpty())
         robotName = "UnknownRobot";
      else
         robotName = EuclidCoreIOTools.getCollectionString("-", session.getRobotDefinitions(), RobotDefinition::getName);

      sessionName = session.getSessionName();

      SCSGuiConfiguration configuration = SCSGuiConfiguration.defaultLoader(robotName, sessionName);

      loadSessionConfiguration(configuration);
   }

   private void loadSessionConfiguration(SCSGuiConfiguration configuration)
   {
      if (!configuration.exists())
         return;

      JavaFXMissingTools.runAndWait(getClass(), ()-> toolkit.getWindowManager().closeAllSecondaryWindows());

      SynchronizeHint synchronizeHint = LOAD_SESSION_SYNCHRONOUS ? SynchronizeHint.SYNCHRONOUS : SynchronizeHint.NONE;
      LogTools.info(synchronizeHint);
      long start = System.nanoTime();

      JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();

      if (LOAD_MAIN_WINDOW_CONFIGURATION)
      {
         JavaFXMissingTools.runAndWait(getClass(), () ->
         {
            if (isFirstSession)
            {// TODO When the main window is already up, changing its configuration is quite unpleasant.
               if (configuration.hasMainWindowConfiguration())
                  configuration.getMainWindowConfiguration(toolkit.getMainWindow());
            }
            JavaFXMissingTools.runNFramesLater(2, () -> toolkit.getMainWindow().toFront()); // TODO Should we wait here too?
         });
      }

      if (configuration.hasYoGraphicsConfiguration())
         messager.submitMessage(topics.getYoGraphicLoadRequest(), configuration.getYoGraphicsConfigurationFile(), synchronizeHint);

      if (configuration.hasYoCompositeConfiguration())
         messager.submitMessage(topics.getYoCompositePatternLoadRequest(), configuration.getYoCompositeConfigurationFile(), synchronizeHint);

      if (configuration.hasYoEntryConfiguration())
      {
         YoEntryTabPaneController yoEntryTabPaneController = mainWindowController.getSidePaneController().getYoEntryTabPaneController();
         if (LOAD_SESSION_SYNCHRONOUS)
         {
            JavaFXMissingTools.runAndWait(getClass(), () -> yoEntryTabPaneController.load(configuration.getYoEntryConfigurationFile()));
         }
         else
         {
            JavaFXMissingTools.runLaterWhen(getClass(),
                                            () -> toolkit.getYoCompositeSearchManager().isSessionLoaded(),
                                            () -> yoEntryTabPaneController.load(configuration.getYoEntryConfigurationFile()));
         }
      }

      if (configuration.hasMainYoChartGroupConfiguration())
      {
         LogTools.info("Submitting message to load main charts {}", configuration.getMainYoChartGroupConfigurationFile());
         Pair<Window, File> messageContent = new Pair<>(toolkit.getMainWindow(), configuration.getMainYoChartGroupConfigurationFile());
         messager.submitMessage(topics.getYoChartGroupLoadConfiguration(), messageContent, synchronizeHint);
      }

      toolkit.getWindowManager().loadSessionConfiguration(configuration);

      if (configuration.hasBufferSize())
         messager.submitMessage(topics.getYoBufferInitializeSize(), configuration.getBufferSize());
      if (configuration.hasRecordTickPeriod())
         messager.submitMessage(topics.getInitializeBufferRecordTickPeriod(), configuration.getRecordTickPeriod());
      if (configuration.hasNumberPrecision())
         messager.submitMessage(topics.getControlsNumberPrecision(), configuration.getNumberPrecision());
      mainWindowController.leftSidePaneOpenProperty().set(configuration.getShowYoSearchPanel());
      messager.submitMessage(topics.getShowOverheadPlotter(), configuration.getShowOverheadPlotter());
      messager.submitMessage(topics.getShowAdvancedControls(), configuration.getShowAdvancedControls());
      if (configuration.hasYoSliderboardConfiguration())
         messager.submitMessage(topics.getYoMultiSliderboardLoad(), configuration.getYoSliderboardConfigurationFile(), synchronizeHint);

      if (LOAD_SESSION_TIME)
      {
         long end = System.nanoTime();
         LogTools.info("Loaded session configuration in: {}[sec]", (end - start) * 1.0e-9);
      }
   }

   public void loadSessionConfiguration(File configurationFile)
   {
      if (!configurationFile.exists() || !configurationFile.isFile())
         return;

      try
      {
         File unzippedConfiguration = SessionIOTools.getTemporaryDirectory("configuration");
         SessionIOTools.unzipFile(configurationFile, unzippedConfiguration);
         loadSessionConfiguration(SCSGuiConfiguration.loaderFromDirectory(robotName, sessionName, unzippedConfiguration));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static File newFile(File destinationDir, ZipEntry zipEntry) throws IOException
   {
      File destFile = new File(destinationDir, zipEntry.getName());

      String destDirPath = destinationDir.getCanonicalPath();
      String destFilePath = destFile.getCanonicalPath();

      if (!destFilePath.startsWith(destDirPath + File.separator))
      {
         throw new IOException("Entry is outside of the target dir: " + zipEntry.getName());
      }

      return destFile;
   }

   public void saveSessionDefaultConfiguration()
   {
      SCSGuiConfiguration configuration = SCSGuiConfiguration.defaultSaver(robotName, sessionName);
      // TODO Some things like sliderboard aren't exported systematically, so we don't want to delete these files unless we change the save.
      // Cleanup files with old extensions.
      //      SessionIOTools.emptyDirectory(configuration.getMainConfigurationFile().getParentFile());
      saveSessionConfiguration(configuration);
   }

   private void saveSessionConfiguration(SCSGuiConfiguration configuration)
   {
      // Can't use the messager as the JavaFX is going down which prevents to save properly.
      toolkit.getYoGraphicFXManager().saveYoGraphicToFile(configuration.getYoGraphicsConfigurationFile());
      toolkit.getYoCompositeSearchManager().saveYoCompositePatternToFile(configuration.getYoCompositeConfigurationFile());
      mainWindowController.getSidePaneController().getYoEntryTabPaneController().exportAllTabs(configuration.getYoEntryConfigurationFile());
      mainWindowController.getYoChartGroupPanelController()
                          .saveChartGroupConfiguration(toolkit.getMainWindow(), configuration.getMainYoChartGroupConfigurationFile());
      toolkit.getWindowManager().saveSessionConfiguration(configuration);
      configuration.setMainStage(toolkit.getMainWindow());

      SessionVisualizerTopics topics = toolkit.getTopics();
      SCS2JavaFXMessager messager = toolkit.getMessager();

      int currentBufferSize = toolkit.getYoManager().getBufferSize();
      configuration.setBufferSize(currentBufferSize);
      Integer bufferRecordTickPeriod = messager.getLastValue(topics.getBufferRecordTickPeriod());
      if (bufferRecordTickPeriod != null)
         configuration.setRecordTickPeriod(bufferRecordTickPeriod);
      Integer numberPrecision = messager.getLastValue(topics.getControlsNumberPrecision());
      if (numberPrecision != null)
         configuration.setNumberPrecision(numberPrecision);
      configuration.setShowYoSearchPanel(mainWindowController.leftSidePaneOpenProperty().get());
      configuration.setShowOverheadPlotter(mainWindowController.showOverheadPlotterProperty().getValue());
      configuration.setShowAdvancedControls(mainWindowController.showAdvancedControlsProperty().get());

      configuration.writeConfiguration();
   }

   public void saveSessionConfiguration(File destinationFile)
   {
      try
      {
         File intermediate = SessionIOTools.getTemporaryDirectory("configuration");
         saveSessionConfiguration(SCSGuiConfiguration.saverToDirectory(robotName, sessionName, intermediate));
         SessionIOTools.zipFile(intermediate, destinationFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void shutdown()
   {
      inactiveControllerMap.values().forEach(controller -> controller.shutdown());
   }
}

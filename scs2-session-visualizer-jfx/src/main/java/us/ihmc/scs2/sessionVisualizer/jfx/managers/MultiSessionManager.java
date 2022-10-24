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
import javafx.util.Pair;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionIOTools;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.MainWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.SCSGuiConfiguration;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionInfoController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.LogSessionManagerController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.remote.RemoteSessionManagerController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.BufferedJavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class MultiSessionManager
{
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
      messager.registerTopicListener(topics.getStartNewSessionRequest(), m -> activeSession.set(m));
      messager.registerJavaFXSyncedTopicListener(topics.getRemoteSessionControlsRequest(), m -> openRemoteSessionControls());
      messager.registerJavaFXSyncedTopicListener(topics.getLogSessionControlsRequest(), m -> openLogSessionControls());
      messager.registerJavaFXSyncedTopicListener(topics.getSessionVisualizerConfigurationLoadRequest(), m -> loadSessionConfiguration(m));
      messager.registerJavaFXSyncedTopicListener(topics.getSessionVisualizerConfigurationSaveRequest(), m -> saveSessionConfiguration(m));
      messager.registerJavaFXSyncedTopicListener(topics.getSessionVisualizerDefaultConfigurationLoadRequest(),
                                                 m -> loadSessionDefaultConfiguration(toolkit.getSession()));
      messager.registerJavaFXSyncedTopicListener(topics.getSessionVisualizerDefaultConfigurationSaveRequest(), m -> saveSessionDefaultConfiguration());
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

      JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();

      if (LOAD_MAIN_WINDOW_CONFIGURATION)
      {
         JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
         {
            if (isFirstSession)
            {// TODO When the main window is already up, changing its configuration is quite unpleasant.
               if (configuration.hasMainWindowConfiguration())
                  configuration.getMainWindowConfiguration(toolkit.getMainWindow());
            }
         });
      }

      if (configuration.hasYoGraphicsConfiguration())
         messager.submitMessage(topics.getYoGraphicLoadRequest(), configuration.getYoGraphicsConfigurationFile());

      if (configuration.hasYoCompositeConfiguration())
         messager.submitMessage(topics.getYoCompositePatternLoadRequest(), configuration.getYoCompositeConfigurationFile());

      if (configuration.hasYoEntryConfiguration())
      {
         JavaFXMissingTools.runLaterWhen(getClass(),
                                         () -> toolkit.getYoCompositeSearchManager().isSessionLoaded(),
                                         () -> mainWindowController.getSidePaneController()
                                                                   .getYoEntryTabPaneController()
                                                                   .load(configuration.getYoEntryConfigurationFile()));
      }

      if (configuration.hasMainYoChartGroupConfiguration())
         messager.submitMessage(topics.getYoChartGroupLoadConfiguration(),
                                new Pair<>(toolkit.getMainWindow(), configuration.getMainYoChartGroupConfigurationFile()));

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
         messager.submitMessage(topics.getYoSliderboardLoadConfiguration(), configuration.getYoSliderboardConfigurationFile());
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
      BufferedJavaFXMessager messager = toolkit.getMessager();

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

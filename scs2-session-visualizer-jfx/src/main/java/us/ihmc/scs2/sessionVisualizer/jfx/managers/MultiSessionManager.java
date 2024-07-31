package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.SynchronizeHint;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationListDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionIOTools;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.MainWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.SCSGuiConfiguration;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.YoNameDisplay;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.entry.YoEntryTabPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenSessionControlsRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionInfoController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.LogSessionManagerController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.mcap.MCAPLogSessionManagerController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.remote.RemoteSessionManagerController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.SCS2JavaFXMessager;
import us.ihmc.scs2.symbolic.YoEquationManager.YoEquationListChange;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class MultiSessionManager
{
   private static final boolean LOAD_SESSION_SYNCHRONOUS = SessionPropertiesHelper.loadBooleanPropertyOrEnvironment("scs2.session.gui.loadconfig.synchronous",
                                                                                                                    "SCS2_GUI_LOAD_SESSION_SYNCHRONOUS",
                                                                                                                    true);
   private static final boolean LOAD_SESSION_TIME = SessionPropertiesHelper.loadBooleanProperty("scs2.session.gui.loadconfig.time", false);
   private static final boolean LOAD_MAIN_WINDOW_CONFIGURATION = SessionPropertiesHelper.loadBooleanProperty("scs2.session.gui.mainwindow.loadconfig", true);
   private static final boolean LOAD_BUFFER_SIZE_CONFIGURATION = SessionPropertiesHelper.loadBooleanProperty("scs2.session.gui.buffersize.loadconfig", false);

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
                                         Alert alert = new Alert(AlertType.CONFIRMATION,
                                                                 "Do you want to save the default configuration?",
                                                                 ButtonType.YES,
                                                                 ButtonType.NO);
                                         Stage owner;
                                         if (activeController.get() != null)
                                            owner = activeController.get().getStage();
                                         else
                                            owner = toolkit.getMainWindow();
                                         alert.initOwner(owner);
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
      messager.addFXTopicListener(topics.getOpenSessionControlsRequest(), m -> openSessionControls(m));
      messager.addFXTopicListener(topics.getSessionVisualizerConfigurationLoadRequest(), m -> loadSessionConfiguration(m));
      messager.addFXTopicListener(topics.getSessionVisualizerConfigurationSaveRequest(), m -> saveSessionConfiguration(m, toolkit.getSession()));
      messager.addFXTopicListener(topics.getSessionVisualizerDefaultConfigurationLoadRequest(), m -> loadSessionDefaultConfiguration(toolkit.getSession()));
      messager.addFXTopicListener(topics.getSessionVisualizerDefaultConfigurationSaveRequest(), m -> saveSessionDefaultConfiguration(toolkit.getSession()));
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
         saveSessionDefaultConfiguration(toolkit.getSession());
      toolkit.stopSession(shutdownSession);
      mainWindowController.stopSession();
      inactiveControllerMap.values().forEach(SessionControlsController::unloadSession);
   }

   private void openSessionControls(OpenSessionControlsRequest request)
   {
      URL fxml;
      Class<? extends SessionControlsController> controllerType;

      switch (request.getSessionType())
      {
         case LOG:
         {
            fxml = SessionVisualizerIOTools.LOG_SESSION_MANAGER_PANE_FXML_URL;
            controllerType = LogSessionManagerController.class;
            break;
         }
         case REMOTE:
         {
            fxml = SessionVisualizerIOTools.REMOTE_SESSION_MANAGER_PANE_FXML_URL;
            controllerType = RemoteSessionManagerController.class;
            break;
         }
         case MCAP:
         {
            fxml = SessionVisualizerIOTools.MCAP_LOG_SESSION_MANAGER_PANE_FXML_URL;
            controllerType = MCAPLogSessionManagerController.class;
            break;
         }
         default:
         {
            LogTools.error("Unhandled session type {}", request.getSessionType());
            return;
         }
      }
      openSessionControls(request.getSource(), controllerType, fxml);
   }

   private void openSessionControls(Window source, Class<? extends SessionControlsController> controllerType, URL fxml)
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
            activeSessionControls.bringUp(source);
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
      controller.bringUp(source);
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

      if (configuration == null || configuration.exists())
         loadSessionConfiguration(configuration);
      else // No default configuration, load the one from the robot.
         loadSessionConfiguration(SCSGuiConfiguration.defaultLoader(robotName));
   }

   private void loadSessionConfiguration(SCSGuiConfiguration configuration)
   {
      if (configuration == null || !configuration.exists())
         return;

      JavaFXMissingTools.runAndWait(getClass(), () -> toolkit.getWindowManager().closeAllSecondaryWindows());

      SynchronizeHint synchronizeHint = LOAD_SESSION_SYNCHRONOUS ? SynchronizeHint.SYNCHRONOUS : SynchronizeHint.NONE;
      LogTools.info(synchronizeHint);
      long start = System.nanoTime();

      JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();

      if (configuration.hasYoEquationConfiguration())
      {
         try (InputStream inputStream = new FileInputStream(configuration.getYoEquationConfigurationFile()))
         {
            YoEquationListDefinition yoEquationListDefinition = DefinitionIOTools.loadYoEquationListDefinition(inputStream);
            if (yoEquationListDefinition != null && yoEquationListDefinition.getYoEquations() != null)
               messager.submitMessage(topics.getSessionYoEquationListChangeRequest(), YoEquationListChange.add(yoEquationListDefinition.getYoEquations()));
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }

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

      if (LOAD_BUFFER_SIZE_CONFIGURATION)
      {
         if (configuration.hasBufferSize())
            messager.submitMessage(topics.getYoBufferInitializeSize(), configuration.getBufferSize());
      }
      if (configuration.hasRecordTickPeriod())
         messager.submitMessage(topics.getInitializeBufferRecordTickPeriod(), configuration.getRecordTickPeriod());
      if (configuration.hasNumberPrecision())
         messager.submitMessage(topics.getControlsNumberPrecision(), configuration.getNumberPrecision());
      mainWindowController.leftSidePaneOpenProperty().set(configuration.getShowYoSearchPanel());
      messager.submitMessage(topics.getShowOverheadPlotter(), configuration.getShowOverheadPlotter());
      messager.submitMessage(topics.getShowAdvancedControls(), configuration.getShowAdvancedControls());
      messager.submitMessage(topics.getYoVariableNameDisplay(),
                             configuration.getShowYoVariableUniqueNames() ? YoNameDisplay.UNIQUE_SHORT_NAME : YoNameDisplay.SHORT_NAME);
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

   public void saveSessionDefaultConfiguration(Session session)
   {
      SCSGuiConfiguration configuration = SCSGuiConfiguration.defaultSaver(robotName, sessionName);
      // TODO Some things like sliderboard aren't exported systematically, so we don't want to delete these files unless we change the save.
      // Cleanup files with old extensions.
      //      SessionIOTools.emptyDirectory(configuration.getMainConfigurationFile().getParentFile());
      saveSessionConfiguration(configuration, session);

      // Also save to the default robot location.
      configuration = SCSGuiConfiguration.defaultSaver(robotName);
      saveSessionConfiguration(configuration, session);
   }

   private void saveSessionConfiguration(SCSGuiConfiguration configuration, Session session)
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
      configuration.setShowYoVariableUniqueNames(mainWindowController.yoNameDisplayProperty().getValue() == YoNameDisplay.UNIQUE_SHORT_NAME);

      try (OutputStream outputStream = new FileOutputStream(configuration.getYoEquationConfigurationFile()))
      {
         DefinitionIOTools.saveYoEquationListDefinition(outputStream, session.getYoEquationDefinitions());
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      configuration.writeConfiguration();
   }

   public void saveSessionConfiguration(File destinationFile, Session session)
   {
      try
      {
         File intermediate = SessionIOTools.getTemporaryDirectory("configuration");
         saveSessionConfiguration(SCSGuiConfiguration.saverToDirectory(robotName, sessionName, intermediate), session);
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

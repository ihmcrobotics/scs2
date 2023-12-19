package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import com.jfoenix.controls.JFXTrimSlider;
import javafx.beans.binding.StringBinding;
import javafx.beans.property.LongProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleLongProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableLongValue;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.Pane;
import javafx.stage.DirectoryChooser;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionVariableFilterPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.ToLongFunction;
import java.util.stream.Collectors;

public class LogSessionManagerController implements SessionControlsController
{
   private static final double THUMBNAIL_WIDTH = 200.0;

   private static final String LOG_FILE_KEY = "logFilePath";

   @FXML
   private AnchorPane mainPane;
   @FXML
   private ProgressIndicator loadingSpinner;
   @FXML
   private Button openSessionButton, endSessionButton;
   @FXML
   private Label sessionNameLabel, dateLabel, logPathLabel;
   @FXML
   private Pane cropControlsContainer;
   @FXML
   private ToggleButton showTrimsButton;
   @FXML
   private Button startTrimToCurrentButton, endTrimToCurrentButton, resetTrimsButton, cropAndExportButton;
   @FXML
   private ToggleButton enableVariableFilterToggleButton;
   @FXML
   private ComboBox<OutputFormat> outputFormatComboxBox;
   @FXML
   private JFXTrimSlider logPositionSlider;
   @FXML
   private Pane cropProgressMonitorPane;
   @FXML
   private TitledPane thumbnailsTitledPane;
   @FXML
   private FlowPane videoThumbnailPane;

   private enum OutputFormat
   {
      Default, MATLAB, CSV;
   }

   private final ObjectProperty<MultiVideoViewer> multiVideoViewerProperty = new SimpleObjectProperty<>(this, "multiVideoThumbnailViewer", null);
   private final ObjectProperty<LogSession> activeSessionProperty = new SimpleObjectProperty<>(this, "activeSession", null);
   private final ObjectProperty<YoVariableLogCropper> logCropperProperty = new SimpleObjectProperty<>(this, "logCropper", null);

   private BackgroundExecutorManager backgroundExecutorManager;

   private final ObjectProperty<SessionVariableFilterPaneController> variableFilterControllerProperty = new SimpleObjectProperty<>(null);

   private Stage stage;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit)
   {
      stage = new Stage();

      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      backgroundExecutorManager = toolkit.getBackgroundExecutorManager();

      logPositionSlider.setValueFactory(param -> new TimeStringBinding(param.valueProperty(), position ->
      {
         if (activeSessionProperty.get() == null)
            return 0;
         LogDataReader logDataReader = activeSessionProperty.get().getLogDataReader();
         return logDataReader.getRelativeTimestamp(position.intValue());
      }));

      ChangeListener<? super LogSession> activeSessionListener = (o, oldValue, newValue) ->
      {
         if (oldValue != null)
         {
            LogDataReader logDataReader = oldValue.getLogDataReader();
            logDataReader.getTimestamp().removeListeners();
         }

         if (newValue == null)
         {
            clearControls();
         }
         else
         {
            messager.submitMessage(topics.getStartNewSessionRequest(), newValue);
            initializeControls(newValue);
         }
      };

      AtomicBoolean logPositionUpdate = new AtomicBoolean(true);

      logPositionSlider.valueProperty().addListener((o, oldValue, newValue) ->
                                                    {
                                                       LogSession logSession = activeSessionProperty.get();
                                                       if (logSession == null || logPositionUpdate.get())
                                                          return;

                                                       logSession.submitLogPositionRequest(newValue.intValue());
                                                    });

      AtomicBoolean sliderFeedbackEnabled = new AtomicBoolean(true);

      logPositionSlider.addEventFilter(MouseEvent.MOUSE_PRESSED, e -> sliderFeedbackEnabled.set(false));
      logPositionSlider.addEventFilter(MouseEvent.MOUSE_DRAGGED, e -> sliderFeedbackEnabled.set(false));
      logPositionSlider.addEventFilter(MouseEvent.MOUSE_RELEASED, e -> sliderFeedbackEnabled.set(true));

      Consumer<YoBufferPropertiesReadOnly> logPositionUpdateListener = properties ->
      {
         LogSession logSession = activeSessionProperty.get();

         if (sliderFeedbackEnabled.get())
         {
            int currentLogPosition = logSession.getLogDataReader().getCurrentLogPosition();

            JavaFXMissingTools.runLater(getClass(), () ->
            {
               if (logSession == null || logSession.getLogDataReader() == null)
                  return;

               if (currentLogPosition != logPositionSlider.valueProperty().intValue())
               {
                  logPositionUpdate.set(true);
                  logPositionSlider.setValue(currentLogPosition);
                  logPositionUpdate.set(false);
               }
            });
         }
      };

      activeSessionProperty.addListener((o, oldValue, newValue) ->
                                        {
                                           if (oldValue != null)
                                              oldValue.removeCurrentBufferPropertiesListener(logPositionUpdateListener);
                                           if (newValue != null)
                                              newValue.addCurrentBufferPropertiesListener(logPositionUpdateListener);
                                        });

      multiVideoViewerProperty.addListener((o, oldValue, newValue) ->
                                           {
                                              if (oldValue != null)
                                                 oldValue.stop();
                                              if (newValue != null)
                                                 newValue.start();
                                           });

      if (toolkit.getSession() instanceof LogSession logSession)
      {
         activeSessionProperty.set(logSession);
         initializeControls(logSession);
      }
      else
      {
         clearControls();
      }
      activeSessionProperty.addListener(activeSessionListener);

      thumbnailsTitledPane.expandedProperty().addListener((o, oldValue, newValue) -> JavaFXMissingTools.runLater(getClass(), stage::sizeToScene));

      logPositionSlider.showTrimProperty().bind(showTrimsButton.selectedProperty());
      logPositionSlider.showTrimProperty().addListener((o, oldValue, newValue) ->
                                                       {
                                                          if (newValue)
                                                             resetTrims();
                                                       });
      startTrimToCurrentButton.disableProperty().bind(showTrimsButton.selectedProperty().not());
      endTrimToCurrentButton.disableProperty().bind(showTrimsButton.selectedProperty().not());
      resetTrimsButton.disableProperty().bind(showTrimsButton.selectedProperty().not());
      outputFormatComboxBox.disableProperty().bind(showTrimsButton.selectedProperty().not());
      cropAndExportButton.disableProperty().bind(showTrimsButton.selectedProperty().not());
      cropProgressMonitorPane.getChildren().addListener((ListChangeListener<Node>) c ->
      {
         c.getList().forEach(node -> JavaFXMissingTools.setAnchorConstraints(node, 0.0));
         stage.sizeToScene();
      });

      loadingSpinner.visibleProperty().addListener((o, oldValue, newValue) -> openSessionButton.setDisable(newValue));
      openSessionButton.setOnAction(e -> openLogFile());

      endSessionButton.setOnAction(e ->
                                   {
                                      LogSession logSession = activeSessionProperty.get();
                                      if (logSession != null)
                                         logSession.shutdownSession();
                                      activeSessionProperty.set(null);
                                   });

      outputFormatComboxBox.setItems(FXCollections.observableArrayList(OutputFormat.values()));
      outputFormatComboxBox.getSelectionModel().select(OutputFormat.Default);
      enableVariableFilterToggleButton.setDisable(true); // Only available if export format is MATLAB/CSV for now.

      outputFormatComboxBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
                                                                                   {
                                                                                      if (newValue == OutputFormat.Default || !showTrimsButton.isSelected())
                                                                                      {
                                                                                         enableVariableFilterToggleButton.setSelected(false);
                                                                                         enableVariableFilterToggleButton.setDisable(true);
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                         enableVariableFilterToggleButton.setDisable(false);
                                                                                      }
                                                                                   });

      showTrimsButton.selectedProperty().addListener((o, oldValue, newValue) ->
                                                     {
                                                        if (newValue && outputFormatComboxBox.getSelectionModel().getSelectedItem() != OutputFormat.Default)
                                                        {
                                                           enableVariableFilterToggleButton.setDisable(false);
                                                        }
                                                        else
                                                        {
                                                           enableVariableFilterToggleButton.setSelected(false);
                                                           enableVariableFilterToggleButton.setDisable(true);
                                                        }
                                                     });

      messager.addFXTopicListener(topics.getDisableUserControls(), m ->
      {
         openSessionButton.setDisable(m);
         endSessionButton.setDisable(m);
         cropControlsContainer.setDisable(m);
         logPositionSlider.setDisable(m);
      });

      enableVariableFilterToggleButton.selectedProperty().addListener((o, oldValue, newValue) ->
                                                                      {
                                                                         if (newValue)
                                                                         {
                                                                            if (variableFilterControllerProperty.get() == null)
                                                                            {
                                                                               try
                                                                               {
                                                                                  FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.SESSION_VARIABLE_FILTER_PANE_URL);
                                                                                  loader.load();
                                                                                  SessionVariableFilterPaneController controller = loader.getController();
                                                                                  Set<String> logVariableSet = activeSessionProperty.get()
                                                                                                                                    .getLogDataReader()
                                                                                                                                    .getParser()
                                                                                                                                    .getYoVariablesList()
                                                                                                                                    .stream()
                                                                                                                                    .map(YoVariable::getFullNameString)
                                                                                                                                    .collect(Collectors.toSet());
                                                                                  controller.initialize(toolkit,
                                                                                                        var -> logVariableSet.contains(var.getFullNameString()));
                                                                                  variableFilterControllerProperty.set(controller);
                                                                               }
                                                                               catch (IOException e1)
                                                                               {
                                                                                  e1.printStackTrace();
                                                                               }
                                                                            }

                                                                            cropControlsContainer.getChildren()
                                                                                                 .add(variableFilterControllerProperty.get().getMainPane());
                                                                         }
                                                                         else
                                                                         {
                                                                            if (variableFilterControllerProperty.get() != null)
                                                                               cropControlsContainer.getChildren()
                                                                                                    .remove(variableFilterControllerProperty.get()
                                                                                                                                            .getMainPane());
                                                                         }

                                                                         // Update the preferred size of the window
                                                                         stage.sizeToScene();
                                                                      });

      stage.setScene(new Scene(mainPane));
      stage.setTitle("Log session controls");
      stage.getIcons().add(SessionVisualizerIOTools.LOG_SESSION_IMAGE);
      toolkit.getMainWindow().addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed())
            shutdown();
      });
   }

   private void initializeControls(LogSession newValue)
   {
      File logDirectory = newValue.getLogDirectory();
      LogDataReader logDataReader = newValue.getLogDataReader();
      LogPropertiesReader logProperties = newValue.getLogProperties();

      sessionNameLabel.setText(newValue.getSessionName());
      dateLabel.setText(getDate(logProperties));
      logPathLabel.setText(logDirectory.getAbsolutePath());
      endSessionButton.setDisable(false);
      logPositionSlider.setDisable(false);
      logPositionSlider.setValue(0.0);
      logPositionSlider.setMin(0.0);
      logPositionSlider.setMax(logDataReader.getNumberOfEntries() - 1);
      cropControlsContainer.setDisable(false);
      MultiVideoDataReader multiReader = new MultiVideoDataReader(logDirectory, logProperties, backgroundExecutorManager);
      multiReader.readVideoFrameNow(logDataReader.getTimestamp().getLongValue());
      logDataReader.getTimestamp().addListener(v -> multiReader.readVideoFrameInBackground(v.getValueAsLongBits()));
      multiVideoViewerProperty.set(new MultiVideoViewer(stage, videoThumbnailPane, multiReader, THUMBNAIL_WIDTH));
      logCropperProperty.set(new YoVariableLogCropper(multiReader, logDirectory, logProperties));
      boolean logHasVideos = multiReader.getNumberOfVideos() > 0;
      thumbnailsTitledPane.setText(logHasVideos ? "Logged videos" : "No video");
      thumbnailsTitledPane.setExpanded(logHasVideos);
      thumbnailsTitledPane.setDisable(!logHasVideos);
      JavaFXMissingTools.runNFramesLater(5, () -> stage.sizeToScene());
      JavaFXMissingTools.runNFramesLater(6, () -> stage.toFront());
   }

   private void clearControls()
   {
      sessionNameLabel.setText("N/D");
      dateLabel.setText("N/D");
      logPathLabel.setText("N/D");
      endSessionButton.setDisable(true);
      logPositionSlider.setDisable(true);
      showTrimsButton.setSelected(false);
      cropControlsContainer.setDisable(true);
      multiVideoViewerProperty.set(null);
      logCropperProperty.set(null);
   }

   public void openLogFile()
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setInitialDirectory(SessionVisualizerIOTools.getDefaultFilePath(LOG_FILE_KEY));
      fileChooser.getExtensionFilters().add(new ExtensionFilter("Log property file", "*.log"));
      fileChooser.setTitle("Choose log directory");
      File result = fileChooser.showOpenDialog(stage);
      if (result == null)
         return;

      unloadSession();
      setIsLoading(true);

      backgroundExecutorManager.executeInBackground(() ->
                                                    {
                                                       LogSession newSession;
                                                       try
                                                       {
                                                          LogTools.info("Creating log session.");
                                                          newSession = new LogSession(result.getParentFile(), null); // TODO Need a progress window
                                                          LogTools.info("Created log session.");
                                                          JavaFXMissingTools.runLater(getClass(), () -> activeSessionProperty.set(newSession));
                                                          SessionVisualizerIOTools.setDefaultFilePath(LOG_FILE_KEY, result);
                                                       }
                                                       catch (IOException ex)
                                                       {
                                                          ex.printStackTrace();
                                                          setIsLoading(false);
                                                       }
                                                    });
   }

   public void setIsLoading(boolean isLoading)
   {
      loadingSpinner.setVisible(isLoading);
   }

   @FXML
   public void resetTrims()
   {
      logPositionSlider.setTrimStartValue(0.0);
      logPositionSlider.setTrimEndValue(logPositionSlider.getMax());
   }

   @FXML
   public void snapStartTrimToCurrent()
   {
      logPositionSlider.setTrimStartValue(logPositionSlider.getValue());
   }

   @FXML
   public void snapEndTrimToCurrent()
   {
      logPositionSlider.setTrimEndValue(logPositionSlider.getValue());
   }

   @FXML
   public void cropAndExport() throws IOException
   {
      YoVariableLogCropper logCropper = logCropperProperty.get();
      if (logCropper == null)
         return;

      File destination;

      OutputFormat outputFormat = outputFormatComboxBox.getSelectionModel().getSelectedItem();
      switch (outputFormat)
      {
         case MATLAB:
         {
            destination = SessionVisualizerIOTools.showSaveDialog(stage,
                                                                  "Export MATLAB data",
                                                                  new ExtensionFilter("MATLAB File format", "*.mat"),
                                                                  LOG_FILE_KEY);
            break;
         }
         case CSV:
         {
            destination = SessionVisualizerIOTools.showSaveDialog(stage,
                                                                  "Export CSV data",
                                                                  new ExtensionFilter("Comma-Separated Value format", "*.csv"),
                                                                  LOG_FILE_KEY);
            break;
         }
         case Default:
         default:
         {
            DirectoryChooser directoryChooser = new DirectoryChooser();
            directoryChooser.setInitialDirectory(SessionVisualizerIOTools.getDefaultFilePath(LOG_FILE_KEY));
            destination = directoryChooser.showDialog(stage);
            break;
         }
      }

      if (destination == null)
         return;

      FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.LOG_CROP_PROGRESS_PANE_FXML_URL);
      loader.load();
      LogCropProgressController controller = loader.getController();
      controller.initialize(cropProgressMonitorPane);

      Predicate<YoVariable> variableFilter;
      Predicate<YoRegistry> registryFilter;

      if (enableVariableFilterToggleButton.isSelected())
      {
         SessionVariableFilterPaneController filterController = variableFilterControllerProperty.get();
         variableFilter = filterController.buildVariableFilter();
         registryFilter = filterController.buildRegistryFilter();
      }
      else
      {
         variableFilter = null;
         registryFilter = null;
      }

      int from = (int) logPositionSlider.getTrimStartValue();
      int to = (int) logPositionSlider.getTrimEndValue();
      List<YoVariable> logVariables = activeSessionProperty.get().getLogDataReader().getParser().getYoVariablesList();

      backgroundExecutorManager.executeInBackground(() ->
                                                    {
                                                       setIsLoading(true);
                                                       messager.submitMessage(topics.getDisableUserControls(), true);
                                                       try
                                                       {

                                                          if (outputFormat == OutputFormat.MATLAB)
                                                             logCropper.cropMATLAB(destination,
                                                                                   logVariables,
                                                                                   variableFilter,
                                                                                   registryFilter,
                                                                                   from,
                                                                                   to,
                                                                                   controller);
                                                          else if (outputFormat == OutputFormat.CSV)
                                                             logCropper.cropCSV(destination,
                                                                                logVariables,
                                                                                variableFilter,
                                                                                registryFilter,
                                                                                from,
                                                                                to,
                                                                                controller);
                                                          else
                                                             logCropper.crop(destination, from, to, controller);
                                                       }
                                                       catch (Exception e)
                                                       {
                                                          e.printStackTrace();
                                                          controller.error("Terminated with exception: " + e.getMessage());
                                                          controller.done();
                                                       }
                                                       finally
                                                       {
                                                          messager.submitMessage(topics.getDisableUserControls(), false);
                                                          setIsLoading(false);
                                                       }
                                                    });
   }

   @Override
   public void shutdown()
   {
      if (variableFilterControllerProperty.get() != null)
      {
         variableFilterControllerProperty.get().dispose();
         variableFilterControllerProperty.set(null);
      }
      stage.close();
   }

   @Override
   public Stage getStage()
   {
      return stage;
   }

   @Override
   public void unloadSession()
   {
      enableVariableFilterToggleButton.setSelected(false);
      variableFilterControllerProperty.set(null);

      if (activeSessionProperty.get() != null)
      {
         activeSessionProperty.get().shutdownSession();
         activeSessionProperty.set(null);
      }
   }

   @Override
   public void notifySessionLoaded()
   {
      setIsLoading(false);
   }

   public static class TimeStringBinding extends StringBinding
   {
      private ObservableLongValue observableNanoTime;

      public <N extends Number> TimeStringBinding(ObservableValue<N> time, ToLongFunction<N> conversionToNano)
      {
         LongProperty nanoTimeProperty = new SimpleLongProperty(this, "nanoTime", 0);
         time.addListener((o, oldValue, newValue) -> nanoTimeProperty.set(conversionToNano.applyAsLong(newValue)));
         observableNanoTime = nanoTimeProperty;
         observableNanoTime.addListener((o, oldValue, newValue) -> invalidate());
      }

      @Override
      protected String computeValue()
      {
         long nanoTime = observableNanoTime.get();
         long hours = TimeUnit.NANOSECONDS.toHours(nanoTime);
         long minutes = TimeUnit.NANOSECONDS.toMinutes(nanoTime);
         long seconds = TimeUnit.NANOSECONDS.toSeconds(nanoTime);
         long millis = TimeUnit.NANOSECONDS.toMillis(nanoTime);
         millis -= 1000 * seconds;
         seconds -= 60 * minutes;
         minutes -= 60 * hours;

         String time = String.format("%02ds%03d", seconds, millis);

         if (minutes > 0 || hours > 0)
            time = String.format("%02d", minutes) + "m\n" + time;
         if (hours > 0)
            time = hours + "h" + time;

         return time;
      }
   }

   private static String getDate(LogProperties logProperties)
   {
      String timestampAsString = logProperties.getTimestampAsString();

      String year = timestampAsString.substring(0, 4);
      String month = timestampAsString.substring(4, 6);
      String day = timestampAsString.substring(6, 8);
      String hour = timestampAsString.substring(9, 11);
      String minute = timestampAsString.substring(11, 13);
      String second = timestampAsString.substring(13, 15);

      return year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
   }
}

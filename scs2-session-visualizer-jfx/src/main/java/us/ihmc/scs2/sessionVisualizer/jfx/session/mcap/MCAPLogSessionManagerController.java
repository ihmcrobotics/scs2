package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import com.jfoenix.controls.JFXTextField;
import com.jfoenix.controls.JFXTrimSlider;
import javafx.beans.property.LongProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleLongProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import javafx.util.converter.IntegerStringConverter;
import javafx.util.converter.LongStringConverter;
import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.log.LogTools;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.SessionRobotDefinitionListChange;
import us.ihmc.scs2.session.mcap.MCAPLogFileReader;
import us.ihmc.scs2.session.mcap.MCAPLogSession;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.LogSessionManagerController.TimeStringBinding;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

import java.io.File;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class MCAPLogSessionManagerController implements SessionControlsController
{
   private static final String LOG_FILE_KEY = "MCAPLogFilePath";
   private static final String ROBOT_MODEL_FILE_KEY = "MCAPRobotModelFilePath";

   private static final String DEFAULT_MODEL_TEXT_FIELD_TEXT = "Path to model file";

   @FXML
   private AnchorPane mainPane;
   @FXML
   private Button openSessionButton, endSessionButton;
   @FXML
   private Label sessionNameLabel, dateLabel, logPathLabel;
   @FXML
   private JFXTrimSlider logPositionSlider;
   @FXML
   private JFXTextField currentModelFilePathTextField;
   @FXML
   private TextField desiredLogDTTextField;
   @FXML
   private TextField bufferRecordTickPeriodTextField;

   private final ObjectProperty<MCAPLogSession> activeSessionProperty = new SimpleObjectProperty<>(this, "activeSession", null);

   private Stage stage;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private BackgroundExecutorManager backgroundExecutorManager;

   private File defaultRobotModelFile = null;

   private final LongProperty desiredLogDTProperty = new SimpleLongProperty(this, "desiredLogDT", TimeUnit.MILLISECONDS.toNanos(1));

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
         MCAPLogFileReader mcapLogFileReader = activeSessionProperty.get().getMCAPLogFileReader();
         return mcapLogFileReader.getRelativeTimestampAtIndex(position.intValue());
      }));

      TextFormatter<Integer> recordPeriodFormatter = new TextFormatter<>(new IntegerStringConverter(), 0, new PositiveIntegerValueFilter());
      bufferRecordTickPeriodTextField.setTextFormatter(recordPeriodFormatter);
      messager.bindBidirectional(topics.getBufferRecordTickPeriod(), recordPeriodFormatter.valueProperty(), false);

      desiredLogDTTextField.setTextFormatter(new TextFormatter<>(new LongStringConverter(), 1000L, new PositiveIntegerValueFilter()));
      MutableBoolean desiredLogDTTextFieldUpdate = new MutableBoolean(false);
      desiredLogDTTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                       {
                                                          if (desiredLogDTTextFieldUpdate.isTrue())
                                                             return;
                                                          desiredLogDTTextFieldUpdate.setTrue();
                                                          try
                                                          {
                                                             desiredLogDTProperty.set(TimeUnit.MICROSECONDS.toNanos(Long.parseLong(newValue)));
                                                          }
                                                          catch (NumberFormatException e)
                                                          {
                                                             desiredLogDTTextField.setText(Long.toString(TimeUnit.NANOSECONDS.toMicros(desiredLogDTProperty.get())));
                                                          }
                                                          finally
                                                          {
                                                             desiredLogDTTextFieldUpdate.setFalse();
                                                          }
                                                       });
      desiredLogDTProperty.addListener((o, oldValue, newValue) ->
                                       {
                                          if (desiredLogDTTextFieldUpdate.isTrue())
                                             return;
                                          desiredLogDTTextFieldUpdate.setTrue();
                                          desiredLogDTTextField.setText(Long.toString(TimeUnit.NANOSECONDS.toMicros(newValue.longValue())));
                                          desiredLogDTTextFieldUpdate.setFalse();
                                       });

      ChangeListener<? super MCAPLogSession> activeSessionListener = (o, oldValue, newValue) ->
      {
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
                                                       MCAPLogSession logSession = activeSessionProperty.get();
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
         MCAPLogSession logSession = activeSessionProperty.get();

         if (sliderFeedbackEnabled.get())
         {
            int currentLogPosition = logSession.getMCAPLogFileReader().getCurrentIndex();

            JavaFXMissingTools.runLater(getClass(), () ->
            {
               if (logSession == null || logSession.getMCAPLogFileReader() == null)
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
                                           {
                                              newValue.addCurrentBufferPropertiesListener(logPositionUpdateListener);
                                              if (newValue.getInitialRobotModelFile() != null && defaultRobotModelFile == null)
                                              { // Display the robot model file path if it is available.
                                                currentModelFilePathTextField.setText(newValue.getInitialRobotModelFile().getAbsolutePath());
                                              }
                                              else if (newValue.getInitialRobotModelFile() == null)
                                              {
                                                 currentModelFilePathTextField.setText(DEFAULT_MODEL_TEXT_FIELD_TEXT);
                                              }
                                              // Otherwise the text field will be updated when the robot model file is loaded.
                                           }
                                        });

      if (toolkit.getSession() instanceof MCAPLogSession mcapLogSession)
      {
         desiredLogDTProperty.set(mcapLogSession.getDesiredLogDT());
         activeSessionProperty.set(mcapLogSession);
         initializeControls(mcapLogSession);
      }
      else
      {
         clearControls();
      }

      activeSessionProperty.addListener(activeSessionListener);

      openSessionButton.setOnAction(e -> openLogFile());

      endSessionButton.setOnAction(e ->
                                   {
                                      MCAPLogSession logSession = activeSessionProperty.get();
                                      if (logSession != null)
                                         logSession.shutdownSession();
                                      activeSessionProperty.set(null);
                                   });

      messager.addFXTopicListener(topics.getDisableUserControls(), m ->
      {
         openSessionButton.setDisable(m);
         endSessionButton.setDisable(m);
         logPositionSlider.setDisable(m);
      });

      stage.setScene(new Scene(mainPane));
      stage.setTitle("MCAP Log session controls");
      stage.getIcons().add(SessionVisualizerIOTools.LOG_SESSION_IMAGE);
      toolkit.getMainWindow().addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed())
            shutdown();
      });
      // TODO Auto-generated method stub
   }

   private void initializeControls(MCAPLogSession session)
   {
      File logFile = session.getMCAPFile();
      MCAPLogFileReader mcapLogFileReader = session.getMCAPLogFileReader();

      sessionNameLabel.setText(session.getSessionName());
      dateLabel.setText(getDate(logFile.getName()));
      logPathLabel.setText(logFile.getAbsolutePath());
      endSessionButton.setDisable(false);
      logPositionSlider.setDisable(false);
      logPositionSlider.setValue(0.0);
      logPositionSlider.setMin(0.0);
      logPositionSlider.setMax(mcapLogFileReader.getNumberOfEntries() - 1);
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
   }

   public void openLogFile()
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setInitialDirectory(SessionVisualizerIOTools.getDefaultFilePath(LOG_FILE_KEY));
      fileChooser.getExtensionFilters().add(new ExtensionFilter("MCAP Log file", "*.mcap"));
      fileChooser.setTitle("Choose MCAP log file");
      File result = fileChooser.showOpenDialog(stage);
      if (result == null)
         return;

      unloadSession();

      backgroundExecutorManager.executeInBackground(() ->
                                                    {
                                                       try
                                                       {
                                                          LogTools.info("Creating log session.");
                                                          MCAPLogSession newSession = new MCAPLogSession(result,
                                                                                                         desiredLogDTProperty.get(),
                                                                                                         defaultRobotModelFile);
                                                          LogTools.info("Created log session.");
                                                          JavaFXMissingTools.runLater(getClass(), () -> activeSessionProperty.set(newSession));
                                                          SessionVisualizerIOTools.setDefaultFilePath(LOG_FILE_KEY, result);
                                                       }
                                                       catch (Exception ex)
                                                       {
                                                          ex.printStackTrace();
                                                       }
                                                    });
   }

   @Override
   public void notifySessionLoaded()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void unloadSession()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void shutdown()
   {
      // TODO Auto-generated method stub

      stage.close();
   }

   @Override
   public Stage getStage()
   {
      return stage;
   }

   private static String getDate(String filename)
   { // FIXME it seems that the timestamps in the MCAP file are epoch unix timestamp. Should use that.
      String year = filename.substring(0, 4);
      String month = filename.substring(4, 6);
      String day = filename.substring(6, 8);
      String hour = filename.substring(9, 11);
      String minute = filename.substring(11, 13);
      String second = filename.substring(13, 15);

      return year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
   }

   @FXML
   public void requestLoadRobotModelFile(ActionEvent actionEvent)
   {
      File result = SessionVisualizerIOTools.showOpenDialog(stage,
                                                            "Choose robot model file",
                                                            new ExtensionFilter("Robot model file", "*.urdf", "*.sdf"),
                                                            ROBOT_MODEL_FILE_KEY);
      if (result == null)
         return;

      backgroundExecutorManager.executeInBackground(() -> submitRobotDefinitionRequest(result));
   }

   private void submitRobotDefinitionRequest(File result)
   {
      MCAPLogSession logSession = activeSessionProperty.get();
      if (logSession == null)
      { // Save the file for the next session.
         defaultRobotModelFile = result;
         currentModelFilePathTextField.setText(result.getAbsolutePath());
         return;
      }

      boolean hasARobot = !logSession.getRobotDefinitions().isEmpty();
      SessionRobotDefinitionListChange request;
      if (hasARobot)
         request = SessionRobotDefinitionListChange.replace(result, logSession.getRobotDefinitions().get(0));
      else
         request = SessionRobotDefinitionListChange.add(result);

      messager.submitMessage(topics.getSessionRobotDefinitionListChangeRequest(), request);
      currentModelFilePathTextField.setText("Loading...");
      TopicListener<SessionRobotDefinitionListChange> listener = new TopicListener<SessionRobotDefinitionListChange>()
      {
         @Override
         public void receivedMessageForTopic(SessionRobotDefinitionListChange m)
         {
            if (m.getAddedRobotDefinition() != null)
            {
               currentModelFilePathTextField.setText(result.getAbsolutePath());
               defaultRobotModelFile = result;
            }
            else
            {
               currentModelFilePathTextField.setText("Failed to load.");
               defaultRobotModelFile = null;
            }
            messager.removeFXTopicListener(topics.getSessionRobotDefinitionListChangeState(), this);
         }
      };
      messager.addFXTopicListener(topics.getSessionRobotDefinitionListChangeState(), listener);
   }
}

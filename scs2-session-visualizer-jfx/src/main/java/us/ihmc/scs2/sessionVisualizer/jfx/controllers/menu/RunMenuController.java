package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import javafx.scene.control.CustomMenuItem;
import javafx.scene.control.Menu;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionProperties;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionChangeListener;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class RunMenuController implements VisualizerController
{
   @FXML
   private Menu menu;
   @FXML
   private CustomMenuItem playbackRealTimeRateMenuItem;
   @FXML
   private CustomMenuItem runMaxDurationMenuItem;
   @FXML
   private CheckMenuItem simulateAtRealTimeCheckMenuItem;
   @FXML
   private TextField playbackRealTimeRateTextField;
   @FXML
   private TextField runMaxDurationTextField;

   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;
   private SessionVisualizerWindowToolkit toolkit;

   private final AtomicReference<YoBufferPropertiesReadOnly> bufferProperties = new AtomicReference<>(null);
   private Session session;
   private Consumer<YoBufferPropertiesReadOnly> bufferPropertiesListener;
   private Consumer<SessionProperties> sessionPropertiesListener;
   /** Guards against feedback loops when a session-driven update sets a UI control's value. */
   private boolean updatingFromSession = false;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();

      messager.addFXTopicListener(topics.getDisableUserControls(), disable -> menu.setDisable(disable));

      TextFormatter<Double> playbackRealTimeRateFormatter = new TextFormatter<>(new DoubleStringConverter());
      playbackRealTimeRateFormatter.setValue(1.0);
      playbackRealTimeRateTextField.setTextFormatter(playbackRealTimeRateFormatter);

      TextFormatter<Double> runMaxDurationFormatter = new TextFormatter<>(new DoubleStringConverter());
      runMaxDurationFormatter.setValue(-1.0);
      runMaxDurationTextField.setTextFormatter(runMaxDurationFormatter);

      simulateAtRealTimeCheckMenuItem.selectedProperty().addListener((o, oldValue, newValue) ->
      {
         if (!updatingFromSession && session != null)
            session.submitRunAtRealTimeRate(newValue);
      });
      playbackRealTimeRateFormatter.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (!updatingFromSession && session != null)
            session.submitPlaybackRealTimeRate(newValue);
      });
      runMaxDurationFormatter.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (!updatingFromSession && session != null)
            session.submitRunMaxDuration(newValue != null ? (long) (newValue * 1.0E9) : -1L);
      });

      SessionChangeListener sessionChangeListener = (previousSession, newSession) ->
      {
         if (previousSession != null)
         {
            previousSession.removeCurrentBufferPropertiesListener(bufferPropertiesListener);
            previousSession.removeSessionPropertiesListener(sessionPropertiesListener);
         }

         session = newSession;

         if (newSession == null)
         {
            bufferPropertiesListener = null;
            sessionPropertiesListener = null;
            return;
         }

         bufferPropertiesListener = bufferProperties::set;
         newSession.addCurrentBufferPropertiesListener(bufferPropertiesListener);

         sessionPropertiesListener = properties -> Platform.runLater(() ->
         {
            updatingFromSession = true;
            try
            {
               simulateAtRealTimeCheckMenuItem.setSelected(properties.isRunAtRealTimeRate());
               playbackRealTimeRateFormatter.setValue(properties.getPlaybackRealTimeRate());
               runMaxDurationFormatter.setValue(properties.getRunMaxDuration() < 0 ? -1.0 : properties.getRunMaxDuration() / 1.0E9);
            }
            finally
            {
               updatingFromSession = false;
            }
         });
         newSession.addSessionPropertiesListener(sessionPropertiesListener);
         sessionPropertiesListener.accept(newSession.getSessionProperties());
      };
      toolkit.addAndTriggerSessionChangedListener(sessionChangeListener);

      MenuTools.configureTextFieldForCustomMenuItem(playbackRealTimeRateMenuItem, playbackRealTimeRateTextField);
      MenuTools.configureTextFieldForCustomMenuItem(runMaxDurationMenuItem, runMaxDurationTextField);
   }

   @FXML
   private void startSimulating()
   {
      if (session != null)
         session.setSessionMode(SessionMode.RUNNING);
   }

   @FXML
   private void startPlayback()
   {
      if (session != null)
         session.setSessionMode(SessionMode.PLAYBACK);
   }

   @FXML
   private void pause()
   {
      if (session != null)
         session.setSessionMode(SessionMode.PAUSE);
   }

   @FXML
   private void setInPoint()
   {
      if (session != null && bufferProperties.get() != null)
         session.submitBufferInPointIndexRequest(bufferProperties.get().getCurrentIndex());
   }

   @FXML
   private void gotoInPoint()
   {
      if (session != null && bufferProperties.get() != null)
         session.submitBufferIndexRequest(bufferProperties.get().getInPoint());
   }

   @FXML
   private void stepBack()
   {
      if (session != null)
         session.submitDecrementBufferIndexRequest(1);
   }

   @FXML
   private void stepForward()
   {
      if (session != null)
         session.submitIncrementBufferIndexRequest(1);
   }

   @FXML
   private void gotoOutPoint()
   {
      if (session != null && bufferProperties.get() != null)
         session.submitBufferIndexRequest(bufferProperties.get().getOutPoint());
   }

   @FXML
   private void setOutPoint()
   {
      if (session != null && bufferProperties.get() != null)
         session.submitBufferOutPointIndexRequest(bufferProperties.get().getCurrentIndex());
   }
}

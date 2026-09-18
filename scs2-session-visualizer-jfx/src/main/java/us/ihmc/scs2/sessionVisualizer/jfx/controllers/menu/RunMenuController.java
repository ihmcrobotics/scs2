package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import javafx.scene.control.CustomMenuItem;
import javafx.scene.control.Menu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.messager.SCS2Messager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionProperties;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionChangeListener;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionAdvancedControlsController;
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
   private MenuItem resetMenuItem;
   @FXML
   private TextField playbackRealTimeRateTextField;
   @FXML
   private TextField runMaxDurationTextField;

   private SCS2Messager messager;
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

      // Only Enter commits the typed value to the session. Clicking away without pressing Enter
      // discards the edit and reverts the field to whatever it showed before the edit started -
      // it never submits, and it isn't fighting the periodic session-properties refresh while the
      // user is still typing (see the focus check in the session-properties listener below).
      //
      // Note: TextFormatter#valueProperty() is NOT kept in sync with the field's text as the user
      // types - TextInputControl only ever converts text -> value on commit (its own internal
      // focus-lost listener, or an explicit TextInputControl#commitValue() call), see
      // TextInputControl's constructor and commitValue()/cancelEdit(). So reading getValue() here
      // must be preceded by our own commitValue() call, or it returns the pre-edit value.
      //
      // Also: MenuTools.configureTextFieldForCustomMenuItem() (below) installs its own KEY_PRESSED
      // filter on these fields that consumes ENTER and moves focus to the menu item to close out
      // its own "edit mode" - which happens before the field's own action-event machinery would
      // normally fire, so setOnAction() alone never sees a real Enter press here, and the resulting
      // focus loss would hit the discard branch instead. Catching ENTER with our own filter first
      // (filters on the same node run in the order they were added, and this one is added first)
      // lets us commit before that focus shift happens.
      AtomicReference<Double> playbackRealTimeRateBeforeEdit = new AtomicReference<>(playbackRealTimeRateFormatter.getValue());
      Runnable commitPlaybackRealTimeRate = () ->
      {
         playbackRealTimeRateTextField.commitValue();
         if (session != null && playbackRealTimeRateFormatter.getValue() != null)
            session.submitPlaybackRealTimeRate(playbackRealTimeRateFormatter.getValue());
         playbackRealTimeRateBeforeEdit.set(playbackRealTimeRateFormatter.getValue());
      };
      playbackRealTimeRateTextField.addEventFilter(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.ENTER)
            commitPlaybackRealTimeRate.run();
      });
      playbackRealTimeRateTextField.setOnAction(e -> commitPlaybackRealTimeRate.run());
      playbackRealTimeRateTextField.focusedProperty().addListener((o, wasFocused, isFocused) ->
      {
         if (isFocused)
            playbackRealTimeRateBeforeEdit.set(playbackRealTimeRateFormatter.getValue());
         else
            playbackRealTimeRateFormatter.setValue(playbackRealTimeRateBeforeEdit.get());
      });

      AtomicReference<Double> runMaxDurationBeforeEdit = new AtomicReference<>(runMaxDurationFormatter.getValue());
      Runnable commitRunMaxDuration = () ->
      {
         runMaxDurationTextField.commitValue();
         if (session != null)
         {
            Double value = runMaxDurationFormatter.getValue();
            session.submitRunMaxDuration(value != null ? (long) (value * 1.0E9) : -1L);
         }
         runMaxDurationBeforeEdit.set(runMaxDurationFormatter.getValue());
      };
      runMaxDurationTextField.addEventFilter(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.ENTER)
            commitRunMaxDuration.run();
      });
      runMaxDurationTextField.setOnAction(e -> commitRunMaxDuration.run());
      runMaxDurationTextField.focusedProperty().addListener((o, wasFocused, isFocused) ->
      {
         if (isFocused)
            runMaxDurationBeforeEdit.set(runMaxDurationFormatter.getValue());
         else
            runMaxDurationFormatter.setValue(runMaxDurationBeforeEdit.get());
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
               // Don't clobber a field the user is actively editing.
               if (!playbackRealTimeRateTextField.isFocused())
                  playbackRealTimeRateFormatter.setValue(properties.getPlaybackRealTimeRate());
               if (!runMaxDurationTextField.isFocused())
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

      SessionAdvancedControlsController.bindSessionResetControlVisibility(toolkit.getGlobalToolkit(), resetMenuItem::setVisible);
      SessionAdvancedControlsController.bindSessionResetControlAvailability(toolkit.getGlobalToolkit(), available -> resetMenuItem.setDisable(!available));
   }

   @FXML
   private void resetToInitialState()
   {
      if (session != null)
         session.submitSessionResetRequest();
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

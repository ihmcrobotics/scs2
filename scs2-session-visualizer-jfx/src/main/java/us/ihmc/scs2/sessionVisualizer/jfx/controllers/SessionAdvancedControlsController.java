package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import javafx.application.Platform;
import javafx.beans.InvalidationListener;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.Region;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionProperties;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class SessionAdvancedControlsController implements VisualizerController
{
   public static final String INACTIVE_MODE = "session-controls-inactive-mode";
   public static final String ACTIVE_MODE = "session-controls-active-mode";

   private Window owner;
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;
   private SessionVisualizerWindowToolkit toolkit;

   @FXML
   private FlowPane buttonsContainer;
   @FXML
   private Button previousKeyFrameButton, nextKeyFrameButton;
   @FXML
   private Node runningIconView, playbackIconView, pauseIconView;

   private final Property<YoBufferPropertiesReadOnly> bufferProperties = new SimpleObjectProperty<>(this, "bufferProperties", null);
   private Consumer<YoBufferPropertiesReadOnly> bufferPropertiesListener;

   private BooleanProperty showProperty = new SimpleBooleanProperty(this, "show", false);

   public SessionAdvancedControlsController()
   {
   }

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      owner = toolkit.getWindow();
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();

      toolkit.addAndTriggerSessionChangedListener((previousSession, newSession) ->
      {
         if (previousSession != null)
            previousSession.removeCurrentBufferPropertiesListener(bufferPropertiesListener);

         if (newSession == null)
         {
            bufferPropertiesListener = null;
            return;
         }

         bufferPropertiesListener = properties -> Platform.runLater(() -> bufferProperties.setValue(properties));
         newSession.addCurrentBufferPropertiesListener(bufferPropertiesListener);
      });

      messager.addFXTopicListener(topics.getShowAdvancedControls(), show -> showProperty.set(show));
      messager.addFXTopicListener(topics.getDisableUserControls(), disable -> buttonsContainer.setDisable(disable));

      showProperty.addListener((o, oldValue, newValue) -> show(newValue));
      show(showProperty.get());

      ReadOnlyObjectProperty<int[]> keyFrameIndicesProperty = toolkit.getKeyFrameManager().keyFrameIndicesProperty();
      keyFrameIndicesProperty.addListener((o, oldValue, newValue) ->
                                          {
                                             boolean disableKeyFrameButtons = newValue == null || newValue.length == 0;
                                             previousKeyFrameButton.setDisable(disableKeyFrameButtons);
                                             nextKeyFrameButton.setDisable(disableKeyFrameButtons);
                                          });
      boolean disableKeyFrameButtons = keyFrameIndicesProperty.get() == null || keyFrameIndicesProperty.get().length == 0;
      previousKeyFrameButton.setDisable(disableKeyFrameButtons);
      nextKeyFrameButton.setDisable(disableKeyFrameButtons);

      setupMainControlsActiveMode(this, toolkit, runningIconView, playbackIconView, pauseIconView);
   }

   public static void setupMainControlsActiveMode(Object bean,
                                                  SessionVisualizerWindowToolkit toolkit,
                                                  Node runningIconView,
                                                  Node playbackIconView,
                                                  Node pauseIconView)
   {
      BooleanProperty isRunningActive = new SimpleBooleanProperty(bean, "isRunningActive", false);
      BooleanProperty isPlaybackActive = new SimpleBooleanProperty(bean, "isPlaybackActive", false);
      BooleanProperty isPauseActive = new SimpleBooleanProperty(bean, "isPauseActive", false);
      AtomicReference<Consumer<SessionProperties>> sessionPropertiesListener = new AtomicReference<>();

      toolkit.addAndTriggerSessionChangedListener((previousSession, newSession) ->
      {
         if (previousSession != null && sessionPropertiesListener.get() != null)
            previousSession.removeSessionPropertiesListener(sessionPropertiesListener.get());

         if (newSession == null)
         {
            sessionPropertiesListener.set(null);
            isRunningActive.set(false);
            isPlaybackActive.set(false);
            isPauseActive.set(false);
            return;
         }

         sessionPropertiesListener.set(properties -> Platform.runLater(() ->
         {
            SessionMode mode = properties.getActiveMode();
            isRunningActive.set(mode == SessionMode.RUNNING);
            isPlaybackActive.set(mode == SessionMode.PLAYBACK);
            isPauseActive.set(mode == SessionMode.PAUSE);
         }));
         newSession.addSessionPropertiesListener(sessionPropertiesListener.get());
         sessionPropertiesListener.get().accept(newSession.getSessionProperties());
      });

      setupActiveMode(isRunningActive, runningIconView, ACTIVE_MODE, INACTIVE_MODE);
      setupActiveMode(isPlaybackActive, playbackIconView, ACTIVE_MODE, INACTIVE_MODE);
      setupActiveMode(isPauseActive, pauseIconView, ACTIVE_MODE, INACTIVE_MODE);
   }

   public static void setupActiveMode(ObservableBooleanValue observableActive, Node iconView, String activeStyleClass, String inactiveStyleClass)
   {
      InvalidationListener listener = observable ->
      {
         if (observableActive.get())
         {
            iconView.getStyleClass().remove(inactiveStyleClass);
            iconView.getStyleClass().add(activeStyleClass);
         }
         else
         {
            iconView.getStyleClass().remove(activeStyleClass);
            iconView.getStyleClass().add(inactiveStyleClass);
         }
      };
      observableActive.addListener(listener);
      listener.invalidated(observableActive);
   }

   public BooleanProperty showProperty()
   {
      return showProperty;
   }

   public void show(boolean show)
   {
      buttonsContainer.setVisible(show);
      if (show)
      {
         buttonsContainer.setMinHeight(Region.USE_COMPUTED_SIZE);
         buttonsContainer.setPrefHeight(Region.USE_COMPUTED_SIZE);
      }
      else
      {
         buttonsContainer.setMinHeight(0.0);
         buttonsContainer.setPrefHeight(0.0);
      }
   }

   @FXML
   private void startRunning()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().setSessionMode(SessionMode.RUNNING);
   }

   @FXML
   private void startPlayback()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().setSessionMode(SessionMode.PLAYBACK);
   }

   @FXML
   private void pause()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().setSessionMode(SessionMode.PAUSE);
   }

   @FXML
   private void setInPoint()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().submitBufferInPointIndexRequest(bufferProperties.getValue().getCurrentIndex());
   }

   @FXML
   private void gotoInPoint()
   {
      if (toolkit.getSession() != null && bufferProperties.getValue() != null)
         toolkit.getSession().submitBufferIndexRequest(bufferProperties.getValue().getInPoint());
   }

   @FXML
   private void stepBack()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().submitDecrementBufferIndexRequest(1);
   }

   @FXML
   private void stepForward()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().submitIncrementBufferIndexRequest(1);
   }

   @FXML
   private void gotoOutPoint()
   {
      if (toolkit.getSession() != null && bufferProperties.getValue() != null)
         toolkit.getSession().submitBufferIndexRequest(bufferProperties.getValue().getOutPoint());
   }

   @FXML
   private void setOutPoint()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().submitBufferOutPointIndexRequest(bufferProperties.getValue().getCurrentIndex());
   }

   @FXML
   private void gotoPreviousKeyFrame()
   {
      messager.submitMessage(topics.getGoToPreviousKeyFrame(), new Object());
   }

   @FXML
   private void addRemoveKeyFrame()
   {
      messager.submitMessage(topics.getToggleKeyFrame(), new Object());
   }

   @FXML
   private void gotoNextKeyFrame()
   {
      messager.submitMessage(topics.getGoToNextKeyFrame(), new Object());
   }

   @FXML
   private void requestZoomInGraphs()
   {
      messager.submitMessage(topics.getYoChartRequestZoomIn(), new Pair<>(owner, true));
   }

   @FXML
   private void requestZoomOutGraphs()
   {
      messager.submitMessage(topics.getYoChartRequestZoomOut(), new Pair<>(owner, true));
   }

   @FXML
   private void openSimpleControls()
   {
      messager.submitMessage(topics.getShowAdvancedControls(), false);
   }
}

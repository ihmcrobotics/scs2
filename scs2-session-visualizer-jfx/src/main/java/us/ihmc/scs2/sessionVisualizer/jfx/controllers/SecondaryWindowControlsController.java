package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import com.jfoenix.controls.JFXButton;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.VBox;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.scs2.sessionVisualizer.jfx.messager.SCS2Messager;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.FXCoalescedUpdater;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

import java.util.function.Consumer;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionAdvancedControlsController.setupMainControlsActiveMode;

public class SecondaryWindowControlsController implements VisualizerController
{
   private Window owner;
   private SCS2Messager messager;
   private SessionVisualizerTopics topics;
   private SessionVisualizerWindowToolkit toolkit;

   @FXML
   private VBox mainPane;
   @FXML
   private FlowPane buttonsContainer;
   @FXML
   private JFXButton previousKeyFrameButton, nextKeyFrameButton;
   @FXML
   private Node runningIconView, playbackIconView, pauseIconView;

   private final Property<YoBufferPropertiesReadOnly> bufferProperties = new SimpleObjectProperty<>(this, "bufferProperties", null);
   private Consumer<YoBufferPropertiesReadOnly> bufferPropertiesListener;
   private final FXCoalescedUpdater<YoBufferPropertiesReadOnly> bufferPropertiesUpdater = new FXCoalescedUpdater<>(bufferProperties::setValue);

   public SecondaryWindowControlsController()
   {
   }

   public FlowPane getButtonsContainer()
   {
      return buttonsContainer;
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

         bufferPropertiesListener = bufferPropertiesUpdater::update;
         newSession.addCurrentBufferPropertiesListener(bufferPropertiesListener);
      });

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
      if (toolkit.getSession() != null && bufferProperties.getValue() != null)
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
      if (toolkit.getSession() != null && bufferProperties.getValue() != null)
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

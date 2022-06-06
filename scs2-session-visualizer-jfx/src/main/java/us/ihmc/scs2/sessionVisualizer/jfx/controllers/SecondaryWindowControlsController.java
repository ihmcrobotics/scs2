package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionAdvancedControlsController.setupMainControlsActiveMode;

import com.jfoenix.controls.JFXButton;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.VBox;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

public class SecondaryWindowControlsController implements VisualizerController
{
   private Window owner;
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;

   @FXML
   private VBox mainPane;
   @FXML
   private FlowPane buttonsContainer;
   @FXML
   private JFXButton previousKeyFrameButton, nextKeyFrameButton;
   @FXML
   private FontAwesomeIconView runningIconView, playbackIconView, pauseIconView;

   private Property<YoBufferPropertiesReadOnly> bufferProperties;

   public SecondaryWindowControlsController()
   {
   }

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      owner = toolkit.getWindow();
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();

      bufferProperties = messager.createPropertyInput(topics.getYoBufferCurrentProperties());

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

      setupMainControlsActiveMode(this, messager, topics, runningIconView, playbackIconView, pauseIconView);
   }

   @FXML
   private void startRunning()
   {
      messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.RUNNING);
   }

   @FXML
   private void startPlayback()
   {
      messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PLAYBACK);
   }

   @FXML
   private void pause()
   {
      messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
   }

   @FXML
   private void setInPoint()
   {
      if (bufferProperties.getValue() != null)
         messager.submitMessage(topics.getYoBufferInPointIndexRequest(), bufferProperties.getValue().getCurrentIndex());
   }

   @FXML
   private void gotoInPoint()
   {
      if (bufferProperties.getValue() != null)
         messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), bufferProperties.getValue().getInPoint());
   }

   @FXML
   private void stepBack()
   {
      messager.submitMessage(topics.getYoBufferDecrementCurrentIndexRequest(), 1);
   }

   @FXML
   private void stepForward()
   {
      messager.submitMessage(topics.getYoBufferIncrementCurrentIndexRequest(), 1);
   }

   @FXML
   private void gotoOutPoint()
   {
      if (bufferProperties.getValue() != null)
         messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), bufferProperties.getValue().getOutPoint());
   }

   @FXML
   private void setOutPoint()
   {
      if (bufferProperties.getValue() != null)
         messager.submitMessage(topics.getYoBufferOutPointIndexRequest(), bufferProperties.getValue().getCurrentIndex());
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

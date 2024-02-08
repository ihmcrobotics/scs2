package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import javafx.scene.control.CustomMenuItem;
import javafx.scene.control.Menu;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

import java.util.concurrent.atomic.AtomicReference;

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

   private AtomicReference<YoBufferPropertiesReadOnly> bufferProperties;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      bufferProperties = messager.createInput(topics.getYoBufferCurrentProperties(), null);

      messager.bindBidirectional(topics.getRunAtRealTimeRate(), simulateAtRealTimeCheckMenuItem.selectedProperty(), false);
      messager.addFXTopicListener(topics.getDisableUserControls(), disable -> menu.setDisable(disable));

      {
         TextFormatter<Double> formatter = new TextFormatter<>(new DoubleStringConverter());
         formatter.setValue(1.0);
         playbackRealTimeRateTextField.setTextFormatter(formatter);
         messager.bindBidirectional(topics.getPlaybackRealTimeRate(), formatter.valueProperty(), false);
      }

      {
         TextFormatter<Double> formatter = new TextFormatter<>(new DoubleStringConverter());
         formatter.setValue(-1.0);
         runMaxDurationTextField.setTextFormatter(formatter);
         messager.bindBidirectional(topics.getRunMaxDuration(), formatter.valueProperty(), new PropertyToMessageTypeConverter<Long, Double>()
         {
            @Override
            public Long convert(Double propertyValue)
            {
               if (propertyValue != null)
                  return (long) (propertyValue * 1.0E9);
               return -1L;
            }

            @Override
            public Double interpret(Long messageContent)
            {
               if (messageContent != null)
                  return messageContent.doubleValue() / 1.0E9;
               return -1.0;
            }
         }, false);
      }

      MenuTools.configureTextFieldForCustomMenuItem(playbackRealTimeRateMenuItem, playbackRealTimeRateTextField);
      MenuTools.configureTextFieldForCustomMenuItem(runMaxDurationMenuItem, runMaxDurationTextField);
   }

   @FXML
   private void startSimulating()
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
      if (bufferProperties.get() != null)
         messager.submitMessage(topics.getYoBufferInPointIndexRequest(), bufferProperties.get().getCurrentIndex());
   }

   @FXML
   private void gotoInPoint()
   {
      if (bufferProperties.get() != null)
         messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), bufferProperties.get().getInPoint());
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
      if (bufferProperties.get() != null)
         messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), bufferProperties.get().getOutPoint());
   }

   @FXML
   private void setOutPoint()
   {
      if (bufferProperties.get() != null)
         messager.submitMessage(topics.getYoBufferOutPointIndexRequest(), bufferProperties.get().getCurrentIndex());
   }
}

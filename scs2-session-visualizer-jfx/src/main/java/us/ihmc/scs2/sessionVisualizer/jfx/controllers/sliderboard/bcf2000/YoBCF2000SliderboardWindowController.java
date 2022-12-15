package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import javax.xml.bind.JAXBException;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.image.ImageView;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Button;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Knob;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Slider;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;

public class YoBCF2000SliderboardWindowController
{
   private static final String BCF2000 = BCF2000SliderboardController.BCF2000;
   private static final String CONNECTED_STRING = "Connected to BCF2000 sliderboard";
   private static final String NOT_CONNECTED_STRING = "Not connected to BCF2000 sliderboard";

   @FXML
   private AnchorPane mainAnchorPane;
   @FXML
   private YoBCF2000KnobController knob0Controller, knob1Controller, knob2Controller, knob3Controller, knob4Controller, knob5Controller, knob6Controller,
         knob7Controller;
   @FXML
   private YoBCF2000ButtonController button0Controller, button1Controller, button2Controller, button3Controller, button4Controller, button5Controller,
         button6Controller, button7Controller;
   @FXML
   private YoBCF2000ButtonController button8Controller, button9Controller, button10Controller, button11Controller, button12Controller, button13Controller,
         button14Controller, button15Controller;
   @FXML
   private YoBCF2000SliderController slider0Controller, slider1Controller, slider2Controller, slider3Controller, slider4Controller, slider5Controller,
         slider6Controller, slider7Controller;

   @FXML
   private Label connectionStateLabel;
   @FXML
   private ImageView connectionStateImageView;

   private List<YoBCF2000KnobController> knobControllers;
   private List<YoBCF2000ButtonController> buttonControllers;
   private List<YoBCF2000SliderController> sliderControllers;

   private Stage window;
   private BCF2000SliderboardController sliderboard;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      knobControllers = Arrays.asList(knob0Controller,
                                      knob1Controller,
                                      knob2Controller,
                                      knob3Controller,
                                      knob4Controller,
                                      knob5Controller,
                                      knob6Controller,
                                      knob7Controller);
      buttonControllers = Arrays.asList(button0Controller,
                                        button1Controller,
                                        button2Controller,
                                        button3Controller,
                                        button4Controller,
                                        button5Controller,
                                        button6Controller,
                                        button7Controller,
                                        button8Controller,
                                        button9Controller,
                                        button10Controller,
                                        button11Controller,
                                        button12Controller,
                                        button13Controller,
                                        button14Controller,
                                        button15Controller);
      sliderControllers = Arrays.asList(slider0Controller,
                                        slider1Controller,
                                        slider2Controller,
                                        slider3Controller,
                                        slider4Controller,
                                        slider5Controller,
                                        slider6Controller,
                                        slider7Controller);

      sliderboard = BCF2000SliderboardController.searchAndConnectToDevice();

      if (sliderboard == null)
      {
         LogTools.error("Could not connect to BCF2000 sliderboard");
         connectionStateLabel.setText(NOT_CONNECTED_STRING);
         connectionStateImageView.setImage(SessionVisualizerIOTools.INVALID_ICON_IMAGE);
      }
      else
      {
         connectionStateLabel.setText(CONNECTED_STRING);
         connectionStateImageView.setImage(SessionVisualizerIOTools.VALID_ICON_IMAGE);
      }

      for (int i = 0; i < knobControllers.size(); i++)
      {
         YoBCF2000KnobController yoKnobController = knobControllers.get(i);
         SliderboardVariable knob = sliderboard == null ? null : sliderboard.getKnob(Knob.values()[i]);
         yoKnobController.initialize(toolkit, knob);
      }

      for (int i = 0; i < buttonControllers.size(); i++)
      {
         YoBCF2000ButtonController yoButtonController = buttonControllers.get(i);
         SliderboardVariable button = sliderboard == null ? null : sliderboard.getButton(Button.values()[i]);
         yoButtonController.initialize(toolkit, button);
      }

      for (int i = 0; i < sliderControllers.size(); i++)
      {
         YoBCF2000SliderController yoSliderController = sliderControllers.get(i);
         SliderboardVariable slider = sliderboard == null ? null : sliderboard.getSlider(Slider.values()[i]);
         yoSliderController.initialize(toolkit, slider);
      }

      if (sliderboard != null)
         sliderboard.start();

      window = new Stage(StageStyle.UTILITY);
      window.addEventHandler(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.ESCAPE)
            window.close();
      });

      toolkit.getMainWindow().addEventFilter(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed())
            window.close();
      });
      window.setTitle("YoSliderboard controller");
      window.setScene(new Scene(mainAnchorPane));
      window.initOwner(toolkit.getMainWindow());

      JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();

      File configurationFile = messager.createInput(topics.getYoSliderboardLoadConfiguration()).get();
      if (configurationFile != null)
         load(configurationFile);

      messager.registerJavaFXSyncedTopicListener(topics.getYoSliderboardLoadConfiguration(), this::load);
      messager.registerJavaFXSyncedTopicListener(topics.getYoSliderboardSaveConfiguration(), this::save);
   }

   @FXML
   public void importYoSliderboard()
   {
      File result = SessionVisualizerIOTools.yoSliderboardConfigurationOpenFileDialog(window);

      if (result != null)
         load(result);
   }

   public void load(File file)
   {
      LogTools.info("Loading from file: " + file);

      try
      {
         YoSliderboardListDefinition definition = XMLTools.loadYoSliderboardListDefinition(new FileInputStream(file));
         setInput(definition);
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   @FXML
   public void exportYoSliderboard()
   {
      File result = SessionVisualizerIOTools.yoSliderboardConfigurationSaveFileDialog(window);

      if (result != null)
         save(result);
   }

   public void save(File file)
   {
      LogTools.info("Saving to file: " + file);

      try
      {
         XMLTools.saveYoSliderboardListDefinition(new FileOutputStream(file), toYoSliderboardListDefinition());
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   public void setInput(YoSliderboardListDefinition input)
   {
      List<YoSliderboardDefinition> yoSliderboards = input.getYoSliderboards()
                                                          .stream()
                                                          .filter(yoSliderboard -> BCF2000.equals(yoSliderboard.getType()))
                                                          .collect(Collectors.toList());

      if (yoSliderboards == null || yoSliderboards.isEmpty())
         return;

      YoSliderboardDefinition yoSliderboard = yoSliderboards.get(0);
      if (yoSliderboard.getKnobs() != null)
      {
         for (int i = 0; i < Math.min(knobControllers.size(), yoSliderboard.getKnobs().size()); i++)
         {
            knobControllers.get(i).setInput(yoSliderboard.getKnobs().get(i));
         }
      }
      if (yoSliderboard.getButtons() != null)
      {
         for (int i = 0; i < Math.min(buttonControllers.size(), yoSliderboard.getButtons().size()); i++)
         {
            buttonControllers.get(i).setInput(yoSliderboard.getButtons().get(i));
         }
      }
      if (yoSliderboard.getSliders() != null)
      {
         for (int i = 0; i < Math.min(sliderControllers.size(), yoSliderboard.getSliders().size()); i++)
         {
            sliderControllers.get(i).setInput(yoSliderboard.getSliders().get(i));
         }
      }
   }

   public void showWindow()
   {
      window.setOpacity(0.0);
      window.toFront();
      window.show();
      Timeline timeline = new Timeline();
      KeyFrame key = new KeyFrame(Duration.seconds(0.125), new KeyValue(window.opacityProperty(), 1.0));
      timeline.getKeyFrames().add(key);
      timeline.play();
   }

   public void close()
   {
      for (YoBCF2000ButtonController yoButtonController : buttonControllers)
         yoButtonController.close();
      for (YoBCF2000KnobController yoKnobController : knobControllers)
         yoKnobController.close();
      for (YoBCF2000SliderController yoSliderController : sliderControllers)
         yoSliderController.close();

      if (sliderboard != null)
         sliderboard.closeAndDispose();
      window.close();
   }

   public Stage getWindow()
   {
      return window;
   }

   public YoSliderboardListDefinition toYoSliderboardListDefinition()
   {
      YoSliderboardDefinition definition = new YoSliderboardDefinition();
      definition.setType(BCF2000);
      definition.setKnobs(knobControllers.stream().map(YoBCF2000KnobController::toYoKnobDefinition).collect(Collectors.toList()));
      definition.setButtons(buttonControllers.stream().map(YoBCF2000ButtonController::toYoButtonDefinition).collect(Collectors.toList()));
      definition.setSliders(sliderControllers.stream().map(YoBCF2000SliderController::toYoSliderDefinition).collect(Collectors.toList()));
      return new YoSliderboardListDefinition(null, Collections.singletonList(definition));
   }
}

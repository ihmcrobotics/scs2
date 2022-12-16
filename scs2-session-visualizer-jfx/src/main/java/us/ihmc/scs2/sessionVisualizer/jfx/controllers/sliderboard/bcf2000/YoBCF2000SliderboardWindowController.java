package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import javax.xml.bind.JAXBException;

import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.image.ImageView;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Window;
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

   private final StringProperty nameProperty = new SimpleStringProperty(this, "name", null);

   private List<YoBCF2000KnobController> knobControllers;
   private List<YoBCF2000ButtonController> buttonControllers;
   private List<YoBCF2000SliderController> sliderControllers;
   private List<YoBCF2000InputController> allInputControllers;

   private BCF2000SliderboardController sliderboard;
   private Window owner;

   public void initialize(Window owner, SessionVisualizerToolkit toolkit)
   {
      this.owner = owner;
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
      allInputControllers = new ArrayList<>();
      allInputControllers.addAll(knobControllers);
      allInputControllers.addAll(buttonControllers);
      allInputControllers.addAll(sliderControllers);

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
      File result = SessionVisualizerIOTools.yoSliderboardConfigurationOpenFileDialog(owner);

      if (result != null)
         load(result);
   }

   public void load(File file)
   {
      LogTools.info("Loading from file: " + file);

      try
      {
         YoSliderboardListDefinition definition = XMLTools.loadYoSliderboardListDefinition(new FileInputStream(file));
         if (definition.getYoSliderboards() == null || definition.getYoSliderboards().isEmpty())
            return;
         setInput(definition.getYoSliderboards().get(0));
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   @FXML
   public void exportYoSliderboard()
   {
      File result = SessionVisualizerIOTools.yoSliderboardConfigurationSaveFileDialog(owner);

      if (result != null)
         save(result);
   }

   public void save(File file)
   {
      LogTools.info("Saving to file: " + file);

      try
      {
         XMLTools.saveYoSliderboardListDefinition(new FileOutputStream(file), new YoSliderboardListDefinition(null, toYoSliderboardDefinition()));
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   public void setInput(YoSliderboardDefinition input)
   {
      if (input.getName() != null)
      {
         nameProperty.set(input.getName());
      }

      if (input.getKnobs() != null)
      {
         for (int i = 0; i < Math.min(knobControllers.size(), input.getKnobs().size()); i++)
         {
            knobControllers.get(i).setInput(input.getKnobs().get(i));
         }
      }
      if (input.getButtons() != null)
      {
         for (int i = 0; i < Math.min(buttonControllers.size(), input.getButtons().size()); i++)
         {
            buttonControllers.get(i).setInput(input.getButtons().get(i));
         }
      }
      if (input.getSliders() != null)
      {
         for (int i = 0; i < Math.min(sliderControllers.size(), input.getSliders().size()); i++)
         {
            sliderControllers.get(i).setInput(input.getSliders().get(i));
         }
      }
   }

   public void clear()
   {
      for (YoBCF2000InputController controller : allInputControllers)
      {
         controller.clear();
      }
   }

   public void start()
   {
      if (sliderboard != null)
         sliderboard.start();
   }

   public void stop()
   {
      if (sliderboard != null)
         sliderboard.stop();
   }

   public void close()
   {
      clear();

      if (sliderboard != null)
         sliderboard.closeAndDispose();
   }

   public StringProperty nameProperty()
   {
      return nameProperty;
   }

   public YoSliderboardDefinition toYoSliderboardDefinition()
   {
      YoSliderboardDefinition definition = new YoSliderboardDefinition();
      definition.setName(nameProperty.get());
      definition.setType(BCF2000);
      definition.setKnobs(knobControllers.stream().map(YoBCF2000KnobController::toYoKnobDefinition).collect(Collectors.toList()));
      definition.setButtons(buttonControllers.stream().map(YoBCF2000ButtonController::toYoButtonDefinition).collect(Collectors.toList()));
      definition.setSliders(sliderControllers.stream().map(YoBCF2000SliderController::toYoSliderDefinition).collect(Collectors.toList()));
      return definition;
   }

   public boolean isEmpty()
   {
      for (YoBCF2000InputController controller : allInputControllers)
      {
         if (!controller.isEmpty())
            return false;
      }

      return true;
   }
}

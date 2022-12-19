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
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
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
         SliderboardVariable knob = sliderboard == null ? null : sliderboard.getKnob(Knob.values[i]);
         yoKnobController.initialize(toolkit, Knob.values[i], knob);
      }

      for (int i = 0; i < buttonControllers.size(); i++)
      {
         YoBCF2000ButtonController yoButtonController = buttonControllers.get(i);
         SliderboardVariable button = sliderboard == null ? null : sliderboard.getButton(Button.values[i]);
         yoButtonController.initialize(toolkit, Button.values[i], button);
      }

      for (int i = 0; i < sliderControllers.size(); i++)
      {
         YoBCF2000SliderController yoSliderController = sliderControllers.get(i);
         SliderboardVariable slider = sliderboard == null ? null : sliderboard.getSlider(Slider.values[i]);
         yoSliderController.initialize(toolkit, Slider.values[i], slider);
      }
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
      clear();

      if (input.getName() != null)
      {
         nameProperty.set(input.getName());
      }

      List<YoKnobDefinition> knobs = input.getKnobs();
      if (knobs != null)
      {
         for (int i = 0; i < knobs.size(); i++)
         {
            YoKnobDefinition knob = knobs.get(i);
            if (knob.getIndex() == -1)
               knobControllers.get(i).setInput(knob);
            else
               knobControllers.get(knob.getIndex()).setInput(knob);
         }
      }

      List<YoButtonDefinition> buttons = input.getButtons();
      if (buttons != null)
      {
         for (int i = 0; i < buttons.size(); i++)
         {
            YoButtonDefinition button = buttons.get(i);
            if (button.getIndex() == -1)
               buttonControllers.get(i).setInput(button);
            else
               buttonControllers.get(button.getIndex()).setInput(button);
         }
      }

      List<YoSliderDefinition> sliders = input.getSliders();
      if (sliders != null)
      {
         for (int i = 0; i < sliders.size(); i++)
         {
            YoSliderDefinition slider = sliders.get(i);
            if (slider.getIndex() == -1)
               sliderControllers.get(i).setInput(slider);
            else
               sliderControllers.get(slider.getIndex()).setInput(slider);
         }
      }
   }

   public void setButtonInput(YoButtonDefinition buttonDefinition)
   {
      if (buttonDefinition.getIndex() < 0 || buttonDefinition.getIndex() >= buttonControllers.size())
      {
         LogTools.error("Illegal button index: {}, expected in range: [0, {}[", buttonDefinition.getIndex(), buttonControllers.size());
         return;
      }

      YoBCF2000ButtonController buttonController = buttonControllers.get(buttonDefinition.getIndex());
      buttonController.setInput(buttonDefinition);
   }

   public void removeButtonInput(int buttonIndex)
   {
      if (buttonIndex < 0 || buttonIndex >= buttonControllers.size())
      {
         LogTools.error("Illegal button index: {}, expected in range: [0, {}[", buttonIndex, buttonControllers.size());
         return;
      }

      buttonControllers.get(buttonIndex).clear();
   }

   public void setKnobInput(YoKnobDefinition knobDefinition)
   {
      if (knobDefinition.getIndex() < 0 || knobDefinition.getIndex() >= knobControllers.size())
      {
         LogTools.error("Illegal knob index: {}, expected in range: [0, {}[", knobDefinition.getIndex(), knobControllers.size());
         return;
      }

      YoBCF2000KnobController knobController = knobControllers.get(knobDefinition.getIndex());
      knobController.setInput(knobDefinition);
   }

   public void removeKnobInput(int knobIndex)
   {
      if (knobIndex < 0 || knobIndex >= knobControllers.size())
      {
         LogTools.error("Illegal knob index: {}, expected in range: [0, {}[", knobIndex, knobControllers.size());
         return;
      }

      knobControllers.get(knobIndex).clear();
   }

   public void setSliderInput(YoSliderDefinition sliderDefinition)
   {
      if (sliderDefinition.getIndex() < 0 || sliderDefinition.getIndex() >= sliderControllers.size())
      {
         LogTools.error("Illegal slider index: {}, expected in range: [0, {}[", sliderDefinition.getIndex(), sliderControllers.size());
         return;
      }

      YoBCF2000SliderController sliderController = sliderControllers.get(sliderDefinition.getIndex());
      sliderController.setInput(sliderDefinition);
   }

   public void removeSliderInput(int sliderIndex)
   {
      if (sliderIndex < 0 || sliderIndex >= sliderControllers.size())
      {
         LogTools.error("Illegal slider index: {}, expected in range: [0, {}[", sliderIndex, sliderControllers.size());
         return;
      }

      sliderControllers.get(sliderIndex).clear();
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

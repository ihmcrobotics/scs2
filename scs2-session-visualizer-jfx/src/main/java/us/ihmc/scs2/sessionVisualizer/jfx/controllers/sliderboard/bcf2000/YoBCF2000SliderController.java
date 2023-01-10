package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000;

import com.jfoenix.controls.JFXTextField;

import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.layout.VBox;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoVariableSlider;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoBCF2000SliderController extends YoBCF2000InputController
{
   @FXML
   private VBox rootPane;
   @FXML
   private JFXTextField sliderMaxTextField;
   @FXML
   private JFXTextField sliderMinTextField;
   @FXML
   private Slider slider;
   @FXML
   private Label yoVariableDropLabel;

   private SliderboardVariable sliderVariable;

   private YoVariableSlider yoVariableSlider;
   private YoManager yoManager;
   private BCF2000SliderboardController.Slider sliderChannel;

   public void initialize(SessionVisualizerToolkit toolkit, BCF2000SliderboardController.Slider sliderChannel, SliderboardVariable sliderVariable)
   {
      this.sliderChannel = sliderChannel;
      this.sliderVariable = sliderVariable;
      yoManager = toolkit.getYoManager();
      super.initialize(toolkit, rootPane, yoVariableDropLabel);
      clear();
   }

   @Override
   public void clear()
   {
      if (yoVariableSlider != null)
         yoVariableSlider.dispose();
      yoVariableSlider = null;

      super.clear();

      slider.setDisable(true);
      sliderMaxTextField.setText("");
      sliderMinTextField.setText("");
      sliderMaxTextField.setDisable(true);
      sliderMinTextField.setDisable(true);
   }

   public void setInput(YoSliderDefinition definition)
   {
      if (definition == null)
      {
         clear();
         return;
      }

      YoVariable yoVariable;
      if (definition.getVariableName() != null)
      {
         yoVariable = yoManager.getRootRegistryDatabase().searchExact(definition.getVariableName());
         if (yoVariable == null)
            LogTools.warn("Could not find variable for slider: " + definition.getVariableName());
      }
      else
      {
         yoVariable = null;
      }

      setYoVariableInput(yoVariable, definition.getMinValue(), definition.getMaxValue());
   }

   @Override
   public void setYoVariableInput(YoVariable yoVariable)
   {
      setYoVariableInput(yoVariable, null, null);
   }

   private void setYoVariableInput(YoVariable yoVariable, String minValue, String maxValue)
   {
      if (yoVariableSlider != null)
         yoVariableSlider.dispose();

      if (yoVariable == null)
      {
         clear();
         return;
      }

      slider.setDisable(false);

      yoVariableSlider = YoVariableSlider.newYoVariableSlider(yoVariable, yoManager.getLinkedRootRegistry());
      yoVariableSlider.bindMinTextField(sliderMinTextField);
      yoVariableSlider.bindMaxTextField(sliderMaxTextField);
      if (sliderVariable != null)
         yoVariableSlider.bindSliderVariable(sliderVariable);
      yoVariableSlider.bindVirtualSlider(slider);

      if (minValue != null && !sliderMinTextField.isDisabled())
      {
         if (isMinValid(yoVariable, minValue))
            sliderMinTextField.setText(minValue);
         else
            LogTools.warn("Discarding invalid minValue (={}) for slider bound to the variable {}", minValue, yoVariable);
      }
      if (maxValue != null && !sliderMaxTextField.isDisabled())
      {
         if (isMaxValid(yoVariable, maxValue))
            sliderMaxTextField.setText(maxValue);
         else
            LogTools.warn("Discarding invalid maxValue (={}) for slider bound to the variable {}", maxValue, yoVariable);
      }

      setupYoVariableSlider(yoVariableSlider);
   }

   public YoSliderDefinition toYoSliderDefinition()
   {
      YoSliderDefinition definition = yoVariableSlider == null ? new YoSliderDefinition() : yoVariableSlider.toYoSliderDefinition();
      definition.setIndex(sliderChannel.ordinal());
      return definition;
   }
}

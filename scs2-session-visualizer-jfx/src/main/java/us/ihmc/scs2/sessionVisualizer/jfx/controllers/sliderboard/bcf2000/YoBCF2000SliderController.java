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

   public void initialize(SessionVisualizerToolkit toolkit, SliderboardVariable sliderVariable)
   {
      this.sliderVariable = sliderVariable;
      yoManager = toolkit.getYoManager();
      super.initialize(toolkit, rootPane, yoVariableDropLabel);

      sliderMaxTextField.setText("");
      sliderMinTextField.setText("");
      sliderMaxTextField.setDisable(true);
      sliderMinTextField.setDisable(true);
      slider.setDisable(true);
   }

   public void setInput(YoSliderDefinition definition)
   {
      if (definition == null)
      {
         setYoVariableInput(null);
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
      {
         yoVariableSlider.dispose();
      }

      if (yoVariable != null)
      {
         slider.setDisable(false);

         yoVariableSlider = YoVariableSlider.newYoVariableSlider(yoVariable, yoManager.getLinkedRootRegistry());
         yoVariableSlider.bindMinTextField(sliderMinTextField);
         yoVariableSlider.bindMaxTextField(sliderMaxTextField);
         if (sliderVariable != null)
            yoVariableSlider.bindSliderVariable(sliderVariable);
         yoVariableSlider.bindVirtualSlider(slider);

         if (minValue != null && !sliderMinTextField.isDisabled())
            sliderMinTextField.setText(minValue);
         if (maxValue != null && !sliderMaxTextField.isDisabled())
            sliderMaxTextField.setText(maxValue);

         setupYoVariableSlider(yoVariableSlider);
      }
      else
      {
         clear();
         slider.setDisable(true);
         yoVariableSlider = null;
         sliderMaxTextField.setText("");
         sliderMinTextField.setText("");
      }
   }

   public void close()
   {
      setYoVariableInput(null);
   }

   public YoSliderDefinition toYoSliderDefinition()
   {
      return yoVariableSlider == null ? new YoSliderDefinition() : yoVariableSlider.toYoSliderDefinition();
   }
}

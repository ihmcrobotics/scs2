package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000;

import com.jfoenix.controls.JFXSpinner;
import com.jfoenix.controls.JFXTextField;

import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.layout.VBox;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoVariableSlider;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoBCF2000KnobController extends YoBCF2000InputController
{
   @FXML
   private VBox rootPane;
   @FXML
   private JFXTextField knobMaxTextField;
   @FXML
   private JFXTextField knobMinTextField;
   @FXML
   private JFXSpinner spinner;
   @FXML
   private Label yoVariableDropLabel;

   private SliderboardVariable sliderVariable;

   private YoVariableSlider yoVariableSlider;
   private YoManager yoManager;
   private BCF2000SliderboardController.Knob knobChannel;

   public void initialize(SessionVisualizerToolkit toolkit, BCF2000SliderboardController.Knob knobChannel, SliderboardVariable sliderVariable)
   {
      this.knobChannel = knobChannel;
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
      spinner.setProgress(0);
      spinner.setDisable(true);

      knobMaxTextField.setText("");
      knobMinTextField.setText("");
      knobMaxTextField.setDisable(true);
      knobMinTextField.setDisable(true);

   }

   public void setInput(YoKnobDefinition definition)
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

      spinner.setDisable(false);

      yoVariableSlider = YoVariableSlider.newYoVariableSlider(yoVariable, yoManager.getLinkedRootRegistry());
      yoVariableSlider.bindMinTextField(knobMinTextField);
      yoVariableSlider.bindMaxTextField(knobMaxTextField);
      if (sliderVariable != null)
         yoVariableSlider.bindSliderVariable(sliderVariable);
      yoVariableSlider.bindVirtualKnob(spinner);

      if (minValue != null && !knobMinTextField.isDisabled())
         knobMinTextField.setText(minValue);
      if (maxValue != null && !knobMaxTextField.isDisabled())
         knobMaxTextField.setText(maxValue);
      setupYoVariableSlider(yoVariableSlider);
   }

   public YoKnobDefinition toYoKnobDefinition()
   {
      YoKnobDefinition definition = yoVariableSlider == null ? new YoKnobDefinition() : yoVariableSlider.toYoKnobDefinition();
      definition.setIndex(knobChannel.ordinal());
      return definition;
   }
}

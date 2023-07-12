package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.VBox;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoBooleanSlider;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoVariableSlider;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoBCF2000ButtonController extends YoBCF2000InputController
{
   @FXML
   private VBox rootPane;
   @FXML
   private ToggleButton button;

   private SliderboardVariable sliderVariable;

   private YoBooleanSlider yoBooleanSlider;
   private YoManager yoManager;
   private BCF2000SliderboardController.Button buttonChannel;

   public void initialize(SessionVisualizerToolkit toolkit, BCF2000SliderboardController.Button buttonChannel, SliderboardVariable sliderVariable)
   {
      this.buttonChannel = buttonChannel;
      this.sliderVariable = sliderVariable;
      yoManager = toolkit.getYoManager();
      super.initialize(toolkit, rootPane, button, YoBoolean.class::isInstance);

      clear();
   }

   @Override
   public void clear()
   {
      if (yoBooleanSlider != null)
         yoBooleanSlider.dispose();
      yoBooleanSlider = null;

      super.clear();

      button.setDisable(true);
   }

   public void setInput(YoButtonDefinition definition)
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

      setYoVariableInput(yoVariable);
   }

   @Override
   public void setYoVariableInput(YoVariable yoVariable)
   {
      if (yoBooleanSlider != null)
         yoBooleanSlider.dispose();

      if (yoVariable == null || !(yoVariable instanceof YoBoolean))
      {
         clear();
         return;
      }

      button.setDisable(false);

      yoBooleanSlider = (YoBooleanSlider) YoVariableSlider.newYoVariableSlider(yoVariable, yoManager.getLinkedRootRegistry());
      if (sliderVariable != null)
         yoBooleanSlider.bindSliderVariable(sliderVariable);
      yoBooleanSlider.getYoBooleanProperty().bindBooleanProperty(button.selectedProperty());

      setupYoVariableSlider(yoBooleanSlider);
   }

   public YoButtonDefinition toYoButtonDefinition()
   {
      YoButtonDefinition definition = new YoButtonDefinition();
      if (yoBooleanSlider != null)
         definition.setVariableName(yoBooleanSlider.getYoVariable().getFullNameString());
      definition.setIndex(buttonChannel.ordinal());
      return definition;
   }
}

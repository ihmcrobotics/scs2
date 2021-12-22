package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000;

import com.jfoenix.controls.JFXToggleNode;

import javafx.fxml.FXML;
import javafx.scene.layout.VBox;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoBooleanSlider;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoVariableSlider;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoBCF2000ButtonController extends YoBCF2000InputController
{
   @FXML
   private VBox rootPane;
   @FXML
   private JFXToggleNode button;

   private SliderboardVariable sliderVariable;

   private YoBooleanSlider yoBooleanSlider;
   private YoManager yoManager;

   public void initialize(SessionVisualizerToolkit toolkit, SliderboardVariable sliderVariable)
   {
      this.sliderVariable = sliderVariable;
      yoManager = toolkit.getYoManager();
      super.initialize(toolkit, rootPane, button, YoBoolean.class::isInstance);

      button.setDisable(true);
   }

   public void setInput(YoButtonDefinition definition)
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

      setYoVariableInput(yoVariable);
   }

   @Override
   public void setYoVariableInput(YoVariable yoVariable)
   {
      if (yoBooleanSlider != null)
      {
         yoBooleanSlider.dispose();
         yoBooleanSlider.getYoBooleanProperty().unbind();
      }

      if (yoVariable != null && yoVariable instanceof YoBoolean)
      {
         button.setDisable(false);

         yoBooleanSlider = (YoBooleanSlider) YoVariableSlider.newYoVariableSlider(yoVariable, yoManager.getLinkedRootRegistry());
         if (sliderVariable != null)
            yoBooleanSlider.bindSliderVariable(sliderVariable);
         yoBooleanSlider.getYoBooleanProperty().bindBooleanProperty(button.selectedProperty());

         setupYoVariableSlider(yoBooleanSlider);
      }
      else
      {
         clear();
         button.setDisable(true);
         yoBooleanSlider = null;
      }
   }

   public void close()
   {
      if (yoBooleanSlider != null)
      {
         setYoVariableInput(null);
      }
   }

   public YoButtonDefinition toYoButtonDefinition()
   {
      YoButtonDefinition definition = new YoButtonDefinition();
      if (yoBooleanSlider != null)
         definition.setVariableName(yoBooleanSlider.getYoVariable().getFullNameString());
      return definition;
   }
}

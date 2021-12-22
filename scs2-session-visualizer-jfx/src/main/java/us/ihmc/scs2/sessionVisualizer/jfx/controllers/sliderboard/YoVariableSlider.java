package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard;

import com.jfoenix.controls.JFXSpinner;
import com.jfoenix.controls.JFXTextField;

import javafx.scene.control.Slider;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public interface YoVariableSlider
{
   public static YoVariableSlider newYoVariableSlider(YoVariable yoVariable, LinkedYoRegistry linkedYoRegistry)
   {
      if (yoVariable instanceof YoDouble)
         return new YoDoubleSlider((YoDouble) yoVariable, linkedYoRegistry);
      if (yoVariable instanceof YoBoolean)
         return new YoBooleanSlider((YoBoolean) yoVariable, linkedYoRegistry);
      if (yoVariable instanceof YoInteger)
         return new YoIntegerSlider((YoInteger) yoVariable, linkedYoRegistry);
      if (yoVariable instanceof YoLong)
         return new YoLongSlider((YoLong) yoVariable, linkedYoRegistry);
      if (yoVariable instanceof YoEnum)
         return new YoEnumSlider((YoEnum<?>) yoVariable, linkedYoRegistry);
      throw new IllegalStateException("Unexpected YoVariable type: " + yoVariable);
   }

   void bindVirtualSlider(Slider virtualSlider);

   void bindVirtualKnob(JFXSpinner virtualKnob);

   void bindSliderVariable(SliderboardVariable sliderVariable);

   void bindMinTextField(JFXTextField minTextField);

   void bindMaxTextField(JFXTextField maxTextField);

   YoVariable getYoVariable();

   YoSliderDefinition toYoSliderDefinition();

   YoKnobDefinition toYoKnobDefinition();

   void dispose();
}

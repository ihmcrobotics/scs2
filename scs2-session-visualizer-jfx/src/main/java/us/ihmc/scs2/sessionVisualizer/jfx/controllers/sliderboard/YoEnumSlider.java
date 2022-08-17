package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import com.jfoenix.controls.JFXSpinner;
import com.jfoenix.controls.JFXTextField;

import javafx.beans.value.ChangeListener;
import javafx.scene.control.Slider;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoEnumAsStringProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoEnumSlider implements YoVariableSlider
{
   private final YoEnumAsStringProperty<?> yoEnumProperty;
   private final List<Runnable> cleanupTasks = new ArrayList<>();

   @SuppressWarnings({"rawtypes", "unchecked"})
   public YoEnumSlider(YoEnum<?> yoEnum, LinkedYoRegistry linkedYoRegistry)
   {
      yoEnumProperty = new YoEnumAsStringProperty<>(yoEnum, this);
      yoEnumProperty.setLinkedBuffer(linkedYoRegistry.linkYoVariable((YoEnum) yoEnum, yoEnumProperty));
   }

   @Override
   public void bindMaxTextField(JFXTextField maxTextField)
   {
      maxTextField.setDisable(true);
      cleanupTasks.add(() -> maxTextField.setDisable(false));
   }

   @Override
   public void bindMinTextField(JFXTextField minTextField)
   {
      minTextField.setDisable(true);
      cleanupTasks.add(() -> minTextField.setDisable(false));
   }

   @Override
   public void bindVirtualSlider(Slider virtualSlider)
   {
      virtualSlider.setMin(0.0);
      virtualSlider.setMax(yoEnumProperty.getYoVariable().getEnumValuesAsString().length);
      virtualSlider.setMajorTickUnit(1.0);

      MutableBoolean updating = new MutableBoolean(false);

      ChangeListener<Object> sliderUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         updating.setTrue();
         virtualSlider.valueProperty().set(yoEnumProperty.toEnumOrdinal(yoEnumProperty.get()));
         updating.setFalse();
      };

      ChangeListener<Number> yoEnumUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         JavaFXMissingTools.runLater(YoEnumSlider.this.getClass(), () ->
         {
            updating.setTrue();
            yoEnumProperty.set(yoEnumProperty.toEnumString(virtualSlider.valueProperty().getValue().intValue()));
            updating.setFalse();
         });
      };

      yoEnumProperty.addListener(sliderUpdater);
      virtualSlider.valueProperty().addListener(yoEnumUpdater);

      cleanupTasks.add(() ->
      {
         yoEnumProperty.removeListener(sliderUpdater);
         virtualSlider.valueProperty().removeListener(yoEnumUpdater);
      });
   }

   @Override
   public void bindVirtualKnob(JFXSpinner virtualKnob)
   {
      ChangeListener<Object> knobUpdater = (o, oldValue, newValue) ->
      {
         double value = (double) (yoEnumProperty.toEnumOrdinal(yoEnumProperty.get())) / (double) (yoEnumProperty.getYoVariable().getEnumSize());
         virtualKnob.setProgress(value);
      };

      knobUpdater.changed(null, null, null);

      yoEnumProperty.addListener(knobUpdater);

      cleanupTasks.add(() ->
      {
         yoEnumProperty.removeListener(knobUpdater);
      });
   }

   @Override
   public void bindSliderVariable(SliderboardVariable sliderVariable)
   {
      MutableBoolean updating = new MutableBoolean(false);

      ChangeListener<Object> sliderUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         int sliderPosition = SliderboardVariable.doubleToInt(yoEnumProperty.getYoVariable().getOrdinal(),
                                                              0,
                                                              yoEnumProperty.getYoVariable().getEnumValuesAsString().length - 1,
                                                              sliderVariable.getMin(),
                                                              sliderVariable.getMax());
         updating.setTrue();
         sliderVariable.setValue(sliderPosition);
         updating.setFalse();
      };

      ChangeListener<Number> yoEnumUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         int yoEnumOrdinal = SliderboardVariable.doubleToInt(newValue.intValue(),
                                                             sliderVariable.getMin(),
                                                             sliderVariable.getMax(),
                                                             0,
                                                             yoEnumProperty.getYoVariable().getEnumValuesAsString().length - 1);
         JavaFXMissingTools.runLater(YoEnumSlider.this.getClass(), () ->
         {
            updating.setTrue();
            yoEnumProperty.set(yoEnumProperty.toEnumString(yoEnumOrdinal));
            updating.setFalse();
         });
      };

      yoEnumProperty.addListener(sliderUpdater);
      sliderVariable.valueProperty().addListener(yoEnumUpdater);

      cleanupTasks.add(() ->
      {
         sliderVariable.valueProperty().removeListener(yoEnumUpdater);
         yoEnumProperty.removeListener(sliderUpdater);
      });
   }

   @Override
   public YoEnum<?> getYoVariable()
   {
      return yoEnumProperty.getYoVariable();
   }

   @Override
   public YoSliderDefinition toYoSliderDefinition()
   {
      YoSliderDefinition definition = new YoSliderDefinition();
      definition.setVariableName(getYoVariable().getFullNameString());
      return definition;
   }

   @Override
   public YoKnobDefinition toYoKnobDefinition()
   {
      YoKnobDefinition definition = new YoKnobDefinition();
      definition.setVariableName(getYoVariable().getFullNameString());
      return definition;
   }

   @Override
   public void dispose()
   {
      yoEnumProperty.dispose();
      cleanupTasks.forEach(Runnable::run);
   }
}

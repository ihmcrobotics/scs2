package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import com.jfoenix.controls.JFXSpinner;
import com.jfoenix.controls.JFXTextField;

import javafx.beans.property.LongProperty;
import javafx.beans.property.SimpleLongProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.control.Slider;
import javafx.scene.control.TextFormatter;
import javafx.util.converter.LongStringConverter;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoLongProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class YoLongSlider implements YoVariableSlider
{
   private final YoLongProperty yoLongProperty;
   private final LongProperty minProperty = new SimpleLongProperty(this, "min", 0);
   private final LongProperty maxProperty = new SimpleLongProperty(this, "max", 1);
   private final List<Runnable> cleanupTasks = new ArrayList<>();

   public YoLongSlider(YoLong yoLong, LinkedYoRegistry linkedYoRegistry)
   {
      yoLongProperty = new YoLongProperty(yoLong, this);
      yoLongProperty.setLinkedBuffer(linkedYoRegistry.linkYoVariable(yoLong, yoLongProperty));
   }

   @Override
   public void bindMinTextField(JFXTextField minTextField)
   {
      TextFormatter<Long> minTextFormatter = new TextFormatter<>(new LongStringConverter());
      minTextFormatter.setValue(yoLongProperty.get() - 1);
      minTextField.setTextFormatter(minTextFormatter);
      minProperty.bind(minTextFormatter.valueProperty());
      minTextField.setDisable(false);

      cleanupTasks.add(() ->
      {
         minProperty.unbind();
         minTextField.setDisable(true);
      });
   }

   @Override
   public void bindMaxTextField(JFXTextField maxTextField)
   {
      TextFormatter<Long> maxTextFormatter = new TextFormatter<>(new LongStringConverter());
      maxTextFormatter.setValue(yoLongProperty.get() + 1);
      maxTextField.setTextFormatter(maxTextFormatter);
      maxProperty.bind(maxTextFormatter.valueProperty());
      maxTextField.setDisable(false);

      cleanupTasks.add(() ->
      {
         maxProperty.unbind();
         maxTextField.setDisable(true);
      });
   }

   @Override
   public void bindVirtualSlider(Slider virtualSlider)
   {
      virtualSlider.minProperty().bind(minProperty);
      virtualSlider.maxProperty().bind(maxProperty);
      ChangeListener<Object> majorTickUnitUpdater = (o, oldValue, newValue) ->
      {
         double range = maxProperty.get() - minProperty.get();
         if (range <= 0.0)
            return;
         virtualSlider.setMajorTickUnit(Math.max(1, Math.round(range / 25.0)));
      };

      minProperty.addListener(majorTickUnitUpdater);
      maxProperty.addListener(majorTickUnitUpdater);

      MutableBoolean updating = new MutableBoolean(false);

      virtualSlider.setValue(yoLongProperty.get());

      ChangeListener<Object> sliderUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         updating.setTrue();
         virtualSlider.valueProperty().set(yoLongProperty.get());
         updating.setFalse();
      };

      ChangeListener<Number> yoIntegerUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         JavaFXMissingTools.runLater(YoLongSlider.this.getClass(), () ->
         {
            updating.setTrue();
            yoLongProperty.set(virtualSlider.valueProperty().getValue().longValue());
            updating.setFalse();
         });
      };

      yoLongProperty.addListener(sliderUpdater);
      virtualSlider.valueProperty().addListener(yoIntegerUpdater);

      cleanupTasks.add(() ->
      {
         yoLongProperty.removeListener(sliderUpdater);
         minProperty.removeListener(majorTickUnitUpdater);
         maxProperty.removeListener(majorTickUnitUpdater);
         virtualSlider.valueProperty().removeListener(yoIntegerUpdater);
         virtualSlider.minProperty().unbind();
         virtualSlider.maxProperty().unbind();
      });
   }

   @Override
   public void bindVirtualKnob(JFXSpinner virtualKnob)
   {
      ChangeListener<Number> knobUpdater = (o, oldValue, newValue) ->
      {
         double value = (yoLongProperty.doubleValue() - minProperty.doubleValue()) / (maxProperty.doubleValue() - minProperty.doubleValue());
         virtualKnob.setProgress(value);
      };

      knobUpdater.changed(null, null, null);

      yoLongProperty.addListener(knobUpdater);
      minProperty.addListener(knobUpdater);
      maxProperty.addListener(knobUpdater);

      cleanupTasks.add(() ->
      {
         yoLongProperty.removeListener(knobUpdater);
         minProperty.removeListener(knobUpdater);
         maxProperty.removeListener(knobUpdater);
      });
   }

   @Override
   public void bindSliderVariable(SliderboardVariable sliderVariable)
   {
      MutableBoolean updating = new MutableBoolean(false);

      if (!minProperty.isBound())
         minProperty.set(yoLongProperty.get() - 1);
      if (!maxProperty.isBound())
         maxProperty.set(yoLongProperty.get() + 1);

      sliderVariable.setValue(SliderboardVariable.longToInt(yoLongProperty.get(),
                                                            minProperty.get(),
                                                            maxProperty.get(),
                                                            sliderVariable.getMin(),
                                                            sliderVariable.getMax()));

      ChangeListener<Object> sliderUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         int sliderPosition = SliderboardVariable.longToInt(yoLongProperty.get(),
                                                            minProperty.get(),
                                                            maxProperty.get(),
                                                            sliderVariable.getMin(),
                                                            sliderVariable.getMax());
         updating.setTrue();
         sliderVariable.setValue(sliderPosition);
         updating.setFalse();
      };

      ChangeListener<Number> yoLongUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         Long yoLongValue = SliderboardVariable.intToLong(newValue.intValue(),
                                                          sliderVariable.getMin(),
                                                          sliderVariable.getMax(),
                                                          minProperty.get(),
                                                          maxProperty.get());
         JavaFXMissingTools.runLater(YoLongSlider.this.getClass(), () ->
         {
            updating.setTrue();
            yoLongProperty.set(yoLongValue);
            updating.setFalse();
         });
      };

      yoLongProperty.addListener(sliderUpdater);
      minProperty.addListener(sliderUpdater);
      maxProperty.addListener(sliderUpdater);

      sliderVariable.valueProperty().addListener(yoLongUpdater);

      cleanupTasks.add(() ->
      {
         yoLongProperty.removeListener(sliderUpdater);
         minProperty.removeListener(sliderUpdater);
         maxProperty.removeListener(sliderUpdater);
         sliderVariable.valueProperty().removeListener(yoLongUpdater);
      });
   }

   @Override
   public YoLong getYoVariable()
   {
      return yoLongProperty.getYoVariable();
   }

   @Override
   public YoSliderDefinition toYoSliderDefinition()
   {
      YoSliderDefinition definition = new YoSliderDefinition();
      definition.setVariableName(getYoVariable().getFullNameString());
      definition.setMinValue(minProperty.getValue().toString());
      definition.setMaxValue(maxProperty.getValue().toString());
      return definition;
   }

   @Override
   public YoKnobDefinition toYoKnobDefinition()
   {
      YoKnobDefinition definition = new YoKnobDefinition();
      definition.setVariableName(getYoVariable().getFullNameString());
      definition.setMinValue(minProperty.getValue().toString());
      definition.setMaxValue(maxProperty.getValue().toString());
      return definition;
   }

   @Override
   public void dispose()
   {
      yoLongProperty.dispose();
      cleanupTasks.forEach(Runnable::run);
   }
}

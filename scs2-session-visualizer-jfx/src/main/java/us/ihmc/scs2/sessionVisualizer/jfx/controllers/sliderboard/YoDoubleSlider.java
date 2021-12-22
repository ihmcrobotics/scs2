package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import com.jfoenix.controls.JFXSpinner;
import com.jfoenix.controls.JFXTextField;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.control.Slider;
import javafx.scene.control.TextFormatter;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoDoubleSlider implements YoVariableSlider
{
   private final YoDoubleProperty yoDoubleProperty;
   private final DoubleProperty minProperty = new SimpleDoubleProperty(this, "min", 0.0);
   private final DoubleProperty maxProperty = new SimpleDoubleProperty(this, "max", 1.0);
   private final List<Runnable> cleanupTasks = new ArrayList<>();

   public YoDoubleSlider(YoDouble yoDouble, LinkedYoRegistry linkedYoRegistry)
   {
      yoDoubleProperty = new YoDoubleProperty(yoDouble, this);
      yoDoubleProperty.setLinkedBuffer(linkedYoRegistry.linkYoVariable(yoDouble));
   }

   @Override
   public void bindMinTextField(JFXTextField minTextField)
   {
      TextFormatter<Double> minTextFormatter = new TextFormatter<>(new DoubleStringConverter());
      minTextFormatter.setValue(Math.floor(yoDoubleProperty.get() - 1.0));
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
      TextFormatter<Double> maxTextFormatter = new TextFormatter<>(new DoubleStringConverter());
      maxTextFormatter.setValue(Math.ceil(yoDoubleProperty.get() + 1.0));
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
         virtualSlider.setMajorTickUnit(range / 25.0);
      };

      minProperty.addListener(majorTickUnitUpdater);
      maxProperty.addListener(majorTickUnitUpdater);

      MutableBoolean updating = new MutableBoolean(false);

      virtualSlider.setValue(yoDoubleProperty.get());

      ChangeListener<Object> sliderUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         updating.setTrue();
         virtualSlider.valueProperty().set(yoDoubleProperty.get());
         updating.setFalse();
      };

      ChangeListener<Number> yoDoubleUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         JavaFXMissingTools.runLater(YoDoubleSlider.this.getClass(), () ->
         {
            updating.setTrue();
            yoDoubleProperty.setAndPush(virtualSlider.valueProperty().get());
            updating.setFalse();
         });
      };

      yoDoubleProperty.addListener(sliderUpdater);
      virtualSlider.valueProperty().addListener(yoDoubleUpdater);

      cleanupTasks.add(() ->
      {
         yoDoubleProperty.removeListener(sliderUpdater);
         minProperty.removeListener(majorTickUnitUpdater);
         maxProperty.removeListener(majorTickUnitUpdater);
         virtualSlider.valueProperty().removeListener(yoDoubleUpdater);
         virtualSlider.minProperty().unbind();
         virtualSlider.maxProperty().unbind();
      });
   }

   @Override
   public void bindVirtualKnob(JFXSpinner virtualKnob)
   {
      ChangeListener<Number> knobUpdater = (o, oldValue, newValue) ->
      {
         double value = (yoDoubleProperty.get() - minProperty.get()) / (maxProperty.get() - minProperty.get());
         virtualKnob.setProgress(value);
      };

      knobUpdater.changed(null, null, null);

      yoDoubleProperty.addListener(knobUpdater);
      minProperty.addListener(knobUpdater);
      maxProperty.addListener(knobUpdater);

      cleanupTasks.add(() ->
      {
         yoDoubleProperty.removeListener(knobUpdater);
         minProperty.removeListener(knobUpdater);
         maxProperty.removeListener(knobUpdater);
      });
   }

   @Override
   public void bindSliderVariable(SliderboardVariable sliderVariable)
   {
      MutableBoolean updating = new MutableBoolean(false);

      if (!minProperty.isBound())
         minProperty.set(Math.floor(yoDoubleProperty.get() - 1.0));
      if (!maxProperty.isBound())
         maxProperty.set(Math.ceil(yoDoubleProperty.get() + 1.0));

      sliderVariable.setValue(SliderboardVariable.doubleToInt(yoDoubleProperty.get(),
                                                              minProperty.get(),
                                                              maxProperty.get(),
                                                              sliderVariable.getMin(),
                                                              sliderVariable.getMax()));

      ChangeListener<Object> sliderUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         int sliderPosition = SliderboardVariable.doubleToInt(yoDoubleProperty.get(),
                                                              minProperty.get(),
                                                              maxProperty.get(),
                                                              sliderVariable.getMin(),
                                                              sliderVariable.getMax());
         updating.setTrue();
         sliderVariable.setValue(sliderPosition);
         updating.setFalse();
      };

      ChangeListener<Number> yoDoubleUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         double yoDoubleValue = SliderboardVariable.intToDouble(newValue.intValue(),
                                                                sliderVariable.getMin(),
                                                                sliderVariable.getMax(),
                                                                minProperty.get(),
                                                                maxProperty.get());
         JavaFXMissingTools.runLater(YoDoubleSlider.this.getClass(), () ->
         {
            updating.setTrue();
            yoDoubleProperty.setAndPush(yoDoubleValue);
            updating.setFalse();
         });
      };

      yoDoubleProperty.addListener(sliderUpdater);
      minProperty.addListener(sliderUpdater);
      maxProperty.addListener(sliderUpdater);

      sliderVariable.valueProperty().addListener(yoDoubleUpdater);

      cleanupTasks.add(() ->
      {
         yoDoubleProperty.removeListener(sliderUpdater);
         minProperty.removeListener(sliderUpdater);
         maxProperty.removeListener(sliderUpdater);
         sliderVariable.valueProperty().removeListener(yoDoubleUpdater);
      });
   }

   @Override
   public YoDouble getYoVariable()
   {
      return yoDoubleProperty.getYoVariable();
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
      yoDoubleProperty.finalize();
      cleanupTasks.forEach(Runnable::run);
   }
}

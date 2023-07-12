package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import com.jfoenix.controls.JFXSpinner;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.util.converter.IntegerStringConverter;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoIntegerProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.SliderboardVariable;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoIntegerSlider implements YoVariableSlider
{
   private final YoIntegerProperty yoIntegerProperty;
   private final IntegerProperty minProperty = new SimpleIntegerProperty(this, "min", 0);
   private final IntegerProperty maxProperty = new SimpleIntegerProperty(this, "max", 1);
   private final List<Runnable> cleanupTasks = new ArrayList<>();

   public YoIntegerSlider(YoInteger yoInteger, LinkedYoRegistry linkedYoRegistry)
   {
      yoIntegerProperty = new YoIntegerProperty(yoInteger, this);
      yoIntegerProperty.setLinkedBuffer(linkedYoRegistry.linkYoVariable(yoInteger, yoIntegerProperty));
   }

   @Override
   public void bindMinTextField(TextField minTextField)
   {
      TextFormatter<Integer> minTextFormatter = new TextFormatter<>(new IntegerStringConverter());
      minTextFormatter.setValue(yoIntegerProperty.get() - 1);
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
   public void bindMaxTextField(TextField maxTextField)
   {
      TextFormatter<Integer> maxTextFormatter = new TextFormatter<>(new IntegerStringConverter());
      maxTextFormatter.setValue(yoIntegerProperty.get() + 1);
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

      virtualSlider.setValue(yoIntegerProperty.get());

      ChangeListener<Object> sliderUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         updating.setTrue();
         virtualSlider.valueProperty().set(yoIntegerProperty.get());
         updating.setFalse();
      };

      ChangeListener<Number> yoIntegerUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         JavaFXMissingTools.runLater(YoIntegerSlider.this.getClass(), () ->
         {
            updating.setTrue();
            yoIntegerProperty.set(virtualSlider.valueProperty().getValue().intValue());
            updating.setFalse();
         });
      };

      yoIntegerProperty.addListener(sliderUpdater);
      virtualSlider.valueProperty().addListener(yoIntegerUpdater);

      cleanupTasks.add(() ->
      {
         yoIntegerProperty.removeListener(sliderUpdater);
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
         double value = (yoIntegerProperty.doubleValue() - minProperty.doubleValue()) / (maxProperty.doubleValue() - minProperty.doubleValue());
         virtualKnob.setProgress(value);
      };

      knobUpdater.changed(null, null, null);

      yoIntegerProperty.addListener(knobUpdater);
      minProperty.addListener(knobUpdater);
      maxProperty.addListener(knobUpdater);

      cleanupTasks.add(() ->
      {
         yoIntegerProperty.removeListener(knobUpdater);
         minProperty.removeListener(knobUpdater);
         maxProperty.removeListener(knobUpdater);
      });
   }

   @Override
   public void bindSliderVariable(SliderboardVariable sliderVariable)
   {
      MutableBoolean updating = new MutableBoolean(false);

      if (!minProperty.isBound())
         minProperty.set(yoIntegerProperty.get() - 1);
      if (!maxProperty.isBound())
         maxProperty.set(yoIntegerProperty.get() + 1);

      sliderVariable.setValue(SliderboardVariable.doubleToInt(yoIntegerProperty.get(),
                                                              minProperty.get(),
                                                              maxProperty.get(),
                                                              sliderVariable.getMin(),
                                                              sliderVariable.getMax()));

      ChangeListener<Object> sliderUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         int sliderPosition = SliderboardVariable.doubleToInt(yoIntegerProperty.get(),
                                                              minProperty.get(),
                                                              maxProperty.get(),
                                                              sliderVariable.getMin(),
                                                              sliderVariable.getMax());
         updating.setTrue();
         sliderVariable.setValue(sliderPosition);
         updating.setFalse();
      };

      ChangeListener<Number> yoIntegerUpdater = (o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         int yoIntegerValue = SliderboardVariable.doubleToInt(newValue.intValue(),
                                                              sliderVariable.getMin(),
                                                              sliderVariable.getMax(),
                                                              minProperty.get(),
                                                              maxProperty.get());
         JavaFXMissingTools.runLater(YoIntegerSlider.this.getClass(), () ->
         {
            updating.setTrue();
            yoIntegerProperty.set(yoIntegerValue);
            updating.setFalse();
         });
      };

      yoIntegerProperty.addListener(sliderUpdater);
      minProperty.addListener(sliderUpdater);
      maxProperty.addListener(sliderUpdater);

      sliderVariable.valueProperty().addListener(yoIntegerUpdater);

      cleanupTasks.add(() ->
      {
         yoIntegerProperty.removeListener(sliderUpdater);
         minProperty.removeListener(sliderUpdater);
         maxProperty.removeListener(sliderUpdater);
         sliderVariable.valueProperty().removeListener(yoIntegerUpdater);
      });
   }

   @Override
   public YoInteger getYoVariable()
   {
      return yoIntegerProperty.getYoVariable();
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
      yoIntegerProperty.dispose();
      cleanupTasks.forEach(Runnable::run);
   }
}

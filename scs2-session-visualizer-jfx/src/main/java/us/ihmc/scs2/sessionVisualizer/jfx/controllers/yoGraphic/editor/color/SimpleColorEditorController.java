package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.color;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.fxml.FXML;
import javafx.scene.control.ColorPicker;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.RoundedDoubleConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public class SimpleColorEditorController extends PaintEditorController<SimpleColorFX>
{
   @FXML
   private Pane mainPane;

   @FXML
   private Label colorLabel;

   @FXML
   private ColorPicker colorPicker;

   @FXML
   private Label opacityLabel;

   @FXML
   private Slider opacitySlider;

   @FXML
   private TextField opacityTextField;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit)
   {
      super.initialize(toolkit, new SimpleColorFX(YoGraphicFX3D.DEFAULT_COLOR));

      TextFormatter<Double> value = new TextFormatter<>(new RoundedDoubleConverter(), opacitySlider.getValue(), new PositiveIntegerValueFilter());
      opacityTextField.setTextFormatter(value);
      value.valueProperty().bindBidirectional(opacitySlider.valueProperty().asObject());

      MutableBoolean enableColorBinding = new MutableBoolean(true);
      colorPicker.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (enableColorBinding.isFalse())
            return;
         enableColorBinding.setFalse();
         SimpleColorFX newSimpleColor = new SimpleColorFX(newValue);
         if (colorProperty.getValue() != null)
         { // Preserve opacity
            newSimpleColor = changeOpacity(newSimpleColor, colorProperty.getValue().getOpacity());
         }
         colorProperty.setValue(newSimpleColor);
         enableColorBinding.setTrue();
      });

      opacitySlider.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (enableColorBinding.isFalse())
            return;

         enableColorBinding.setFalse();
         colorProperty.setValue(changeOpacity(colorProperty.getValue(), 0.01 * newValue.doubleValue()));
         enableColorBinding.setTrue();
      });

      colorProperty.addListener((o, oldValue, newValue) ->
      {
         if (enableColorBinding.isFalse())
            return;

         enableColorBinding.setFalse();
         colorPicker.valueProperty().set(newValue.get());
         opacitySlider.setValue(100.0 * newValue.get().getOpacity());
         enableColorBinding.setTrue();
      });

      inputsValidityProperty = new SimpleBooleanProperty(this, "inputsValidility", true);
   }

   @Override
   public void setInput(PaintDefinition input)
   {
      if (input instanceof ColorDefinition colorDefinition)
         setInput(colorDefinition);
      else
         LogTools.error("Unexpected input: {}", input);
   }

   public void setInput(ColorDefinition definition)
   {
      colorProperty.setValue(new SimpleColorFX(JavaFXVisualTools.toColor(definition, YoGraphicFX3D.DEFAULT_COLOR)));
   }

   private static SimpleColorFX changeOpacity(SimpleColorFX original, double newOpacity)
   {
      Color color = original.get();
      double red = color.getRed();
      double green = color.getGreen();
      double blue = color.getBlue();
      return new SimpleColorFX(new Color(red, green, blue, newOpacity));
   }

   @Override
   public Pane getMainPane()
   {
      return mainPane;
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor;

import org.apache.commons.lang3.mutable.MutableBoolean;

import com.jfoenix.controls.JFXColorPicker;
import com.jfoenix.controls.JFXSlider;
import com.jfoenix.controls.JFXTextField;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextFormatter;
import javafx.scene.layout.GridPane;
import javafx.scene.paint.Color;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.RoundedDoubleConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;

public class YoGraphic3DStyleEditorPaneController
{
   @FXML
   private GridPane mainPane;

   @FXML
   private Label colorLabel;

   @FXML
   private JFXColorPicker colorPicker;

   @FXML
   private Label opacityLabel;

   @FXML
   private JFXSlider opacitySlider;

   @FXML
   private JFXTextField opacityTextField;

   private final ObjectProperty<Color> colorProperty = new SimpleObjectProperty<>(this, "color", null);
   private final ObservableBooleanValue inputsValidityProperty = new SimpleBooleanProperty(this, "inputsValidity", true);

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      colorPicker.valueProperty().bindBidirectional(colorProperty);

      TextFormatter<Double> value = new TextFormatter<>(new RoundedDoubleConverter(), opacitySlider.getValue(), new PositiveIntegerValueFilter());
      opacityTextField.setTextFormatter(value);
      value.valueProperty().bindBidirectional(opacitySlider.valueProperty().asObject());

      MutableBoolean updatingOpacity = new MutableBoolean(false);
      MutableBoolean updatingSlider = new MutableBoolean(false);

      opacitySlider.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (updatingSlider.isFalse())
         {
            updatingOpacity.setTrue();
            colorProperty.set(changeOpacity(colorProperty.get(), newValue.doubleValue()));
            updatingOpacity.setFalse();
         }
      });

      colorProperty.addListener((o, oldValue, newValue) ->
      {
         if (updatingOpacity.isFalse())
         {
            updatingSlider.setTrue();
            opacitySlider.setValue(100.0 * newValue.getOpacity());
            updatingSlider.setFalse();
         }
      });
   }

   public void setInput(YoGraphic3DDefinition definition)
   {
      setInput(JavaFXVisualTools.toColor(definition.getColor(), YoGraphicFX3D.DEFAULT_COLOR));
   }

   public void setInput(Color color)
   {
      colorPicker.setValue(color);
      opacitySlider.setValue(100.0 * color.getOpacity());
   }

   public void bindYoGraphicFX3D(YoGraphicFX3D yoGraphicFX3DToBind)
   {
      colorProperty.addListener((o, oldValue, newValue) -> yoGraphicFX3DToBind.setColor(newValue));
   }

   public void addInputNotification(Runnable callback)
   {
      colorProperty.addListener((o, oldValue, newValue) -> callback.run());
   }

   private static Color changeOpacity(Color original, double newOpacity)
   {
      double red = original.getRed();
      double green = original.getGreen();
      double blue = original.getBlue();
      return new Color(red, green, blue, 0.01 * newOpacity);
   }

   public ObservableBooleanValue inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   public ReadOnlyObjectProperty<Color> colorProperty()
   {
      return colorProperty;
   }

   public GridPane getMainPane()
   {
      return mainPane;
   }
}

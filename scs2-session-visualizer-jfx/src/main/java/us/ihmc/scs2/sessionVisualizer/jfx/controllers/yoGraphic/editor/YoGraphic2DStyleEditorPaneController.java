package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools.bindValidityImageView;

import com.jfoenix.controls.JFXColorPicker;
import com.jfoenix.controls.JFXRadioButton;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleGroup;
import javafx.scene.image.ImageView;
import javafx.scene.layout.GridPane;
import javafx.scene.paint.Color;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic2DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX2D;

public class YoGraphic2DStyleEditorPaneController
{
   public static final double DEFAULT_STROKE_WIDTH = YoGraphicFX2D.DEFAULT_STROKE_WIDTH.get();

   @FXML
   private GridPane mainPane;
   @FXML
   private JFXRadioButton fillRadioButton;
   @FXML
   private JFXColorPicker fillColorPicker;
   @FXML
   private JFXRadioButton strokeRadioButton;
   @FXML
   private JFXColorPicker strokeColorPicker;
   @FXML
   private TextField strokeWidthTextField;
   @FXML
   private ImageView strokeWidthValidImageView;

   private final ObjectProperty<Color> fillColorProperty = new SimpleObjectProperty<>(this, "fillColor", null);
   private final ObjectProperty<Color> strokeColorProperty = new SimpleObjectProperty<>(this, "strokeColor", null);
   private final BooleanProperty strokeWidthValidityProperty = new SimpleBooleanProperty(this, "strokeWidthValidityProperty", false);
   private final DoubleProperty strokeWidthProperty = new SimpleDoubleProperty(this, "strokeWidth", DEFAULT_STROKE_WIDTH);
   private ObservableBooleanValue inputsValidityProperty;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      new ToggleGroup().getToggles().addAll(fillRadioButton, strokeRadioButton);

      fillColorPicker.disableProperty().bind(strokeRadioButton.selectedProperty());
      strokeColorPicker.disableProperty().bind(fillRadioButton.selectedProperty());

      YoGraphicFXControllerTools.numericalValidityBinding(strokeWidthTextField.textProperty(), strokeWidthValidityProperty);

      bindValidityImageView(strokeWidthValidityProperty, strokeWidthValidImageView);

      inputsValidityProperty = strokeWidthValidityProperty;

      strokeWidthTextField.textProperty().addListener((observable, oldValue, newValue) ->
      {
         if (strokeWidthValidityProperty.get())
            strokeWidthProperty.set(Double.parseDouble(newValue));
      });

      fillRadioButton.selectedProperty().addListener((observable, oldValue, newValue) ->
      {
         if (newValue)
         {
            fillColorProperty.set(fillColorPicker.getValue());
            strokeColorProperty.set(null);
         }
         else
         {
            strokeColorProperty.set(strokeColorPicker.getValue());
            fillColorProperty.set(null);
         }
      });

      fillColorPicker.valueProperty().addListener((o, oldValue, newValue) -> fillColorProperty.set(newValue));
      strokeColorPicker.valueProperty().addListener((o, oldValue, newValue) -> strokeColorProperty.set(newValue));
   }

   public void setInput(YoGraphic2DDefinition definition)
   {
      setInput(JavaFXVisualTools.toColor(definition.getFillColor(), null),
               JavaFXVisualTools.toColor(definition.getStrokeColor(), null),
               definition.getStrokeWidth());
   }

   public void setInput(Color fillColor, Color strokeColor, String strokeWidth)
   {
      if (fillColor != null)
      {
         fillRadioButton.setSelected(true);
         fillColorPicker.setValue(fillColor);
      }
      else
      {
         strokeRadioButton.setSelected(true);
         strokeColorPicker.setValue(strokeColor);
      }

      strokeWidthTextField.setText(strokeWidth);
   }

   public void bindYoGraphicFX2D(YoGraphicFX2D yoGraphicFX2DToBind)
   {
      fillColorProperty.addListener((o, oldValue, newValue) -> yoGraphicFX2DToBind.setFillColor(newValue));
      strokeColorProperty.addListener((o, oldValue, newValue) -> yoGraphicFX2DToBind.setStrokeColor(newValue));
      strokeWidthProperty.addListener((o, oldValue, newValue) -> yoGraphicFX2DToBind.setStrokeWidth(newValue.doubleValue()));
   }

   public void addInputNotification(Runnable callback)
   {
      ChangeListener<Object> changeListener = (o, oldValue, newValue) -> callback.run();
      fillColorProperty.addListener(changeListener);
      strokeColorProperty.addListener(changeListener);
      strokeWidthProperty.addListener(changeListener);
   }

   public ObservableBooleanValue inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   public ReadOnlyObjectProperty<Color> fillColorProperty()
   {
      return fillColorProperty;
   }

   public ReadOnlyObjectProperty<Color> strokeColorProperty()
   {
      return strokeColorProperty;
   }

   public ReadOnlyDoubleProperty strokeWidthProperty()
   {
      return strokeWidthProperty;
   }

   public GridPane getMainPane()
   {
      return mainPane;
   }
}

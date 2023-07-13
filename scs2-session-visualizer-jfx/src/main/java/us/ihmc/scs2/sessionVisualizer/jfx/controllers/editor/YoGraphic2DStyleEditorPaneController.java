package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools.bindValidityImageView;

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
import javafx.scene.control.RadioButton;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleGroup;
import javafx.scene.image.ImageView;
import javafx.scene.layout.GridPane;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic2DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.color.ColorEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX2D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;

public class YoGraphic2DStyleEditorPaneController
{
   public static final double DEFAULT_STROKE_WIDTH = YoGraphicFX2D.DEFAULT_STROKE_WIDTH.get();

   @FXML
   private GridPane mainPane;
   @FXML
   private RadioButton fillRadioButton;
   @FXML
   private ColorEditorController fillColorEditorController;
   @FXML
   private RadioButton strokeRadioButton;
   @FXML
   private ColorEditorController strokeColorEditorController;
   @FXML
   private TextField strokeWidthTextField;
   @FXML
   private ImageView strokeWidthValidImageView;

   private final ObjectProperty<BaseColorFX> fillColorProperty = new SimpleObjectProperty<>(this, "fillColor", null);
   private final ObjectProperty<BaseColorFX> strokeColorProperty = new SimpleObjectProperty<>(this, "strokeColor", null);
   private final BooleanProperty strokeWidthValidityProperty = new SimpleBooleanProperty(this, "strokeWidthValidityProperty", false);
   private final DoubleProperty strokeWidthProperty = new SimpleDoubleProperty(this, "strokeWidth", DEFAULT_STROKE_WIDTH);
   private ObservableBooleanValue inputsValidityProperty;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      new ToggleGroup().getToggles().addAll(fillRadioButton, strokeRadioButton);

      fillColorEditorController.initialize(toolkit);
      strokeColorEditorController.initialize(toolkit);
      fillColorEditorController.getMainPane().disableProperty().bind(strokeRadioButton.selectedProperty());
      strokeColorEditorController.getMainPane().disableProperty().bind(fillRadioButton.selectedProperty());

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
            fillColorProperty.bind(fillColorEditorController.colorProperty());
            strokeColorProperty.unbind();
            strokeColorProperty.set(null);
         }
         else
         {
            strokeColorProperty.bind(strokeColorEditorController.colorProperty());
            fillColorProperty.unbind();
            fillColorProperty.set(null);
         }
      });

      // Initialize
      if (fillRadioButton.isSelected())
      {
         fillColorProperty.bind(fillColorEditorController.colorProperty());
         strokeColorProperty.unbind();
         strokeColorProperty.set(null);
      }
      else
      {
         strokeColorProperty.bind(strokeColorEditorController.colorProperty());
         fillColorProperty.unbind();
         fillColorProperty.set(null);
      }
   }

   public void setInput(YoGraphic2DDefinition definition)
   {
      setInput(definition.getFillColor(), definition.getStrokeColor(), definition.getStrokeWidth());
   }

   public void setInput(PaintDefinition fillColor, PaintDefinition strokeColor, String strokeWidth)
   {
      if (fillColor != null)
      {
         fillRadioButton.setSelected(true);
         fillColorEditorController.setInput(fillColor);
      }
      else
      {
         strokeRadioButton.setSelected(true);
         strokeColorEditorController.setInput(strokeColor);
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

   public ReadOnlyObjectProperty<BaseColorFX> fillColorProperty()
   {
      return fillColorEditorController.colorProperty();
   }

   public ReadOnlyObjectProperty<BaseColorFX> strokeColorProperty()
   {
      return strokeColorEditorController.colorProperty();
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

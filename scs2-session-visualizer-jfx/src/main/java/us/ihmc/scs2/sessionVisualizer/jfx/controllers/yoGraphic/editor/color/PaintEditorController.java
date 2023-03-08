package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.color;

import java.util.function.BiConsumer;

import javafx.beans.binding.Bindings;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Pane;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField.YoDoubleTextField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField.YoIntegerTextField;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

public abstract class PaintEditorController<C extends BaseColorFX>
{
   protected ObservableBooleanValue inputsValidityProperty;

   protected final Property<C> colorProperty = new SimpleObjectProperty<>(this, "color", null);

   protected SessionVisualizerToolkit toolkit;
   protected YoCompositeSearchManager yoCompositeSearchManager;

   public abstract void initialize(SessionVisualizerToolkit toolkit);

   public void initialize(SessionVisualizerToolkit toolkit, C defaultColor)
   {
      this.toolkit = toolkit;
      yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();

      colorProperty.setValue(defaultColor);
   }

   public abstract void setInput(PaintDefinition input);

   protected void setupDoublePropertyEditor(TextField textField, ImageView validImageView, BiConsumer<C, DoubleProperty> colorComponentSetter)
   {
      LinkedYoRegistry linkedRootRegistry = toolkit.getYoManager().getLinkedRootRegistry();
      YoDoubleTextField yoDoubleTextField = new YoDoubleTextField(textField, yoCompositeSearchManager, linkedRootRegistry, validImageView);
      yoDoubleTextField.setupAutoCompletion();

      yoDoubleTextField.supplierProperty().addListener((o, oldValue, newValue) ->
      {
         colorComponentSetter.accept(colorProperty.getValue(), newValue);
      });
      //      textField.textProperty().addListener(this::updateHasChangesPendingProperty);
      if (inputsValidityProperty == null)
         inputsValidityProperty = yoDoubleTextField.getValidityProperty();
      else
         inputsValidityProperty = Bindings.and(inputsValidityProperty, yoDoubleTextField.getValidityProperty());
   }

   protected void setupIntegerPropertyEditor(TextField textField, ImageView validImageView, BiConsumer<C, IntegerProperty> colorComponentSetter)
   {
      LinkedYoRegistry linkedRootRegistry = toolkit.getYoManager().getLinkedRootRegistry();
      YoIntegerTextField yoIntegerTextField = new YoIntegerTextField(textField, yoCompositeSearchManager, linkedRootRegistry, validImageView);
      yoIntegerTextField.setupAutoCompletion();

      yoIntegerTextField.supplierProperty().addListener((o, oldValue, newValue) ->
      {
         colorComponentSetter.accept(colorProperty.getValue(), newValue);
      });
      //      textField.textProperty().addListener(this::updateHasChangesPendingProperty);
      if (inputsValidityProperty == null)
         inputsValidityProperty = yoIntegerTextField.getValidityProperty();
      else
         inputsValidityProperty = Bindings.and(inputsValidityProperty, yoIntegerTextField.getValidityProperty());
   }

   public ReadOnlyProperty<C> colorProperty()
   {
      return colorProperty;
   }

   public ObservableBooleanValue inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   public abstract Pane getMainPane();
}

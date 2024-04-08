package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor;

import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.shape.DrawMode;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.color.ColorEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;

public class YoGraphic3DStyleEditorPaneController
{
   @FXML
   private VBox mainPane;

   @FXML
   private ColorEditorController colorEditorController;

   @FXML
   private Pane drawModeContainer;
   @FXML
   private ComboBox<DrawMode> drawModeComboBox;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      colorEditorController.initialize(toolkit);

      drawModeComboBox.getItems().addAll(DrawMode.values());
      drawModeComboBox.getSelectionModel().select(DrawMode.FILL);
   }

   public void setInput(YoGraphic3DDefinition definition)
   {
      setInput(definition.getColor());
   }

   public void setInput(PaintDefinition definition)
   {
      colorEditorController.setInput(definition);
   }

   public void setInput(String drawMode)
   {
      if (drawMode == null)
      {
         drawModeComboBox.setValue(DrawMode.FILL);
         return;
      }

      drawMode = drawMode.trim();

      if (DrawMode.LINE.name().equalsIgnoreCase(drawMode) || "WIREFRAME".equalsIgnoreCase(drawMode))
         drawModeComboBox.setValue(DrawMode.LINE);
      else
         drawModeComboBox.setValue(DrawMode.FILL);
   }

   public void bindYoGraphicFX3D(YoGraphicFX3D yoGraphicFX3DToBind)
   {
      colorEditorController.colorProperty().addListener((o, oldValue, newValue) -> yoGraphicFX3DToBind.setColor(newValue));
      drawModeComboBox.valueProperty().addListener((o, oldValue, newValue) -> yoGraphicFX3DToBind.setDrawMode(newValue));
   }

   public void addInputNotification(Runnable callback)
   {
      colorEditorController.colorProperty().addListener((o, oldValue, newValue) -> callback.run());
      drawModeComboBox.valueProperty().addListener((o, oldValue, newValue) -> callback.run());
   }

   public ObservableBooleanValue inputsValidityProperty()
   {
      return colorEditorController.inputsValidityProperty();
   }

   public ReadOnlyObjectProperty<BaseColorFX> colorProperty()
   {
      return colorEditorController.colorProperty();
   }

   public Property<DrawMode> drawModeProperty()
   {
      return drawModeComboBox.valueProperty();
   }

   public Pane getMainPane()
   {
      return mainPane;
   }
}

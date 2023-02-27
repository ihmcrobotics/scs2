package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor;

import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.color.ColorEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;

public class YoGraphic3DStyleEditorPaneController
{
   @FXML
   private VBox mainPane;

   @FXML
   private ColorEditorController colorEditorController;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      colorEditorController.initialize(toolkit);
   }

   public void setInput(YoGraphic3DDefinition definition)
   {
      setInput(definition.getColor());
   }

   public void setInput(PaintDefinition definition)
   {
      colorEditorController.setInput(definition);
   }

   public void bindYoGraphicFX3D(YoGraphicFX3D yoGraphicFX3DToBind)
   {
      colorEditorController.colorProperty().addListener((o, oldValue, newValue) -> yoGraphicFX3DToBind.setColor(newValue));
   }

   public void addInputNotification(Runnable callback)
   {
      colorEditorController.colorProperty().addListener((o, oldValue, newValue) -> callback.run());
   }

   public ObservableBooleanValue inputsValidityProperty()
   {
      return colorEditorController.inputsValidityProperty();
   }

   public ReadOnlyObjectProperty<BaseColorFX> colorProperty()
   {
      return colorEditorController.colorProperty();
   }

   public Pane getMainPane()
   {
      return mainPane;
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.binding.Bindings;
import javafx.fxml.FXML;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphic3DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;

public abstract class YoGraphicFX3DEditorController<G extends YoGraphicFX3D> extends YoGraphicFXEditorController<G>
{
   @FXML
   protected YoGraphic3DStyleEditorPaneController styleEditorController;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, G yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      styleEditorController.initialize(toolkit);
      styleEditorController.bindYoGraphicFX3D(yoGraphicToEdit);
      styleEditorController.addInputNotification(() -> updateHasChangesPendingProperty(null, null, null));

      inputsValidityProperty = Bindings.and(inputsValidityProperty, styleEditorController.inputsValidityProperty());
   }
}

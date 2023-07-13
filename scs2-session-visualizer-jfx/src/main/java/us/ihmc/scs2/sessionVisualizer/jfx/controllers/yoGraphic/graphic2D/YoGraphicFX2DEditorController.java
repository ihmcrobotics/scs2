package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic2D;

import javafx.beans.binding.Bindings;
import javafx.fxml.FXML;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphic2DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX2D;

public abstract class YoGraphicFX2DEditorController<G extends YoGraphicFX2D> extends YoGraphicFXEditorController<G>
{
   @FXML
   protected YoGraphic2DStyleEditorPaneController styleEditorController;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, G yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      styleEditorController.initialize(toolkit);
      styleEditorController.bindYoGraphicFX2D(yoGraphicToEdit);
      styleEditorController.addInputNotification(() -> updateHasChangesPendingProperty(null, null, null));

      inputsValidityProperty = Bindings.and(inputsValidityProperty, styleEditorController.inputsValidityProperty());
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.graphic3D;

import javafx.beans.binding.Bindings;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.layout.VBox;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphic3DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphicNameEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoBoxFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

public class YoBoxFX3DGroupEditorController extends YoGroupFXEditorController<YoBoxFX3D>
{
   @FXML
   private VBox mainPane;
   @FXML
   private YoCompositeEditorPaneController sizeEditorController;
   @FXML
   private YoGraphic3DStyleEditorPaneController styleEditorController;
   @FXML
   private YoGraphicNameEditorPaneController nameEditorController;

   private ObservableBooleanValue inputsValidityProperty;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGroupFX yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      this.yoGraphicToEdit = yoGraphicToEdit;

      setupTuple3DEditor(sizeEditorController, false, "Size", YoBoxFX3D::setSize, YoBoxFX3D::getSize);
      setupStyleEditor(styleEditorController);
      setupNameEditor(nameEditorController);

      inputsValidityProperty = Bindings.and(sizeEditorController.inputsValidityProperty(), nameEditorController.inputsValidityProperty());

      resetFields();
   }

   @Override
   public ObservableBooleanValue inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   @Override
   public VBox getMainPane()
   {
      return mainPane;
   }

   @Override
   public Class<YoBoxFX3D> getChildrenCommonType()
   {
      return YoBoxFX3D.class;
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicSTPBox3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoSTPBoxFX3D;

public class YoSTPBoxFX3DEditorController extends YoGraphicFX3DEditorController<YoSTPBoxFX3D>
{
   @FXML
   private YoCompositeEditorPaneController positionEditorController, orientationEditorController;
   @FXML
   private YoCompositeEditorPaneController sizeEditorController;
   @FXML
   private TextField minimumMarginTextField, maximumMarginTextField;
   @FXML
   private ImageView minimumMarginValidImageView, maximumMarginValidImageView;

   private YoGraphicSTPBox3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoSTPBoxFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicSTPBox3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple3DPropertyEditor(positionEditorController, "Position", true, yoGraphicToEdit.getPosition());
      setupOrientation3DProperty(orientationEditorController, "Orientation", true, yoGraphicToEdit.getOrientation());
      setupTuple3DPropertyEditor(sizeEditorController, "Size", false, yoGraphicToEdit.getSize());
      setupDoublePropertyEditor(minimumMarginTextField, minimumMarginValidImageView, YoSTPBoxFX3D::setMinimumMargin);
      setupDoublePropertyEditor(maximumMarginTextField, maximumMarginValidImageView, YoSTPBoxFX3D::setMaximumMargin);

      resetFields();
   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicSTPBox3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      positionEditorController.setInput(definitionBeforeEdits.getPosition());
      orientationEditorController.setInput(definitionBeforeEdits.getOrientation());
      sizeEditorController.setInput(definitionBeforeEdits.getSize());
      minimumMarginTextField.setText(definitionBeforeEdits.getMinimumMargin());
      maximumMarginTextField.setText(definitionBeforeEdits.getMaximumMargin());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicSTPBox3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

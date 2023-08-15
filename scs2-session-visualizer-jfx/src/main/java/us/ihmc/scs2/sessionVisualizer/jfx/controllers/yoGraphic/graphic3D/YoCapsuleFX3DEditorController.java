package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCapsule3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoCapsuleFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;

public class YoCapsuleFX3DEditorController extends YoGraphicFX3DEditorController<YoCapsuleFX3D>
{
   @FXML
   private YoCompositeEditorPaneController centerEditorController, axisEditorController;
   @FXML
   private TextField lengthTextField, radiusTextField;

   @FXML
   private ImageView lengthValidImageView, radiusValidImageView;

   private YoGraphicCapsule3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoCapsuleFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicCapsule3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple3DPropertyEditor(centerEditorController, "Center", true, yoGraphicToEdit.getCenter());
      setupTuple3DPropertyEditor(axisEditorController, "Axis", true, yoGraphicToEdit.getAxis());
      setupDoublePropertyEditor(lengthTextField, lengthValidImageView, YoCapsuleFX3D::setLength);
      setupDoublePropertyEditor(radiusTextField, radiusValidImageView, YoCapsuleFX3D::setRadius);

      resetFields();
   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicCapsule3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      centerEditorController.setInput(definitionBeforeEdits.getCenter());
      axisEditorController.setInput(definitionBeforeEdits.getAxis());
      lengthTextField.setText(definitionBeforeEdits.getLength());
      radiusTextField.setText(definitionBeforeEdits.getRadius());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicCapsule3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

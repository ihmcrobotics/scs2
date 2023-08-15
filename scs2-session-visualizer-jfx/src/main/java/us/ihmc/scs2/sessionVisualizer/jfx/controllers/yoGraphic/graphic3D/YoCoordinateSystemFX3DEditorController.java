package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoCoordinateSystemFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;

public class YoCoordinateSystemFX3DEditorController extends YoGraphicFX3DEditorController<YoCoordinateSystemFX3D>
{
   @FXML
   private YoCompositeEditorPaneController positionEditorController, orientationEditorController;
   @FXML
   private TextField bodyLengthTextField, headLengthTextField;
   @FXML
   private TextField bodyRadiusTextField, headRadiusTextField;

   @FXML
   private ImageView bodyLengthValidImageView, headLengthValidImageView;
   @FXML
   private ImageView bodyRadiusValidImageView, headRadiusValidImageView;

   private YoGraphicCoordinateSystem3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoCoordinateSystemFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicCoordinateSystem3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple3DPropertyEditor(positionEditorController, "Position", true, yoGraphicToEdit.getPosition());
      setupOrientation3DProperty(orientationEditorController, "Orientation", true, yoGraphicToEdit.getOrientation());
      setupDoublePropertyEditor(bodyLengthTextField, bodyLengthValidImageView, YoCoordinateSystemFX3D::setBodyLength);
      setupDoublePropertyEditor(headLengthTextField, headLengthValidImageView, YoCoordinateSystemFX3D::setHeadLength);
      setupDoublePropertyEditor(bodyRadiusTextField, bodyRadiusValidImageView, YoCoordinateSystemFX3D::setBodyRadius);
      setupDoublePropertyEditor(headRadiusTextField, headRadiusValidImageView, YoCoordinateSystemFX3D::setHeadRadius);

      resetFields();
   }

   @Override
   public <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicCoordinateSystem3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      positionEditorController.setInput(definitionBeforeEdits.getPosition());
      orientationEditorController.setInput(definitionBeforeEdits.getOrientation());
      bodyLengthTextField.setText(definitionBeforeEdits.getBodyLength());
      headLengthTextField.setText(definitionBeforeEdits.getHeadLength());
      bodyRadiusTextField.setText(definitionBeforeEdits.getBodyRadius());
      headRadiusTextField.setText(definitionBeforeEdits.getHeadRadius());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicCoordinateSystem3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

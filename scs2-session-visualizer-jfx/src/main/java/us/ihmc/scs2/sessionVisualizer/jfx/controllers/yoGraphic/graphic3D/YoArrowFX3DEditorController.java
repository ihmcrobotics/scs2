package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import com.jfoenix.controls.JFXCheckBox;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoArrowFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;

public class YoArrowFX3DEditorController extends YoGraphicFX3DEditorController<YoArrowFX3D>
{
   @FXML
   private YoCompositeEditorPaneController originEditorController, directionEditorController;
   @FXML
   private TextField bodyLengthTextField, headLengthTextField;
   @FXML
   private TextField bodyRadiusTextField, headRadiusTextField;
   @FXML
   private JFXCheckBox scaleLengthCheckBox, scaleRadiusCheckBox;

   @FXML
   private ImageView bodyLengthValidImageView, headLengthValidImageView;
   @FXML
   private ImageView bodyRadiusValidImageView, headRadiusValidImageView;

   private YoGraphicArrow3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoArrowFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicArrow3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple3DPropertyEditor(originEditorController, "Origin", true, yoGraphicToEdit.getOrigin());
      setupTuple3DPropertyEditor(directionEditorController, "Direction", true, yoGraphicToEdit.getDirection());

      setupDoublePropertyEditor(bodyLengthTextField, bodyLengthValidImageView, YoArrowFX3D::setBodyLength);
      setupDoublePropertyEditor(headLengthTextField, headLengthValidImageView, YoArrowFX3D::setHeadLength);
      setupDoublePropertyEditor(bodyRadiusTextField, bodyRadiusValidImageView, YoArrowFX3D::setBodyRadius);
      setupDoublePropertyEditor(headRadiusTextField, headRadiusValidImageView, YoArrowFX3D::setHeadRadius);

      scaleLengthCheckBox.selectedProperty().addListener((observable, oldValue, newValue) -> yoGraphicToEdit.setScaleLength(newValue));
      scaleRadiusCheckBox.selectedProperty().addListener((observable, oldValue, newValue) -> yoGraphicToEdit.setScaleRadius(newValue));

      scaleLengthCheckBox.selectedProperty().addListener(this::updateHasChangesPendingProperty);
      scaleRadiusCheckBox.selectedProperty().addListener(this::updateHasChangesPendingProperty);

      resetFields();
   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicArrow3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      originEditorController.setInput(definitionBeforeEdits.getOrigin());
      directionEditorController.setInput(definitionBeforeEdits.getDirection());
      bodyLengthTextField.setText(definitionBeforeEdits.getBodyLength());
      headLengthTextField.setText(definitionBeforeEdits.getHeadLength());
      scaleLengthCheckBox.setSelected(definitionBeforeEdits.isScaleLength());
      bodyRadiusTextField.setText(definitionBeforeEdits.getBodyRadius());
      headRadiusTextField.setText(definitionBeforeEdits.getHeadRadius());
      scaleRadiusCheckBox.setSelected(definitionBeforeEdits.isScaleRadius());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicArrow3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

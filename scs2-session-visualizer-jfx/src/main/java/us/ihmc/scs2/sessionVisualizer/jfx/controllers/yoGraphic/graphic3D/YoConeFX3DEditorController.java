package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCone3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoConeFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;

public class YoConeFX3DEditorController extends YoGraphicFX3DEditorController<YoConeFX3D>
{
   @FXML
   private YoCompositeEditorPaneController positionEditorController, axisEditorController;
   @FXML
   private TextField heightTextField, radiusTextField;

   @FXML
   private ImageView heightValidImageView, radiusValidImageView;

   private YoGraphicCone3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoConeFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicCone3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple3DPropertyEditor(positionEditorController, "Position", true, yoGraphicToEdit.getPosition());
      setupTuple3DPropertyEditor(axisEditorController, "Axis", true, yoGraphicToEdit.getAxis());
      setupDoublePropertyEditor(heightTextField, heightValidImageView, YoConeFX3D::setHeight);
      setupDoublePropertyEditor(radiusTextField, radiusValidImageView, YoConeFX3D::setRadius);

      resetFields();
   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicCone3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      positionEditorController.setInput(definitionBeforeEdits.getPosition());
      axisEditorController.setInput(definitionBeforeEdits.getAxis());
      heightTextField.setText(definitionBeforeEdits.getHeight());
      radiusTextField.setText(definitionBeforeEdits.getRadius());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicCone3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

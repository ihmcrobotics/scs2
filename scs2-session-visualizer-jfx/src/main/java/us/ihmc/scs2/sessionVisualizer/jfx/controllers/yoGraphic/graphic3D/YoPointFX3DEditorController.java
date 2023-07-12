package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXResourceManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPointFX3D;

public class YoPointFX3DEditorController extends YoGraphicFX3DEditorController<YoPointFX3D>
{
   @FXML
   private YoCompositeEditorPaneController positionEditorController;
   @FXML
   private TextField sizeTextField;
   @FXML
   private ComboBox<String> graphicComboBox;
   @FXML
   private ImageView sizeValidImageView;

   private YoGraphicPoint3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoPointFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPoint3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple3DPropertyEditor(positionEditorController, "Position", true, yoGraphicToEdit.getPosition());
      setupDoublePropertyEditor(sizeTextField, sizeValidImageView, YoPointFX3D::setSize);

      YoGraphicFXResourceManager yoGraphicFXResourceManager = toolkit.getYoGraphicFXManager().getYoGraphicFXResourceManager();
      graphicComboBox.setItems(FXCollections.observableArrayList(yoGraphicFXResourceManager.getGraphic3DNameList()));

      graphicComboBox.valueProperty()
                     .addListener((o, oldValue, newValue) -> yoGraphicToEdit.setGraphicResource(yoGraphicFXResourceManager.loadGraphic3DResource(newValue)));

      graphicComboBox.valueProperty().addListener(this::updateHasChangesPendingProperty);

      resetFields();

   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicPoint3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      positionEditorController.setInput(definitionBeforeEdits.getPosition());
      sizeTextField.setText(definitionBeforeEdits.getSize());
      if (definitionBeforeEdits.getGraphicName() == null)
         definitionBeforeEdits.setGraphicName(YoGraphicFXResourceManager.DEFAULT_POINT3D_GRAPHIC_RESOURCE.getResourceName());
      graphicComboBox.setValue(definitionBeforeEdits.getGraphicName());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPoint3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

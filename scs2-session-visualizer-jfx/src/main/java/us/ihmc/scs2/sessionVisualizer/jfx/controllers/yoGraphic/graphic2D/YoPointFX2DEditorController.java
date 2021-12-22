package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic2D;

import com.jfoenix.controls.JFXComboBox;

import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXResourceManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPointFX2D;

public class YoPointFX2DEditorController extends YoGraphicFX2DEditorController<YoPointFX2D>
{
   @FXML
   private YoCompositeEditorPaneController positionEditorController;
   @FXML
   private TextField sizeTextField;
   @FXML
   private JFXComboBox<String> graphicComboBox;
   @FXML
   private ImageView sizeValidImageView;

   private YoGraphicPoint2DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoPointFX2D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      definitionBeforeEdits = YoGraphicTools.toYoGraphicPoint2DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple2DPropertyEditor(positionEditorController, "Position", true, yoGraphicToEdit.getPosition());
      setupDoublePropertyEditor(sizeTextField, sizeValidImageView, YoPointFX2D::setSize);

      YoGraphicFXResourceManager yoGraphicFXResourceManager = toolkit.getYoGraphicFXManager().getYoGraphicFXResourceManager();
      graphicComboBox.setItems(FXCollections.observableArrayList(yoGraphicFXResourceManager.getGraphic2DNameList()));
      graphicComboBox.valueProperty()
                     .addListener((o, oldValue, newValue) -> yoGraphicToEdit.setGraphicResource(yoGraphicFXResourceManager.loadGraphic2DResource(newValue)));
      graphicComboBox.valueProperty().addListener(this::updateHasChangesPendingProperty);

      resetFields();
   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicPoint2DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      positionEditorController.setInput(definitionBeforeEdits.getPosition());
      sizeTextField.setText(definitionBeforeEdits.getSize());
      styleEditorController.setInput(definitionBeforeEdits);
      if (definitionBeforeEdits.getGraphicName() == null)
         definitionBeforeEdits.setGraphicName(YoGraphicFXResourceManager.DEFAULT_POINT2D_GRAPHIC_RESOURCE.getResourceName());
      graphicComboBox.setValue(definitionBeforeEdits.getGraphicName());
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPoint2DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

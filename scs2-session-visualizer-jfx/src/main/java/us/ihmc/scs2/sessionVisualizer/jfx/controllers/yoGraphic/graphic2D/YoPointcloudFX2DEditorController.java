package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic2D;

import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Region;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPointcloud2DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeListEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXResourceManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPointcloudFX2D;

public class YoPointcloudFX2DEditorController extends YoGraphicFX2DEditorController<YoPointcloudFX2D>
{
   @FXML
   private YoCompositeListEditorPaneController pointListEditorController;
   @FXML
   private TextField sizeTextField;
   @FXML
   private ImageView sizeValidImageView;
   @FXML
   private ComboBox<String> graphicComboBox;

   private YoGraphicPointcloud2DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoPointcloudFX2D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPointcloud2DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupDoublePropertyEditor(sizeTextField, sizeValidImageView, YoPointcloudFX2D::setSize);
      setupTuple2DPropertyListEditor(pointListEditorController, "Point", true, yoGraphicToEdit::setNumberOfPoints, yoGraphicToEdit::setPoints);

      YoGraphicFXResourceManager yoGraphicFXResourceManager = toolkit.getYoGraphicFXManager().getYoGraphicFXResourceManager();
      graphicComboBox.setItems(FXCollections.observableArrayList(yoGraphicFXResourceManager.getGraphic2DNameList()));
      graphicComboBox.valueProperty()
                     .addListener((o, oldValue, newValue) -> yoGraphicToEdit.setGraphicResource(yoGraphicFXResourceManager.loadGraphic2DResource(newValue)));

      graphicComboBox.valueProperty().addListener(this::updateHasChangesPendingProperty);

      setupHeightAdjustment();
      resetFields();
   }

   private void setupHeightAdjustment()
   {
      mainPane.parentProperty().addListener((o, oldValue, newValue) ->
      {
         Region parent = (Region) newValue;

         while (parent != null && !(parent instanceof ScrollPane))
            parent = (Region) parent.getParent();

         if (parent == null)
            return;

         pointListEditorController.setupHeightAdjustmentForScrollPane((ScrollPane) parent);
      });
   }

   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicPointcloud2DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      pointListEditorController.setInputFromDefinition(definitionBeforeEdits.getPoints(), definitionBeforeEdits.getNumberOfPoints());
      styleEditorController.setInput(definitionBeforeEdits);
      if (definitionBeforeEdits.getGraphicName() == null)
         definitionBeforeEdits.setGraphicName(YoGraphicFXResourceManager.DEFAULT_POINT2D_GRAPHIC_RESOURCE.getResourceName());
      graphicComboBox.setValue(definitionBeforeEdits.getGraphicName());
      sizeTextField.setText(definitionBeforeEdits.getSize());
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPointcloud2DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

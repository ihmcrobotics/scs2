package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Region;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygonExtruded3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeListEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPolygonExtrudedFX3D;

public class YoPolygonExtrudedFX3DEditorController extends YoGraphicFX3DEditorController<YoPolygonExtrudedFX3D>
{
   @FXML
   private YoCompositeEditorPaneController positionEditorController, orientationEditorController;
   @FXML
   private YoCompositeListEditorPaneController vertexListEditorController;
   @FXML
   private TextField thicknessTextField;

   @FXML
   private ImageView thicknessValidImageView;

   private YoGraphicPolygonExtruded3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoPolygonExtrudedFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPolygonExtruded3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple3DPropertyEditor(positionEditorController, "Position", true, yoGraphicToEdit.getPosition());
      setupOrientation3DProperty(orientationEditorController, "Orientation", true, yoGraphicToEdit.getOrientation());
      setupTuple2DPropertyListEditor(vertexListEditorController, "Vertex", true, yoGraphicToEdit::setNumberOfVertices, yoGraphicToEdit::setVertices);
      setupDoublePropertyEditor(thicknessTextField, thicknessValidImageView, YoPolygonExtrudedFX3D::setThickness);

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

         vertexListEditorController.setupHeightAdjustmentForScrollPane((ScrollPane) parent);
      });
   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicPolygonExtruded3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      positionEditorController.setInput(definitionBeforeEdits.getPosition());
      orientationEditorController.setInput(definitionBeforeEdits.getOrientation());
      vertexListEditorController.setInputFromDefinition(definitionBeforeEdits.getVertices(), definitionBeforeEdits.getNumberOfVertices());
      thicknessTextField.setText(definitionBeforeEdits.getThickness());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPolygonExtruded3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

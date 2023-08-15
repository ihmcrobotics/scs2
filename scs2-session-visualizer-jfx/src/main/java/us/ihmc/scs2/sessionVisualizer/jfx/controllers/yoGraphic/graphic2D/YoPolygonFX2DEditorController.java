package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic2D;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.ScrollPane;
import javafx.scene.layout.Region;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygon2DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeListEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPolygonFX2D;

public class YoPolygonFX2DEditorController extends YoGraphicFX2DEditorController<YoPolygonFX2D>
{
   @FXML
   private YoCompositeListEditorPaneController vertexListEditorController;

   private YoGraphicPolygon2DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoPolygonFX2D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      definitionBeforeEdits = YoGraphicTools.toYoGraphicPolygon2DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple2DPropertyListEditor(vertexListEditorController,
                                     "Vertex",
                                     "Vertices",
                                     true,
                                     yoGraphicToEdit::setNumberOfVertices,
                                     yoGraphicToEdit::setVertices);

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
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicPolygon2DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      vertexListEditorController.setInputFromDefinition(definitionBeforeEdits.getVertices(), definitionBeforeEdits.getNumberOfVertices());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPolygon2DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

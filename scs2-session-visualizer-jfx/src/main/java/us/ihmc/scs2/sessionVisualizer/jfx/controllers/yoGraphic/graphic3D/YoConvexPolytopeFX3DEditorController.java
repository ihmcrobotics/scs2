package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.ScrollPane;
import javafx.scene.layout.Region;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicConvexPolytope3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeListEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoConvexPolytopeFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;

public class YoConvexPolytopeFX3DEditorController extends YoGraphicFX3DEditorController<YoConvexPolytopeFX3D>
{
   @FXML
   private YoCompositeEditorPaneController positionEditorController, orientationEditorController;
   @FXML
   private YoCompositeListEditorPaneController vertexListEditorController;

   private YoGraphicConvexPolytope3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoConvexPolytopeFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicConvexPolytope3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupTuple3DPropertyEditor(positionEditorController, "Position", true, yoGraphicToEdit.getPosition());
      setupOrientation3DProperty(orientationEditorController, "Orientation", true, yoGraphicToEdit.getOrientation());
      setupTuple3DPropertyListEditor(vertexListEditorController, "Vertex", true, yoGraphicToEdit::setNumberOfVertices, yoGraphicToEdit::setVertices);

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
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicConvexPolytope3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      positionEditorController.setInput(definitionBeforeEdits.getPosition());
      orientationEditorController.setInput(definitionBeforeEdits.getOrientation());
      vertexListEditorController.setInputFromDefinition(definitionBeforeEdits.getVertices(), definitionBeforeEdits.getNumberOfVertices());
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicConvexPolytope3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

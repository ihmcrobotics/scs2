package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic2D;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.RadioButton;
import javafx.scene.control.ToggleGroup;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicLine2DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoLineFX2D;

public class YoLineFX2DEditorController extends YoGraphicFX2DEditorController<YoLineFX2D>
{
   @FXML
   private YoCompositeEditorPaneController originEditorController, directionEditorController, destinationEditorController;
   @FXML
   private RadioButton directionRadioButton, destinationRadioButton;

   private Tuple2DProperty directionProperty = new Tuple2DProperty(null, 0, 1);
   private Tuple2DProperty destinationProperty = new Tuple2DProperty(null, 0, 0);
   private YoGraphicLine2DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoLineFX2D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicLine2DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      new ToggleGroup().getToggles().addAll(directionRadioButton, destinationRadioButton);
      directionEditorController.getMainPane().disableProperty().bind(destinationRadioButton.selectedProperty());
      destinationEditorController.getMainPane().disableProperty().bind(directionRadioButton.selectedProperty());

      if (yoGraphicToEdit.getDirection() != null)
         directionProperty = new Tuple2DProperty(yoGraphicToEdit.getDirection());
      else
         directionProperty = new Tuple2DProperty(toolkit.getReferenceFrameManager().getWorldFrame(), 0, 1);
      if (yoGraphicToEdit.getDestination() != null)
         destinationProperty = new Tuple2DProperty(yoGraphicToEdit.getDestination());
      else
         destinationProperty = new Tuple2DProperty(toolkit.getReferenceFrameManager().getWorldFrame(), 0, 1);

      directionRadioButton.selectedProperty().addListener((o, oldValue, newValue) -> updateSelection(newValue, yoGraphicToEdit));

      setupTuple2DPropertyEditor(originEditorController, "Origin", true, yoGraphicToEdit.getOrigin());
      setupTuple2DPropertyEditor(directionEditorController, "Direction", true, directionProperty);
      setupTuple2DPropertyEditor(destinationEditorController, "Desintation", true, destinationProperty);

      updateSelection(directionRadioButton.isSelected(), yoGraphicToEdit);
      resetFields();
   }

   private void updateSelection(boolean isDirectionSelected, YoLineFX2D yoGraphicToEdit)
   {
      if (isDirectionSelected)
      {
         yoGraphicToEdit.setDirection(directionProperty);
         yoGraphicToEdit.setDestination(null);
      }
      else
      {
         yoGraphicToEdit.setDirection(null);
         yoGraphicToEdit.setDestination(destinationProperty);
      }
   }

   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicLine2DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      originEditorController.setInput(definitionBeforeEdits.getOrigin());
      if (definitionBeforeEdits.getDirection() != null)
      {
         directionEditorController.setInput(definitionBeforeEdits.getDirection());
         directionRadioButton.setSelected(true);
      }
      else
      {
         destinationEditorController.setInput(definitionBeforeEdits.getDestination());
         destinationRadioButton.setSelected(true);
      }
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicLine2DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

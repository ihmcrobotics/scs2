package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import com.jfoenix.controls.JFXTextField;

import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolynomial3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeListEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPolynomialFX3D;

public class YoPolynomialFX3DEditorController extends YoGraphicFX3DEditorController<YoPolynomialFX3D>
{
   @FXML
   private YoCompositeListEditorPaneController coefficientsXListEditorController, coefficientsYListEditorController, coefficientsZListEditorController;
   @FXML
   private JFXTextField startTimeTextField, endTimeTextField;
   @FXML
   private JFXTextField sizeTextField;
   @FXML
   private ImageView startTimeValidImageView, endTimeValidImageView;
   @FXML
   private ImageView sizeValidImageView;

   private YoGraphicPolynomial3DDefinition definitionBeforeEdits;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoPolynomialFX3D yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPolynomial3DDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      setupDoublePropertyListEditor(coefficientsXListEditorController,
                                    "Coefficent X",
                                    "Coefficients X",
                                    yoGraphicToEdit::setNumberOfCoefficientsX,
                                    yoGraphicToEdit::setCoefficientsX);
      setupDoublePropertyListEditor(coefficientsYListEditorController,
                                    "Coefficent Y",
                                    "Coefficients Y",
                                    yoGraphicToEdit::setNumberOfCoefficientsY,
                                    yoGraphicToEdit::setCoefficientsY);
      setupDoublePropertyListEditor(coefficientsZListEditorController,
                                    "Coefficent Z",
                                    "Coefficients Z",
                                    yoGraphicToEdit::setNumberOfCoefficientsZ,
                                    yoGraphicToEdit::setCoefficientsZ);
      setupDoublePropertyEditor(startTimeTextField, startTimeValidImageView, YoPolynomialFX3D::setStartTime);
      setupDoublePropertyEditor(endTimeTextField, endTimeValidImageView, YoPolynomialFX3D::setEndTime);
      setupDoublePropertyEditor(sizeTextField, sizeValidImageView, YoPolynomialFX3D::setSize);

      coefficientsXListEditorController.setPrefHeight(4);
      coefficientsYListEditorController.setPrefHeight(4);
      coefficientsZListEditorController.setPrefHeight(4);
      resetFields();
   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicPolynomial3DDefinition(yoGraphicToEdit)));
   }

   @Override
   public void resetFields()
   {
      coefficientsXListEditorController.setInputSingletonComposites(definitionBeforeEdits.getCoefficientsX(), definitionBeforeEdits.getNumberOfCoefficientsX());
      coefficientsYListEditorController.setInputSingletonComposites(definitionBeforeEdits.getCoefficientsY(), definitionBeforeEdits.getNumberOfCoefficientsY());
      coefficientsZListEditorController.setInputSingletonComposites(definitionBeforeEdits.getCoefficientsZ(), definitionBeforeEdits.getNumberOfCoefficientsZ());
      styleEditorController.setInput(definitionBeforeEdits);
      startTimeTextField.setText(definitionBeforeEdits.getStartTime());
      endTimeTextField.setText(definitionBeforeEdits.getEndTime());
      sizeTextField.setText(definitionBeforeEdits.getSize());
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicPolynomial3DDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }
}

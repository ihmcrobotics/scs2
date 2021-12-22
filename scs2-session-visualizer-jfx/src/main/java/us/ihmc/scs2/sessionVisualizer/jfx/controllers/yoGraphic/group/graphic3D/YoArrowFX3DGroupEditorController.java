package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.graphic3D;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools.setField;

import com.jfoenix.controls.JFXCheckBox;

import javafx.beans.binding.Bindings;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.VBox;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoGraphic3DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoGraphicNameEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField.YoDoubleTextField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoArrowFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

public class YoArrowFX3DGroupEditorController extends YoGroupFXEditorController<YoArrowFX3D>
{
   @FXML
   private VBox mainPane;
   @FXML
   private TextField bodyLengthTextField, headLengthTextField;
   @FXML
   private TextField bodyRadiusTextField, headRadiusTextField;
   @FXML
   private JFXCheckBox scaleLengthCheckBox, scaleRadiusCheckBox;
   @FXML
   private YoGraphic3DStyleEditorPaneController styleEditorController;
   @FXML
   private YoGraphicNameEditorPaneController nameEditorController;

   @FXML
   private ImageView bodyLengthValidImageView, headLengthValidImageView;
   @FXML
   private ImageView bodyRadiusValidImageView, headRadiusValidImageView;

   private YoDoubleTextField yoBodyLengthTextField;
   private YoDoubleTextField yoBodyRadiusTextField;
   private YoDoubleTextField yoHeadLengthTextField;
   private YoDoubleTextField yoHeadRadiusTextField;
   private ObservableBooleanValue inputsValidityProperty;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGroupFX yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      yoBodyLengthTextField = setupDoublePropertyEditor(bodyLengthTextField, bodyLengthValidImageView, YoArrowFX3D::setBodyLength, YoArrowFX3D::getBodyLength);
      yoHeadLengthTextField = setupDoublePropertyEditor(headLengthTextField, headLengthValidImageView, YoArrowFX3D::setHeadLength, YoArrowFX3D::getHeadLength);
      yoBodyRadiusTextField = setupDoublePropertyEditor(bodyRadiusTextField, bodyRadiusValidImageView, YoArrowFX3D::setBodyRadius, YoArrowFX3D::getBodyRadius);
      yoHeadRadiusTextField = setupDoublePropertyEditor(headRadiusTextField, headRadiusValidImageView, YoArrowFX3D::setHeadRadius, YoArrowFX3D::getHeadRadius);

      registerResetAction(() ->
      {
         Boolean initialScaleLength = YoGroupFXEditorTools.getCommonValue(YoGroupFXEditorTools.getField(graphicChildren, YoArrowFX3D::getScaleLength));
         Boolean initialScaleRadius = YoGroupFXEditorTools.getCommonValue(YoGroupFXEditorTools.getField(graphicChildren, YoArrowFX3D::getScaleRadius));

         if (initialScaleLength != null)
            scaleLengthCheckBox.selectedProperty().set(initialScaleLength);
         if (initialScaleRadius != null)
            scaleRadiusCheckBox.selectedProperty().set(initialScaleRadius);
      });

      setupStyleEditor(styleEditorController);
      setupNameEditor(nameEditorController);

      inputsValidityProperty = Bindings.and(yoBodyLengthTextField.getValidityProperty(), yoBodyRadiusTextField.getValidityProperty())
                                       .and(yoHeadLengthTextField.getValidityProperty()).and(yoHeadRadiusTextField.getValidityProperty())
                                       .and(styleEditorController.inputsValidityProperty()).and(nameEditorController.inputsValidityProperty());

      scaleLengthCheckBox.selectedProperty().addListener((observable, oldValue, newValue) -> setField(graphicChildren, YoArrowFX3D::setScaleLength, newValue));
      scaleRadiusCheckBox.selectedProperty().addListener((observable, oldValue, newValue) -> setField(graphicChildren, YoArrowFX3D::setScaleRadius, newValue));

      resetFields();
   }

   @Override
   public ObservableBooleanValue inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   @Override
   public VBox getMainPane()
   {
      return mainPane;
   }

   @Override
   public Class<YoArrowFX3D> getChildrenCommonType()
   {
      return YoArrowFX3D.class;
   }
}

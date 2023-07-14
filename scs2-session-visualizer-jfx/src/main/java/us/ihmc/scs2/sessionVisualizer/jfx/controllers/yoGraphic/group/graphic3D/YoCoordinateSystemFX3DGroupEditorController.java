package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.graphic3D;

import javafx.beans.binding.Bindings;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.VBox;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.DoubleSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoGraphic3DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoGraphicNameEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoCoordinateSystemFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

public class YoCoordinateSystemFX3DGroupEditorController extends YoGroupFXEditorController<YoCoordinateSystemFX3D>
{
   @FXML
   private VBox mainPane;
   @FXML
   private TextField bodyLengthTextField, headLengthTextField;
   @FXML
   private TextField bodyRadiusTextField, headRadiusTextField;
   @FXML
   private YoGraphic3DStyleEditorPaneController styleEditorController;
   @FXML
   private YoGraphicNameEditorPaneController nameEditorController;

   @FXML
   private ImageView bodyLengthValidImageView, headLengthValidImageView;
   @FXML
   private ImageView bodyRadiusValidImageView, headRadiusValidImageView;

   private DoubleSearchField yoBodyLengthTextField;
   private DoubleSearchField yoBodyRadiusTextField;
   private DoubleSearchField yoHeadLengthTextField;
   private DoubleSearchField yoHeadRadiusTextField;
   private ObservableBooleanValue inputsValidityProperty;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGroupFX yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      yoBodyLengthTextField = setupDoublePropertyEditor(bodyLengthTextField,
                                                        bodyLengthValidImageView,
                                                        YoCoordinateSystemFX3D::setBodyLength,
                                                        YoCoordinateSystemFX3D::getBodyLength);
      yoHeadLengthTextField = setupDoublePropertyEditor(headLengthTextField,
                                                        headLengthValidImageView,
                                                        YoCoordinateSystemFX3D::setHeadLength,
                                                        YoCoordinateSystemFX3D::getHeadLength);
      yoBodyRadiusTextField = setupDoublePropertyEditor(bodyRadiusTextField,
                                                        bodyRadiusValidImageView,
                                                        YoCoordinateSystemFX3D::setBodyRadius,
                                                        YoCoordinateSystemFX3D::getBodyRadius);
      yoHeadRadiusTextField = setupDoublePropertyEditor(headRadiusTextField,
                                                        headRadiusValidImageView,
                                                        YoCoordinateSystemFX3D::setHeadRadius,
                                                        YoCoordinateSystemFX3D::getHeadRadius);

      setupStyleEditor(styleEditorController);
      setupNameEditor(nameEditorController);

      inputsValidityProperty = Bindings.and(yoBodyLengthTextField.getValidityProperty(), yoBodyRadiusTextField.getValidityProperty())
                                       .and(yoHeadLengthTextField.getValidityProperty()).and(yoHeadRadiusTextField.getValidityProperty())
                                       .and(styleEditorController.inputsValidityProperty()).and(nameEditorController.inputsValidityProperty());

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
   public Class<YoCoordinateSystemFX3D> getChildrenCommonType()
   {
      return YoCoordinateSystemFX3D.class;
   }
}

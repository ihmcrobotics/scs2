package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.graphic3D;

import javafx.beans.binding.Bindings;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.VBox;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphic3DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphicNameEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.DoubleSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoCapsuleFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

public class YoCapsuleFX3DGroupEditorController extends YoGroupFXEditorController<YoCapsuleFX3D>
{
   @FXML
   private VBox mainPane;
   @FXML
   private TextField lengthTextField, radiusTextField;
   @FXML
   private YoGraphic3DStyleEditorPaneController styleEditorController;
   @FXML
   private YoGraphicNameEditorPaneController nameEditorController;

   @FXML
   private ImageView lengthValidImageView, radiusValidImageView;

   private DoubleSearchField yoLengthTextField;
   private DoubleSearchField yoRadiusTextField;
   private ObservableBooleanValue inputsValidityProperty;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGroupFX yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      yoLengthTextField = setupDoublePropertyEditor(lengthTextField, lengthValidImageView, YoCapsuleFX3D::setLength, YoCapsuleFX3D::getLength);
      yoRadiusTextField = setupDoublePropertyEditor(radiusTextField, radiusValidImageView, YoCapsuleFX3D::setRadius, YoCapsuleFX3D::getRadius);

      setupStyleEditor(styleEditorController);
      setupNameEditor(nameEditorController);

      inputsValidityProperty = Bindings.and(yoLengthTextField.getValidityProperty(), yoRadiusTextField.getValidityProperty())
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
   public Class<YoCapsuleFX3D> getChildrenCommonType()
   {
      return YoCapsuleFX3D.class;
   }
}

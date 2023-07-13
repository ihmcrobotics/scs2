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
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPolynomialFX3D;

public class YoPolynomialFX3DGroupEditorController extends YoGroupFXEditorController<YoPolynomialFX3D>
{
   @FXML
   private VBox mainPane;
   @FXML
   private TextField sizeTextField;
   @FXML
   private ImageView sizeValidImageView;
   @FXML
   private YoGraphic3DStyleEditorPaneController styleEditorController;
   @FXML
   private YoGraphicNameEditorPaneController nameEditorController;

   private ObservableBooleanValue inputsValidityProperty;

   private DoubleSearchField yoSizeTextField;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGroupFX yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      yoSizeTextField = setupDoublePropertyEditor(sizeTextField, sizeValidImageView, YoPolynomialFX3D::setSize, YoPolynomialFX3D::getSize);

      setupStyleEditor(styleEditorController);
      setupNameEditor(nameEditorController);

      inputsValidityProperty = Bindings.and(yoSizeTextField.getValidityProperty(), nameEditorController.inputsValidityProperty());

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
   public Class<YoPolynomialFX3D> getChildrenCommonType()
   {
      return YoPolynomialFX3D.class;
   }
}

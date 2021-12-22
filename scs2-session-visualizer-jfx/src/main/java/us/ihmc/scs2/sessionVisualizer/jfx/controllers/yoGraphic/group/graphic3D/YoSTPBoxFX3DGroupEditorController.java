package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.graphic3D;

import javafx.beans.binding.Bindings;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.VBox;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoGraphic3DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoGraphicNameEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField.YoDoubleTextField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoSTPBoxFX3D;

public class YoSTPBoxFX3DGroupEditorController extends YoGroupFXEditorController<YoSTPBoxFX3D>
{
   @FXML
   private VBox mainPane;
   @FXML
   private YoCompositeEditorPaneController sizeEditorController;
   @FXML
   private TextField minimumMarginTextField, maximumMarginTextField;
   @FXML
   private YoGraphic3DStyleEditorPaneController styleEditorController;
   @FXML
   private YoGraphicNameEditorPaneController nameEditorController;
   @FXML
   private ImageView minimumMarginValidImageView, maximumMarginValidImageView;

   private YoDoubleTextField yoMinimumMarginTextField;
   private YoDoubleTextField yoMaximumMarginTextField;
   private ObservableBooleanValue inputsValidityProperty;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGroupFX yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      setupTuple3DEditor(sizeEditorController, false, "Size", YoSTPBoxFX3D::setSize, YoSTPBoxFX3D::getSize);

      yoMinimumMarginTextField = setupDoublePropertyEditor(minimumMarginTextField,
                                                           minimumMarginValidImageView,
                                                           YoSTPBoxFX3D::setMinimumMargin,
                                                           YoSTPBoxFX3D::getMinimumMargin);
      yoMaximumMarginTextField = setupDoublePropertyEditor(maximumMarginTextField,
                                                           maximumMarginValidImageView,
                                                           YoSTPBoxFX3D::setMaximumMargin,
                                                           YoSTPBoxFX3D::getMaximumMargin);

      setupStyleEditor(styleEditorController);
      setupNameEditor(nameEditorController);

      inputsValidityProperty = Bindings.and(sizeEditorController.inputsValidityProperty(), yoMinimumMarginTextField.getValidityProperty())
                                       .and(yoMaximumMarginTextField.getValidityProperty()).and(nameEditorController.inputsValidityProperty());

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
   public Class<YoSTPBoxFX3D> getChildrenCommonType()
   {
      return YoSTPBoxFX3D.class;
   }
}

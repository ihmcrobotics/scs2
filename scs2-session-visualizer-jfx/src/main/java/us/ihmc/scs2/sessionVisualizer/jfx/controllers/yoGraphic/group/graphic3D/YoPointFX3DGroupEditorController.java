package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.graphic3D;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools.getCommonValue;
import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools.getField;
import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools.setField;

import javafx.beans.property.BooleanProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.VBox;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphic3DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphicNameEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.DoubleSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXResource;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXResourceManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPointFX3D;

public class YoPointFX3DGroupEditorController extends YoGroupFXEditorController<YoPointFX3D>
{
   @FXML
   private VBox mainPane;
   @FXML
   private TextField sizeTextField;
   @FXML
   private ComboBox<String> graphicComboBox;
   @FXML
   private YoGraphicNameEditorPaneController nameEditorController;
   @FXML
   private ImageView sizeValidImageView;
   @FXML
   private YoGraphic3DStyleEditorPaneController styleEditorController;

   private ObservableBooleanValue inputsValidityProperty;

   private DoubleSearchField yoSizeTextField;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGroupFX yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);

      yoSizeTextField = setupDoublePropertyEditor(sizeTextField, sizeValidImageView, YoPointFX3D::setSize, YoPointFX3D::getSize);

      YoGraphicFXResourceManager yoGraphicFXResourceManager = toolkit.getYoGraphicFXManager().getYoGraphicFXResourceManager();
      graphicComboBox.setItems(FXCollections.observableArrayList(yoGraphicFXResourceManager.getGraphic3DNameList()));

      registerResetAction(() ->
      {
         YoGraphicFXResource initialResource = getCommonValue(getField(graphicChildren, YoPointFX3D::getGraphicResource));
         if (initialResource != null)
            graphicComboBox.getSelectionModel().select(initialResource.getResourceName());
         else
            graphicComboBox.getSelectionModel().select(null);
      });

      setupStyleEditor(styleEditorController);
      setupNameEditor(nameEditorController);

      BooleanProperty radiusValidityProperty = yoSizeTextField.getValidityProperty();
      inputsValidityProperty = radiusValidityProperty.and(nameEditorController.inputsValidityProperty());

      graphicComboBox.valueProperty()
                     .addListener((o, oldValue, newValue) -> setField(graphicChildren,
                                                                      YoPointFX3D::setGraphicResource,
                                                                      yoGraphicFXResourceManager.loadGraphic3DResource(newValue)));

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
   public Class<YoPointFX3D> getChildrenCommonType()
   {
      return YoPointFX3D.class;
   }
}

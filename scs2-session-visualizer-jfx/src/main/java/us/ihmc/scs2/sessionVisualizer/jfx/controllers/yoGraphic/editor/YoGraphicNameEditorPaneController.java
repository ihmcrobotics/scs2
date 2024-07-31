package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ReadOnlyBooleanProperty;
import javafx.beans.property.ReadOnlyStringProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.GridPane;
import org.controlsfx.control.textfield.TextFields;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

public class YoGraphicNameEditorPaneController
{
   @FXML
   private GridPane mainPane;
   @FXML
   private Label nameLabel;
   @FXML
   private TextField nameTextField;
   @FXML
   private Label namespaceLabel;
   @FXML
   private TextField namespaceTextField;

   @FXML
   private ImageView nameValidImageView;

   private YoGroupFX rootGroup;

   private final BooleanProperty inputsValidityProperty = new SimpleBooleanProperty(this, "inputsValidity", false);

   private final StringProperty nameProperty = new SimpleStringProperty(this, "name", null);
   private final StringProperty namespaceProperty = new SimpleStringProperty(this, "namespace", null);

   private YoGraphicFXItem yoGraphicFXItem;

   public void initialize(SessionVisualizerToolkit toolkit, YoGraphicFXItem yoGraphicFXItem)
   {
      this.yoGraphicFXItem = yoGraphicFXItem;
      rootGroup = toolkit.getYoGraphicFXRootGroup();

      TextFields.bindAutoCompletion(namespaceTextField, YoGraphicTools.collectAllExistingNamespaces(rootGroup));
      YoGraphicFXControllerTools.bindValidityImageView(inputsValidityProperty, nameValidImageView);

      nameTextField.textProperty().addListener((o, oldValue, newValue) ->
                                               {
                                                  if (newValue == null || newValue.isEmpty())
                                                  {
                                                     inputsValidityProperty.set(false);
                                                  }
                                                  else
                                                  {
                                                     YoGraphicFXItem searchResult = findYoGraphicFXItem(newValue, namespaceTextField.getText());
                                                     boolean isNameValid = searchResult == null || searchResult == yoGraphicFXItem;
                                                     inputsValidityProperty.set(isNameValid);
                                                     if (isNameValid)
                                                        nameProperty.set(newValue);
                                                  }
                                               });

      namespaceTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                    {
                                                       if (newValue == null || newValue.isEmpty())
                                                       {
                                                          inputsValidityProperty.set(false);
                                                       }
                                                       else
                                                       {
                                                          YoGraphicFXItem searchResult = findYoGraphicFXItem(nameTextField.getText(), newValue);
                                                          boolean isNamespaceValid = searchResult == null || searchResult == yoGraphicFXItem;
                                                          inputsValidityProperty.set(isNamespaceValid);
                                                          if (isNamespaceValid)
                                                             namespaceProperty.set(newValue);
                                                       }
                                                    });
   }

   private YoGraphicFXItem findYoGraphicFXItem(String itemName, String namespace)
   {
      return YoGraphicTools.findYoGraphicFXItem(rootGroup, namespace, itemName, yoGraphicFXItem.getClass());
   }

   public void setInput(String name, String namespace)
   {
      nameTextField.setText(name);
      namespaceTextField.setText(namespace);
      namespaceTextField.setDisable(true);
   }

   public void bindYoGraphicFXItem(YoGraphicFXItem yoGraphicFXItemToBind)
   {
      nameProperty.addListener((o, oldValue, newValue) -> yoGraphicFXItemToBind.setName(newValue));
      // TODO figure out the namespace, if needed.
   }

   public void addAnyChangeListener(ChangeListener<Object> changeListener)
   {
      nameProperty.addListener(changeListener);
      namespaceProperty.addListener(changeListener);
   }

   public ReadOnlyBooleanProperty inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   public ReadOnlyStringProperty nameProperty()
   {
      return nameProperty;
   }

   public ReadOnlyStringProperty namespaceProperty()
   {
      return namespaceProperty;
   }

   public GridPane getMainPane()
   {
      return mainPane;
   }

   public Label getNameLabel()
   {
      return nameLabel;
   }

   public Label getNamespaceLabel()
   {
      return namespaceLabel;
   }

   public TextField getNameTextField()
   {
      return nameTextField;
   }
}

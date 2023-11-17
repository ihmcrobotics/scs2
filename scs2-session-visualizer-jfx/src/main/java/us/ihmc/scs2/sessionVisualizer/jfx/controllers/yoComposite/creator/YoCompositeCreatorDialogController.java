package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator;

import com.jfoenix.controls.JFXComboBox;
import com.jfoenix.controls.JFXTextField;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.fxml.FXML;
import javafx.scene.control.ButtonType;
import javafx.scene.control.Dialog;
import javafx.scene.control.DialogPane;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXCreatorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Optional;

public class YoCompositeCreatorDialogController
{
   @FXML
   private DialogPane dialogPane;
   @FXML
   public JFXTextField yoCompositeNameTextField;
   @FXML
   public ImageView yoCompositeNameValidImageView;
   @FXML
   public JFXComboBox<YoCompositeType> yoCompositeTypeComboBox;

   public enum YoCompositeType
   {Double, Integer}

   public YoComposite showAndWait(YoRegistry userRegistry)
   {
      yoCompositeTypeComboBox.getItems().addAll(YoCompositeType.values());
      yoCompositeTypeComboBox.getSelectionModel().selectFirst();

      BooleanProperty validityProperty = new SimpleBooleanProperty(this, "validity", false);
      yoCompositeNameTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                          {
                                                             if (newValue == null || newValue.isEmpty())
                                                             {
                                                                validityProperty.set(false);
                                                                return;
                                                             }

                                                             validityProperty.set(!userRegistry.hasVariable(newValue));
                                                          });

      YoGraphicFXControllerTools.bindValidityImageView(validityProperty, yoCompositeNameValidImageView);
      dialogPane.lookupButton(ButtonType.OK).disableProperty().bind(validityProperty.not());

      Dialog<ButtonType> dialog = new Dialog<>();
      dialog.dialogPaneProperty().set(dialogPane);
      dialog.setTitle("Create YoComposite");
      Optional<ButtonType> result = dialog.showAndWait();

      if (result.isPresent() && result.get() == ButtonType.OK)
      {
         YoVariable yoVariable = switch (yoCompositeTypeComboBox.getValue())
         {
            case Double -> new YoDouble(yoCompositeNameTextField.getText(), userRegistry);
            case Integer -> new YoInteger(yoCompositeNameTextField.getText(), userRegistry);
         };
         return new YoComposite(YoCompositeSearchManager.yoVariablePattern, yoVariable);
      }
      else
      {
         return null;
      }
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoFilter;

import com.jfoenix.controls.JFXComboBox;
import com.jfoenix.controls.JFXTextField;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.fxml.FXML;
import javafx.scene.control.ButtonType;
import javafx.scene.control.Dialog;
import javafx.scene.control.DialogPane;
import javafx.scene.image.ImageView;
import javafx.stage.Window;
import us.ihmc.scs2.filtering.AlphaFilteredVariable;
import us.ihmc.scs2.filtering.YoFilter;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Optional;

/**
 * This class is based off of {@link us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator.YoCompositeCreatorDialogController}
 */
public class YoFilterCreatorDialogController
{
   @FXML
   private DialogPane dialogPane;
   @FXML
   public JFXTextField yoFilterNameTextField;
   @FXML
   public ImageView yoFilterNameValidImageView;
   public JFXComboBox<YoFilterType> yoFilterTypeComboBox;

   public enum YoFilterType
   {Alpha}

   public YoFilter showAndWait(Window owner, YoRegistry filterRegistry, YoSharedBuffer sharedBuffer)
   {
      yoFilterTypeComboBox.getItems().addAll(YoFilterType.values());
      yoFilterTypeComboBox.getSelectionModel().selectFirst();

      BooleanProperty validityProperty = new SimpleBooleanProperty(this, "validity", false);
      yoFilterNameTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                          {
                                                             if (newValue == null || newValue.isEmpty())
                                                             {
                                                                validityProperty.set(false);
                                                                return;
                                                             }

                                                             validityProperty.set(!filterRegistry.hasVariable(newValue));
                                                          });

      YoGraphicFXControllerTools.bindValidityImageView(validityProperty, yoFilterNameValidImageView);
      dialogPane.lookupButton(ButtonType.OK).disableProperty().bind(validityProperty.not());

      Dialog<ButtonType> dialog = new Dialog<>();
      dialog.initOwner(owner);
      dialog.dialogPaneProperty().set(dialogPane);
      dialog.setTitle("Create YoFilter");
      dialog.setOnShowing(e -> yoFilterNameTextField.requestFocus());
      SessionVisualizerIOTools.addSCSIconToDialog(dialog);
      JavaFXMissingTools.centerDialogInOwner(dialog);
      Optional<ButtonType> result = dialog.showAndWait();

      if (result.isPresent() && result.get() == ButtonType.OK)
      {
         YoFilter filter = switch (yoFilterTypeComboBox.getValue())
         {
            case Alpha -> new AlphaFilteredVariable(yoFilterNameTextField.getText(), sharedBuffer.getRegistryBuffer());
         };
         return filter;
      }
      else
      {
         return null;
      }
   }
}

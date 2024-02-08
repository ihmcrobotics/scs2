package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.TransferMode;
import javafx.util.Callback;
import org.controlsfx.control.textfield.AutoCompletionBinding;
import org.controlsfx.control.textfield.AutoCompletionBinding.ISuggestionRequest;
import org.controlsfx.control.textfield.TextFields;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;

import java.util.Collection;
import java.util.List;

/**
 * Wrapper around a simple {@link TextField} to add search functionality, auto-completion,
 * validation.
 *
 * @param <T>
 * @author Sylvain Bertrand
 */
public abstract class PropertySearchField<T extends Property<?>>
{
   private final TextField textField;
   private final BooleanProperty validityProperty = new SimpleBooleanProperty(this, "searchFieldEntryValidity", false);
   private final ObjectProperty<T> supplierProperty = new SimpleObjectProperty<>(this, "supplier", null);

   public PropertySearchField(TextField textField, ImageView validImageView)
   {
      this.textField = textField;

      if (validImageView != null)
         attachValidImageView(validImageView);

      textField.textProperty().addListener((observable, oldValue, newValue) ->
                                           {
                                              if (newValue != null && newValue.equals(oldValue))
                                                 return;

                                              String simplifiedText = simplifyText(newValue);
                                              if (simplifiedText != null)
                                              {
                                                 textField.setText(simplifiedText);
                                                 return;
                                              }
                                              boolean isTextValid = isTextValid(newValue);

                                              validityProperty.set(isTextValid);

                                              if (isTextValid)
                                                 supplierProperty.set(toSupplier(newValue));
                                           });

      textField.setOnDragEntered(this::handleDragEntered);
      textField.setOnDragExited(this::handleDragExited);
      textField.setOnDragOver(this::handleDragOver);
      textField.setOnDragDropped(this::handleDragDropped);
   }

   protected abstract boolean isTextValid(String text);

   protected abstract String simplifyText(String text);

   protected abstract Callback<ISuggestionRequest, Collection<String>> createSuggestions();

   public void setupAutoCompletion()
   {
      AutoCompletionBinding<String> autoCompletionBinding = TextFields.bindAutoCompletion(textField, createSuggestions());

      autoCompletionBinding.prefWidthProperty().bind(textField.widthProperty());
      textField.addEventHandler(KeyEvent.KEY_PRESSED, e ->
      {
         if (textField.isFocused() && e.isControlDown() && e.getCode() == KeyCode.SPACE)
         {
            autoCompletionBinding.setUserInput(textField.getText());
            e.consume();
         }
      });
   }

   public void enable()
   {

   }

   public void disable()
   {

   }

   public void attachValidImageView(ImageView imageView)
   {
      attachValidImageView(imageView, SessionVisualizerIOTools.VALID_ICON_IMAGE, SessionVisualizerIOTools.INVALID_ICON_IMAGE);
   }

   public void attachValidImageView(ImageView imageView, Image validIcon, Image invalidIcon)
   {
      YoGraphicFXControllerTools.bindBooleanToImageView(validityProperty, imageView, validIcon, invalidIcon);
   }

   public BooleanProperty getValidityProperty()
   {
      return validityProperty;
   }

   protected abstract T toSupplier(String text);

   public T getSupplier()
   {
      if (!validityProperty.get())
         return null;
      return supplierProperty.get();
   }

   public ObjectProperty<T> supplierProperty()
   {
      return supplierProperty;
   }

   private void handleDragEntered(DragEvent dragEvent)
   {
      if (acceptDragEventForDrop(dragEvent))
         dragEvent.consume();
   }

   private void handleDragExited(DragEvent event)
   {
      if (acceptDragEventForDrop(event))
         event.consume();
   }

   private void handleDragOver(DragEvent event)
   {
      if (acceptDragEventForDrop(event))
         event.acceptTransferModes(TransferMode.ANY);
      event.consume();
   }

   protected abstract List<YoComposite> retrieveYoCompositesFromDragboard(Dragboard dragboard);

   private void handleDragDropped(DragEvent event)
   {
      Dragboard dragboard = event.getDragboard();
      boolean success = false;
      List<YoComposite> yoComposites = retrieveYoCompositesFromDragboard(dragboard);
      if (yoComposites != null)
      {
         success = true;
         textField.setText(yoComposites.get(0).getUniqueName());
      }
      event.setDropCompleted(success);
      event.consume();
   }

   private boolean acceptDragEventForDrop(DragEvent event)
   {
      if (event.getGestureSource() == textField)
         return false;

      return retrieveYoCompositesFromDragboard(event.getDragboard()) != null;
   }

   public TextField getTextField()
   {
      return textField;
   }
}

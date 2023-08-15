package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField;

import java.util.ArrayList;
import java.util.List;

import org.controlsfx.control.textfield.AutoCompletionBinding;
import org.controlsfx.control.textfield.TextFields;

import javafx.scene.control.TextField;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.TransferMode;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositePattern;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoCompositeSearchField
{
   private final TextField textField;
   private final int numberOfComponents;
   private final YoCompositePattern yoCompositePattern;
   private final YoCompositeCollection yoCompositeCollection;
   private final YoCompositeCollection yoVariableCollection;
   private AutoCompletionBinding<String> autoCompletionBinding;

   private TextField[] componentTextFields;

   public YoCompositeSearchField(YoCompositeSearchManager searchManager, YoCompositeCollection yoCompositeCollection, TextField textField)
   {
      this.textField = textField;
      this.yoCompositeCollection = yoCompositeCollection;

      yoCompositePattern = yoCompositeCollection.getPattern();
      yoVariableCollection = searchManager.getYoVariableCollection();
      numberOfComponents = yoCompositePattern.getComponentIdentifiers().length;

      textField.setOnDragEntered(this::handleDragEntered);
      textField.setOnDragExited(this::handleDragExited);
      textField.setOnDragOver(this::handleDragOver);
      textField.setOnDragDropped(this::handleDragDropped);
   }

   public void setInput(YoComposite input)
   {
      if (yoCompositeCollection.getYoCompositeFromUniqueName(input.getUniqueName()) != null)
      {
         textField.setText(input.getUniqueName());
         setIndividualComponentFields(input);
      }
   }

   public void initializeFieldFromComponents()
   {
      if (componentTextFields == null)
         return;

      List<YoVariable> components = new ArrayList<>(numberOfComponents);

      for (int i = 0; i < numberOfComponents; i++)
      {
         YoComposite yoVariableAsComposite = yoVariableCollection.getYoCompositeFromUniqueName(componentTextFields[i].getText());

         if (yoVariableAsComposite == null || !yoVariableAsComposite.getPattern().getType().equals(YoCompositeTools.YO_VARIABLE))
            return;

         components.add(yoVariableAsComposite.getYoComponents().get(0));
      }

      // This computes the name instead of the uniqueName, need to figure out the uniqueName after.
      String compositeName = YoCompositeTools.getYoCompositeName(yoCompositePattern, components);

      if (compositeName != null)
      { // It is a composite, so it should in the collection so we can retrieve the unique name.
         String uniqueName = yoCompositeCollection.getYoComposites().stream().filter(composite -> composite.getYoComponents().contains(components.get(0)))
                                                  .findFirst().get().getUniqueName();
         textField.setText(uniqueName);
      }
   }

   public void setupAutoCompletion()
   {
      autoCompletionBinding = TextFields.bindAutoCompletion(textField, yoCompositeCollection.uniqueNameCollection());
      autoCompletionBinding.prefWidthProperty().bind(textField.widthProperty());
      textField.addEventHandler(KeyEvent.KEY_PRESSED, e ->
      {
         if (textField.isFocused() && e.isControlDown() && e.getCode() == KeyCode.SPACE)
         {
            autoCompletionBinding.setUserInput(textField.getText());
            e.consume();
         }
      });

      if (componentTextFields != null)
         autoCompletionBinding.setOnAutoCompleted(event -> setIndividualComponentFields(yoCompositeCollection.getYoCompositeFromUniqueName(event.getCompletion())));
   }

   public void attachIndividualComponentFields(TextField[] componentTextFields)
   {
      this.componentTextFields = componentTextFields;

      if (autoCompletionBinding != null)
         autoCompletionBinding.setOnAutoCompleted(event -> setIndividualComponentFields(yoCompositeCollection.getYoCompositeFromUniqueName(event.getCompletion())));
   }

   private void setIndividualComponentFields(YoComposite yoComposite)
   {
      List<YoVariable> components = yoComposite.getYoComponents();

      for (int i = 0; i < numberOfComponents; i++)
      {
         componentTextFields[i].setText(yoVariableCollection.getYoVariableUniqueName(components.get(i)));
      }
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

   private void handleDragDropped(DragEvent event)
   {
      Dragboard dragboard = event.getDragboard();
      boolean success = false;
      List<YoComposite> yoComposites = DragAndDropTools.retrieveYoCompositesFromDragBoard(dragboard, yoCompositeCollection);
      if (yoComposites != null)
      {
         success = true;
         textField.setText(yoComposites.get(0).getUniqueName());
         setIndividualComponentFields(yoComposites.get(0));
      }
      event.setDropCompleted(success);
      event.consume();
   }

   private boolean acceptDragEventForDrop(DragEvent event)
   {
      if (event.getGestureSource() == textField)
         return false;

      return DragAndDropTools.retrieveYoCompositesFromDragBoard(event.getDragboard(), yoCompositeCollection) != null;
   }
}

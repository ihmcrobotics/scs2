package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField;

import java.util.List;
import java.util.Map;

import org.controlsfx.control.textfield.AutoCompletionBinding;
import org.controlsfx.control.textfield.TextFields;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;

public class YoCompositeListTextField
{
   private TextField textField;
   private final Map<String, List<YoComposite>> yoCompositeListMap;
   private final ObjectProperty<List<YoComposite>> compositeListProperty = new SimpleObjectProperty<>(this, "compositeList", null);
   private AutoCompletionBinding<String> autoCompletionBinding;

   public YoCompositeListTextField(Map<String, List<YoComposite>> yoCompositeListMap, TextField textField)
   {
      this.textField = textField;
      this.yoCompositeListMap = yoCompositeListMap;
   }

   public void setupAutoCompletion()
   {
      autoCompletionBinding = TextFields.bindAutoCompletion(textField, yoCompositeListMap.keySet());
      autoCompletionBinding.prefWidthProperty().bind(textField.widthProperty());
      textField.addEventHandler(KeyEvent.KEY_PRESSED, e ->
      {
         if (textField.isFocused() && e.isControlDown() && e.getCode() == KeyCode.SPACE)
         {
            autoCompletionBinding.setUserInput(textField.getText());
            e.consume();
         }
      });

      autoCompletionBinding.setOnAutoCompleted(event -> compositeListProperty.set(yoCompositeListMap.get(event.getCompletion())));
   }

   public ReadOnlyObjectProperty<List<YoComposite>> compositeListProperty()
   {
      return compositeListProperty;
   }
}

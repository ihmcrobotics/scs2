package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.control.TextField;
import javafx.scene.control.Tooltip;
import javafx.scene.image.ImageView;
import javafx.scene.input.Dragboard;
import javafx.util.Callback;
import org.controlsfx.control.textfield.AutoCompletionBinding.ISuggestionRequest;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class ReferenceFrameSearchField extends PropertySearchField<Property<ReferenceFrameWrapper>>
{
   private static final String DEFAULT_TEXT_FILL = "black";
   private static final String UNDEFINED_TEXT_FILL = "darkred";

   private final ReferenceFrameManager referenceFrameManager;

   public ReferenceFrameSearchField(TextField textField, ReferenceFrameManager referenceFrameManager)
   {
      this(textField, referenceFrameManager, null);
   }

   public ReferenceFrameSearchField(TextField textField, ReferenceFrameManager referenceFrameManager, ImageView validImageView)
   {
      super(textField, validImageView);
      this.referenceFrameManager = referenceFrameManager;
      textField.setText(referenceFrameManager.getWorldFrame().getName());

      updateTextFieldStyle(textField, true);

      textField.textProperty().addListener(new ChangeListener<String>()
      {
         private ReferenceFrameWrapper previousFrame = referenceFrameManager.getWorldFrame();
         private final Runnable frameChangeListener = () -> updateTextFieldStyle(textField, true);

         @Override
         public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue)
         {
            if (newValue == null || newValue.equals(oldValue))
               return;

            ReferenceFrameWrapper frame = ReferenceFrameSearchField.this.findReferenceFrame(newValue);
            updateTextFieldStyle(textField, frame == null || frame.isDefined());
            if (previousFrame != null)
               previousFrame.removeChangeListener(frameChangeListener);
            if (frame != null && !frame.isDefined())
               frame.addChangeListener(frameChangeListener);
            previousFrame = frame;
         }
      });
   }

   private static void updateTextFieldStyle(TextField textField, boolean isFrameDefined)
   {
      textField.setStyle("-fx-text-fill: %s;".formatted(isFrameDefined ? DEFAULT_TEXT_FILL : UNDEFINED_TEXT_FILL));
      if (!isFrameDefined)
         textField.setTooltip(new Tooltip("Undefined reference frame."));
   }

   @Override
   protected boolean isTextValid(String text)
   {
      if (text == null || text.isEmpty())
         return false;

      ReferenceFrameWrapper frame = findReferenceFrame(text);
      return frame != null;
   }

   private ReferenceFrameWrapper findReferenceFrame(String name)
   {
      ReferenceFrameWrapper frame = referenceFrameManager.getReferenceFrameFromUniqueName(name);
      if (frame == null)
         frame = referenceFrameManager.getReferenceFrameFromFullname(name);
      return frame;
   }

   @Override
   protected String simplifyText(String text)
   {
      if (text == null)
         return referenceFrameManager.getWorldFrame().getUniqueShortName();

      ReferenceFrameWrapper frame = findReferenceFrame(text);

      String uniqueName = frame.getUniqueShortName();
      if (uniqueName != null && uniqueName.equals(text))
         return null;

      return uniqueName;
   }

   @Override
   protected Callback<ISuggestionRequest, Collection<String>> createSuggestions()
   {
      return request ->
      {
         Collection<String> uniqueNames = referenceFrameManager.getReferenceFrameUniqueShortNames();
         String userText = request.getUserText();
         if (userText.isEmpty())
            return uniqueNames;

         String userTextLowerCase = userText.toLowerCase();
         return uniqueNames.stream().filter(v -> v.toLowerCase().contains(userTextLowerCase)).collect(Collectors.toList());
      };
   }

   @Override
   protected Property<ReferenceFrameWrapper> toSupplier(String text)
   {
      ReferenceFrameWrapper referenceFrame = referenceFrameManager.getReferenceFrameFromUniqueName(text);
      if (referenceFrame == null)
         return null;
      else
         return new SimpleObjectProperty<>(referenceFrame);
   }

   @Override
   protected List<YoComposite> retrieveYoCompositesFromDragboard(Dragboard dragboard)
   {
      LogTools.error("Unsupported operation, implement me!");
      return null;
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField;

import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.input.Dragboard;
import javafx.util.Callback;
import org.controlsfx.control.textfield.AutoCompletionBinding.ISuggestionRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Wrapper around a simple {@link TextField} to add search functionality, auto-completion, validation.
 * <p>
 * The search field is valid if the text is not empty and if it is a valid {@link YoComposite} name or a parsable number.
 * </p>
 */
public class StringSearchField extends PropertySearchField<StringProperty>
{
   private final YoCompositeCollection yoVariableCollection;
   private final LinkedYoRegistry linkedRootRegistry;

   public StringSearchField(TextField textField, YoCompositeSearchManager searchManager, LinkedYoRegistry linkedRootRegistry)
   {
      this(textField, searchManager, linkedRootRegistry, null);
   }

   public StringSearchField(TextField textField, YoCompositeSearchManager searchManager, LinkedYoRegistry linkedRootRegistry, ImageView validImageView)
   {
      super(textField, validImageView);
      this.linkedRootRegistry = linkedRootRegistry;
      yoVariableCollection = searchManager.getYoVariableCollection();
   }

   @Override
   protected boolean isTextValid(String text)
   {
      if (text == null || text.isEmpty())
         return false;

      YoComposite yoComposite = yoVariableCollection.getYoCompositeFromUniqueName(text);
      if (yoComposite == null)
         yoComposite = yoVariableCollection.getYoCompositeFromFullname(text);
      return yoComposite != null || CompositePropertyTools.isParsableAsNumber(text);
   }

   @Override
   protected String simplifyText(String text)
   {
      YoComposite yoComposite = yoVariableCollection.getYoCompositeFromFullname(text);
      return yoComposite == null ? null : yoComposite.getUniqueName();
   }

   @Override
   protected Callback<ISuggestionRequest, Collection<String>> createSuggestions()
   {
      Collection<String> uniqueNameCollection = yoVariableCollection.uniqueNameCollection();

      return request ->
      {
         String userText = request.getUserText();

         if (CompositePropertyTools.isParsableAsNumber(userText))
            return null;

         if (userText.isEmpty())
            return uniqueNameCollection;

         String userTextLowerCase = userText.toLowerCase();
         return uniqueNameCollection.stream().filter(v -> v.toLowerCase().contains(userTextLowerCase)).collect(Collectors.toList());
      };
   }

   @Override
   protected StringProperty toSupplier(String text)
   {
      YoComposite yoComposite = yoVariableCollection.getYoCompositeFromUniqueName(text);
      if (yoComposite == null)
         return new SimpleStringProperty(text);
      else
         return new SimpleStringProperty(yoComposite.getYoComponents().get(0).getFullNameString());
   }

   @Override
   protected List<YoComposite> retrieveYoCompositesFromDragboard(Dragboard dragboard)
   {
      return DragAndDropTools.retrieveYoCompositesFromDragBoard(dragboard, yoVariableCollection);
   }
}

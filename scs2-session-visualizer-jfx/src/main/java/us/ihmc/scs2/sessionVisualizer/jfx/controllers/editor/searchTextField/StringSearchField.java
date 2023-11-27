package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField;

import javafx.beans.property.*;
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
   private final YoCompositeSearchManager searchManager;
   private final BooleanProperty isNumberProperty = new SimpleBooleanProperty(this, "isNumber", false);
   private final BooleanProperty isYoCompositeProperty = new SimpleBooleanProperty(this, "isYoComposite", false);
   private final Property<YoComposite> yoCompositeProperty = new SimpleObjectProperty<>(this, "yoComposite", null);

   public StringSearchField(TextField textField, YoCompositeSearchManager searchManager)
   {
      this(textField, searchManager, null);
   }

   public StringSearchField(TextField textField, YoCompositeSearchManager searchManager, ImageView validImageView)
   {
      super(textField, validImageView);
      this.searchManager = searchManager;

      supplierProperty().addListener((o, oldValue, newValue) ->
                                     {
                                        if (newValue == null)
                                        {
                                           isNumberProperty.set(false);
                                           isYoCompositeProperty.set(false);
                                        }
                                        else
                                        {
                                           isNumberProperty.set(CompositePropertyTools.isParsableAsNumber(newValue.get()));
                                           YoCompositeCollection yoVariableCollection = searchManager.getYoVariableCollection();

                                           YoComposite yoComposite = yoVariableCollection.getYoCompositeFromUniqueName(newValue.get());
                                           if (yoComposite == null)
                                              yoComposite = yoVariableCollection.getYoCompositeFromFullname(newValue.get());
                                           yoCompositeProperty.setValue(yoComposite);
                                           isYoCompositeProperty.set(yoCompositeProperty.getValue() != null);
                                        }
                                     });
   }

   @Override
   protected boolean isTextValid(String text)
   {
      if (text == null || text.isEmpty())
         return false;

      YoCompositeCollection yoVariableCollection = searchManager.getYoVariableCollection();
      YoComposite yoComposite = yoVariableCollection.getYoCompositeFromUniqueName(text);
      if (yoComposite == null)
         yoComposite = yoVariableCollection.getYoCompositeFromFullname(text);
      return yoComposite != null || CompositePropertyTools.isParsableAsNumber(text);
   }

   @Override
   protected String simplifyText(String text)
   {
      YoCompositeCollection yoVariableCollection = searchManager.getYoVariableCollection();
      YoComposite yoComposite = yoVariableCollection.getYoCompositeFromFullname(text);
      return yoComposite == null ? null : yoComposite.getUniqueName();
   }

   @Override
   protected Callback<ISuggestionRequest, Collection<String>> createSuggestions()
   {

      return request ->
      {
         String userText = request.getUserText();

         if (CompositePropertyTools.isParsableAsNumber(userText))
            return null;

         YoCompositeCollection yoVariableCollection = searchManager.getYoVariableCollection();
         Collection<String> uniqueNameCollection = yoVariableCollection.uniqueNameCollection();

         if (userText.isEmpty())
            return uniqueNameCollection;

         String userTextLowerCase = userText.toLowerCase();
         return uniqueNameCollection.stream().filter(v -> v.toLowerCase().contains(userTextLowerCase)).collect(Collectors.toList());
      };
   }

   @Override
   protected StringProperty toSupplier(String text)
   {
      YoCompositeCollection yoVariableCollection = searchManager.getYoVariableCollection();
      YoComposite yoComposite = yoVariableCollection.getYoCompositeFromUniqueName(text);
      if (yoComposite == null)
         return new SimpleStringProperty(text);
      else
         return new SimpleStringProperty(yoComposite.getYoComponents().get(0).getFullNameString());
   }

   @Override
   protected List<YoComposite> retrieveYoCompositesFromDragboard(Dragboard dragboard)
   {
      return DragAndDropTools.retrieveYoCompositesFromDragBoard(dragboard, searchManager.getYoVariableCollection());
   }

   public BooleanProperty isNumberProperty()
   {
      return isNumberProperty;
   }

   public boolean isNumber()
   {
      return isNumberProperty.get();
   }

   public BooleanProperty isYoCompositeProperty()
   {
      return isYoCompositeProperty;
   }

   public boolean isYoComposite()
   {
      return isYoCompositeProperty.get();
   }

   public Property<YoComposite> yoCompositeProperty()
   {
      return yoCompositeProperty;
   }

   public YoComposite getYoComposite()
   {
      return yoCompositeProperty.getValue();
   }
}

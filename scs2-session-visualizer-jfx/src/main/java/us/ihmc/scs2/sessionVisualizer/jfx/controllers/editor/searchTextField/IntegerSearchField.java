package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.input.Dragboard;
import javafx.util.Callback;
import org.controlsfx.control.textfield.AutoCompletionBinding.ISuggestionRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoIntegerProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class IntegerSearchField extends PropertySearchField<IntegerProperty>
{
   private final YoCompositeSearchManager searchManager;
   private final LinkedYoRegistry linkedRootRegistry;
   private final boolean isInputOptional;

   public IntegerSearchField(TextField textField, YoCompositeSearchManager searchManager, LinkedYoRegistry linkedRootRegistry)
   {
      this(textField, searchManager, linkedRootRegistry, null);
   }

   public IntegerSearchField(TextField textField, YoCompositeSearchManager searchManager, LinkedYoRegistry linkedRootRegistry, ImageView validImageView)
   {
      this(textField, searchManager, linkedRootRegistry, false, validImageView);
   }

   public IntegerSearchField(TextField textField,
                             YoCompositeSearchManager searchManager,
                             LinkedYoRegistry linkedRootRegistry,
                             boolean isInputOptional,
                             ImageView validImageView)
   {
      super(textField, validImageView);
      this.searchManager = searchManager;
      this.linkedRootRegistry = linkedRootRegistry;
      this.isInputOptional = isInputOptional;
   }

   @Override
   protected boolean isTextValid(String text)
   {
      if (text == null || text.isEmpty())
         return isInputOptional;

      YoCompositeCollection yoIntegerCollection = searchManager.getYoIntegerCollection();
      YoComposite yoComposite = yoIntegerCollection.getYoCompositeFromUniqueName(text);
      if (yoComposite == null)
         yoComposite = yoIntegerCollection.getYoCompositeFromFullname(text); // TODO Happens when loading file, needs to update TextField to use unique name.
      return yoComposite != null || CompositePropertyTools.isParsableAsInteger(text);
   }

   @Override
   protected String simplifyText(String text)
   {
      YoCompositeCollection yoIntegerCollection = searchManager.getYoIntegerCollection();
      YoComposite yoComposite = yoIntegerCollection.getYoCompositeFromFullname(text);
      return yoComposite == null ? null : yoComposite.getUniqueName();
   }

   @Override
   protected Callback<ISuggestionRequest, Collection<String>> createSuggestions()
   {
      return request ->
      {
         String userText = request.getUserText();
         if (CompositePropertyTools.isParsableAsInteger(userText))
            return null;

         YoCompositeCollection yoIntegerCollection = searchManager.getYoIntegerCollection();
         Collection<String> uniqueNameCollection = yoIntegerCollection.uniqueNameCollection();

         if (userText.isEmpty())
            return uniqueNameCollection;

         String userTextLowerCase = userText.toLowerCase();
         return uniqueNameCollection.stream().filter(v -> v.toLowerCase().contains(userTextLowerCase)).collect(Collectors.toList());
      };
   }

   @Override
   protected IntegerProperty toSupplier(String text)
   {
      if (isInputOptional && text == null)
         return null;

      YoCompositeCollection yoIntegerCollection = searchManager.getYoIntegerCollection();
      YoComposite yoComposite = yoIntegerCollection.getYoCompositeFromUniqueName(text);
      if (yoComposite == null)
      {
         return new SimpleIntegerProperty(Integer.parseInt(text));
      }
      else
      {
         YoIntegerProperty yoIntegerProperty = new YoIntegerProperty((YoInteger) yoComposite.getYoComponents().get(0));
         yoIntegerProperty.setLinkedBuffer(linkedRootRegistry.linkYoVariable(yoIntegerProperty.getYoVariable(), yoIntegerProperty));
         return yoIntegerProperty;
      }
   }

   @Override
   protected List<YoComposite> retrieveYoCompositesFromDragboard(Dragboard dragboard)
   {
      YoCompositeCollection yoIntegerCollection = searchManager.getYoIntegerCollection();
      return DragAndDropTools.retrieveYoCompositesFromDragBoard(dragboard, yoIntegerCollection);
   }
}

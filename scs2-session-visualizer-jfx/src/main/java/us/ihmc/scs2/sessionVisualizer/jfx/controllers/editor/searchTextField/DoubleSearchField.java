package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.input.Dragboard;
import javafx.util.Callback;
import org.controlsfx.control.textfield.AutoCompletionBinding.ISuggestionRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class DoubleSearchField extends PropertySearchField<DoubleProperty>
{
   private final YoCompositeSearchManager searchManager;
   private final LinkedYoRegistry linkedRootRegistry;

   public DoubleSearchField(TextField textField, YoCompositeSearchManager searchManager, LinkedYoRegistry linkedRootRegistry)
   {
      this(textField, searchManager, linkedRootRegistry, null);
   }

   public DoubleSearchField(TextField textField, YoCompositeSearchManager searchManager, LinkedYoRegistry linkedRootRegistry, ImageView validImageView)
   {
      super(textField, validImageView);
      this.searchManager = searchManager;
      this.linkedRootRegistry = linkedRootRegistry;
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
      return yoComposite != null || CompositePropertyTools.isParsableAsDouble(text);
   }

   @Override
   protected String simplifyText(String text)
   {
      YoCompositeCollection yoVariableCollection = searchManager.getYoVariableCollection();
      YoComposite yoComposite = yoVariableCollection.getYoCompositeFromFullname(text);
      if (yoComposite == null)
         yoComposite = yoVariableCollection.getYoCompositeFromUniqueName(text);
      return yoComposite == null ? null : yoComposite.getUniqueShortName();
   }

   @Override
   protected Callback<ISuggestionRequest, Collection<String>> createSuggestions()
   {
      return request ->
      {
         String userText = request.getUserText();

         if (CompositePropertyTools.isParsableAsDouble(userText))
            return null;

         YoCompositeCollection yoVariableCollection = searchManager.getYoVariableCollection();
         Collection<String> uniqueShortNameCollection = yoVariableCollection.uniqueShortNameCollection();

         if (userText.isEmpty())
            return uniqueShortNameCollection;

         String userTextLowerCase = userText.toLowerCase();
         return uniqueShortNameCollection.stream().filter(v -> v.toLowerCase().contains(userTextLowerCase)).collect(Collectors.toList());
      };
   }

   @Override
   protected DoubleProperty toSupplier(String text)
   {
      YoCompositeCollection yoVariableCollection = searchManager.getYoVariableCollection();
      YoComposite yoComposite = yoVariableCollection.getYoCompositeFromUniqueName(text);
      if (yoComposite == null)
      {
         return new SimpleDoubleProperty(Double.parseDouble(text));
      }
      else
      {
         YoDoubleProperty yoDoubleProperty = new YoDoubleProperty((YoDouble) yoComposite.getYoComponents().get(0));
         yoDoubleProperty.setLinkedBuffer(linkedRootRegistry.linkYoVariable(yoDoubleProperty.getYoVariable(), yoDoubleProperty));
         return yoDoubleProperty;
      }
   }

   @Override
   protected List<YoComposite> retrieveYoCompositesFromDragboard(Dragboard dragboard)
   {
      return DragAndDropTools.retrieveYoCompositesFromDragBoard(dragboard, searchManager.getYoVariableCollection());
   }
}

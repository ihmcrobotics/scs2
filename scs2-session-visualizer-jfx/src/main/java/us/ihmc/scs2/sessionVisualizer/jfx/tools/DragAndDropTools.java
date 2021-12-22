package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DataFormat;
import javafx.scene.input.Dragboard;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

@SuppressWarnings("unchecked")
public class DragAndDropTools
{
   public static final DataFormat YO_COMPOSITE_REFERENCE = new DataFormat("Reference to a YoComposite, List<String> of 2 elements: 0-type of the YoComposite, 1-the ownerFullname.");
   public static final DataFormat YO_COMPOSITE_LIST_REFERENCE = new DataFormat("Reference to a list of YoComposites, List<String> of 2xN elements: 0-type of the YoComposite, 1-the ownerFullname; and N is the number of YoComposites in the list.");
   public static final DataFormat YO_GRAPHIC_ITEMS_REFERENCE = new DataFormat("Reference to a list of YoGraphicFXItems, per YoGraphicFXItem there are 3 elements: 0-type of the item, 1-the item name, 2-the item namespace.");

   public static List<YoComposite> retrieveYoCompositesFromDragBoard(Dragboard dragboard, YoCompositeSearchManager searchManager)
   {
      if (dragboard.hasContent(YO_COMPOSITE_REFERENCE))
      {
         List<String> content = (List<String>) dragboard.getContent(YO_COMPOSITE_REFERENCE);
         String type = content.get(0);
         String fullname = content.get(1);
         YoComposite yoComposite = searchManager.getYoComposite(type, fullname);
         return yoComposite == null ? null : Collections.singletonList(yoComposite);
      }
      else if (dragboard.hasContent(YO_COMPOSITE_LIST_REFERENCE))
      {
         List<String> content = (List<String>) dragboard.getContent(YO_COMPOSITE_LIST_REFERENCE);
         List<YoComposite> compositeList = new ArrayList<>();

         for (int i = 0; i < content.size(); i += 2)
         {
            String type = content.get(i);
            String fullname = content.get(i + 1);
            YoComposite yoComposite = searchManager.getYoComposite(type, fullname);
            if (yoComposite != null)
               compositeList.add(yoComposite);
         }
         return compositeList.isEmpty() ? null : compositeList;
      }
      else
      {
         return null;
      }
   }

   public static List<YoComposite> retrieveYoCompositesFromDragBoard(Dragboard dragboard, YoCompositeCollection yoCompositeCollection)
   {
      if (dragboard.hasContent(YO_COMPOSITE_REFERENCE))
      {
         List<String> content = (List<String>) dragboard.getContent(YO_COMPOSITE_REFERENCE);
         String type = content.get(0);
         if (!type.equals(yoCompositeCollection.getPattern().getType()))
            return null;
         String fullname = content.get(1);
         YoComposite yoComposite = yoCompositeCollection.getYoCompositeFromFullname(fullname);
         return yoComposite == null ? null : Collections.singletonList(yoComposite);
      }
      else if (dragboard.hasContent(YO_COMPOSITE_LIST_REFERENCE))
      {
         List<String> content = (List<String>) dragboard.getContent(YO_COMPOSITE_REFERENCE);
         List<YoComposite> compositeList = new ArrayList<>();

         for (int i = 0; i < content.size(); i += 2)
         {
            String type = content.get(i);
            if (!type.equals(yoCompositeCollection.getPattern().getType()))
               continue;
            String fullname = content.get(i + 1);
            compositeList.add(yoCompositeCollection.getYoCompositeFromFullname(fullname));
         }
         return compositeList.isEmpty() ? null : compositeList;
      }
      else
      {
         return null;
      }
   }

   public static ClipboardContent toClipboardContent(List<? extends YoGraphicFXItem> items)
   {
      List<String> content = new ArrayList<>();

      for (YoGraphicFXItem item : items)
      {
         String itemType = item.getClass().getSimpleName();

         if (itemType == null)
            throw new RuntimeException("Unexpected item type: " + item.getClass());

         String itemName = item.getName();
         String itemNamespace = item.getNamespace();

         content.addAll(Arrays.asList(itemType, itemName, itemNamespace));
      }

      ClipboardContent clipboardContent = new ClipboardContent();
      clipboardContent.put(YO_GRAPHIC_ITEMS_REFERENCE, content);
      return clipboardContent;
   }

   public static List<YoGraphicFXItem> retrieveYoGraphicFXItemsFromDragBoard(Dragboard dragboard, YoGroupFX rootGroup)
   {
      if (!dragboard.hasContent(YO_GRAPHIC_ITEMS_REFERENCE))
         return null;

      List<YoGraphicFXItem> items = new ArrayList<>();
      List<String> content = (List<String>) dragboard.getContent(YO_GRAPHIC_ITEMS_REFERENCE);
      for (int i = 0; i < content.size(); i += 3)
      {
         String itemTypeName = content.get(i);
         Class<? extends YoGraphicFXItem> itemType = YoGraphicFXControllerTools.yoGraphicFXTypes.stream()
                                                                                                .filter(type -> type.getSimpleName().equals(itemTypeName))
                                                                                                .findFirst().get();
         String itemName = content.get(i + 1);
         String itemNamespace = content.get(i + 2);
         YoGraphicFXItem item = YoGraphicTools.findYoGraphicFXItem(rootGroup, itemNamespace, itemName, itemType);
         if (item != null)
            items.add(item);
      }
      if (items.isEmpty())
         return null;
      else
         return items;
   }
}

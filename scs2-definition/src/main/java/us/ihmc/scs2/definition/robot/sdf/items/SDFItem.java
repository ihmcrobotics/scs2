package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public interface SDFItem
{
   String getContentAsString();

   default String format(String format, Object... args)
   {
      for (int i = 0; i < args.length; i++)
      {
         if (args[i] instanceof SDFItem)
            args[i] = ((SDFItem) args[i]).getContentAsString();
      }
      return String.format(format, args);
   }

   default String itemToString()
   {
      return getClass().getSimpleName() + ": " + getContentAsString();
   }

   List<? extends SDFURIHolder> getURIHolders();

   static List<SDFURIHolder> combineItemURIHolders(SDFItem... sdfItems)
   {
      if (sdfItems == null || sdfItems.length == 0)
         return Collections.emptyList();

      List<SDFURIHolder> uriHolders = new ArrayList<>();

      for (SDFItem urdfItem : sdfItems)
      {
         if (urdfItem != null)
            uriHolders.addAll(urdfItem.getURIHolders());
      }
      return uriHolders;
   }

   @SafeVarargs
   static List<SDFURIHolder> combineItemListsURIHolders(List<? extends SDFItem>... sdfItemLists)
   {
      if (sdfItemLists == null || sdfItemLists.length == 0)
         return Collections.emptyList();

      List<SDFURIHolder> uriHolders = new ArrayList<>();

      for (List<? extends SDFItem> urdfItemList : sdfItemLists)
      {
         if (urdfItemList != null)
         {
            for (SDFItem urdfItem : urdfItemList)
            {
               if (urdfItem != null)
                  uriHolders.addAll(urdfItem.getURIHolders());
            }
         }
      }

      return uriHolders;
   }
}
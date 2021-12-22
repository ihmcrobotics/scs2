package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public interface URDFItem
{
   String getContentAsString();

   default String format(String format, Object... args)
   {
      for (int i = 0; i < args.length; i++)
         if (args[i] instanceof URDFItem)
            args[i] = ((URDFItem) args[i]).getContentAsString();
      return String.format(format, args);
   }

   default String itemToString()
   {
      return getClass().getSimpleName() + ": " + getContentAsString();
   }

   List<URDFFilenameHolder> getFilenameHolders();

   static List<URDFFilenameHolder> combineItemFilenameHolders(URDFItem... urdfItems)
   {
      if (urdfItems == null || urdfItems.length == 0)
         return Collections.emptyList();

      List<URDFFilenameHolder> filenameHolders = new ArrayList<>();

      for (URDFItem urdfItem : urdfItems)
      {
         if (urdfItem != null)
            filenameHolders.addAll(urdfItem.getFilenameHolders());
      }
      return filenameHolders;
   }

   @SafeVarargs
   static List<URDFFilenameHolder> combineItemListsFilenameHolders(List<? extends URDFItem>... urdfItemLists)
   {
      if (urdfItemLists == null || urdfItemLists.length == 0)
         return Collections.emptyList();

      List<URDFFilenameHolder> filenameHolders = new ArrayList<>();

      for (List<? extends URDFItem> urdfItemList : urdfItemLists)
      {
         if (urdfItemList != null)
         {
            for (URDFItem urdfItem : urdfItemList)
            {
               if (urdfItem != null)
                  filenameHolders.addAll(urdfItem.getFilenameHolders());
            }
         }
      }

      return filenameHolders;
   }
}

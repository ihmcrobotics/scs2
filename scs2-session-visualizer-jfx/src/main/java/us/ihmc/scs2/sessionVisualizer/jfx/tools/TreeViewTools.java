package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import javafx.scene.control.TreeItem;

public class TreeViewTools
{
   public static void expandRecursively(TreeItem<?> item)
   {
      if (item == null)
         return;
      item.setExpanded(true);
      for (TreeItem<?> child : item.getChildren())
         expandRecursively(child);
   }

   public static void collapseRecursively(TreeItem<?> item)
   {
      if (item != null && !item.isLeaf())
      {
         item.setExpanded(false);

         for (TreeItem<?> child : item.getChildren())
            collapseRecursively(child);
      }
   }

   public static <T> TreeItem<T> findItem(TreeItem<T> parent, T value)
   {
      if (parent == null)
         return null;

      if (parent.getValue() == value)
         return parent;

      for (TreeItem<T> child : parent.getChildren())
      {
         TreeItem<T> result = findItem(child, value);
         if (result != null)
            return result;
      }

      return null;
   }
}

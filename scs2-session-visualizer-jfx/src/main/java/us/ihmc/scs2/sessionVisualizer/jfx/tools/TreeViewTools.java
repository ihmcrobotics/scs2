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

}

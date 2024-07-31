package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import javafx.collections.ObservableList;
import javafx.scene.control.MenuItem;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeTableView;
import org.kordamp.ikonli.javafx.FontIcon;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

public class TreeTableViewTools
{
   public static <T> Function<TreeTableView<T>, MenuItem> removeMenuItemFactory(boolean allowMultipleSelection)
   {
      return removeMenuItemFactory(allowMultipleSelection, value -> true, t ->
      {
      });
   }

   public static <T> Function<TreeTableView<T>, MenuItem> removeMenuItemFactory(boolean allowMultipleSelection, Predicate<T> filter, Consumer<T> removeAction)
   {
      return treeTableView ->
      {
         FontIcon removeIcon = new FontIcon();
         removeIcon.getStyleClass().add("remove-icon-view");
         MenuItem removeMenuItem = new MenuItem("Remove", removeIcon);

         if (allowMultipleSelection)
         {
            ObservableList<TreeItem<T>> selectedItems = treeTableView.getSelectionModel().getSelectedItems();
            if (selectedItems.isEmpty())
               return null;
            boolean areNoneRemovable = selectedItems.stream().noneMatch(item -> filter.test(item.getValue()));
            if (areNoneRemovable)
               return null;

            removeMenuItem.setOnAction(e2 ->
                                       {
                                          selectedItems.forEach(item ->
                                                                {
                                                                   TreeItem<T> parent = item.getParent();
                                                                   if (parent != null)
                                                                   {
                                                                      parent.getChildren().remove(item);
                                                                      removeAction.accept(item.getValue());
                                                                   }
                                                                });
                                       });
         }
         else
         {
            TreeItem<T> selectedItem = treeTableView.getSelectionModel().getSelectedItem();
            if (selectedItem == null)
               return null;

            if (!filter.test(selectedItem.getValue()))
               return null;

            removeMenuItem.setOnAction(e2 ->
                                       {
                                          TreeItem<T> parent = selectedItem.getParent();
                                          if (parent != null)
                                          {
                                             parent.getChildren().remove(selectedItem);
                                             removeAction.accept(selectedItem.getValue());
                                          }
                                       });
         }
         return removeMenuItem;
      };
   }
}

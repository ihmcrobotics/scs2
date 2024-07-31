package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import javafx.collections.ObservableList;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;
import org.kordamp.ikonli.javafx.FontIcon;

import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.function.Supplier;

public class ListViewTools
{
   public static <T> Function<ListView<T>, MenuItem> removeMenuItemFactory(boolean allowMultipleSelection)
   {
      return listView ->
      {
         FontIcon removeIcon = new FontIcon();
         removeIcon.getStyleClass().add("remove-icon-view");
         MenuItem removeMenuItem = new MenuItem("Remove", removeIcon);

         if (allowMultipleSelection)
         {
            ObservableList<T> selectedItems = listView.getSelectionModel().getSelectedItems();
            if (selectedItems.isEmpty())
               return null;
            removeMenuItem.setOnAction(e2 -> listView.getItems().removeAll(selectedItems));
         }
         else
         {
            T selectedItem = listView.getSelectionModel().getSelectedItem();
            if (selectedItem == null)
               return null;
            removeMenuItem.setOnAction(e2 -> listView.getItems().remove(selectedItem));
         }
         return removeMenuItem;
      };
   }

   public static <T> Function<ListView<T>, MenuItem> removeAllMenuItemFactory()
   {
      return listView ->
      {
         FontIcon removeIcon = new FontIcon();
         removeIcon.getStyleClass().add("remove-icon-view");
         MenuItem removeMenuItem = new MenuItem("Remove all", removeIcon);

         if (listView.getItems().isEmpty())
            return null;
         removeMenuItem.setOnAction(e2 -> listView.getItems().clear());
         return removeMenuItem;
      };
   }

   public static <T> Function<ListView<T>, MenuItem> addBeforeMenuItemFactory(Supplier<T> addAction)
   {
      return addBeforeMenuItemFactory(index -> addAction.get());
   }

   public static <T> Function<ListView<T>, MenuItem> addBeforeMenuItemFactory(IntFunction<T> addAction)
   {
      return listView ->
      {
         ObservableList<T> items = listView.getItems();
         int selectedIndex = listView.getSelectionModel().getSelectedIndex();

         FontIcon addBeforeIcon = new FontIcon();
         addBeforeIcon.getStyleClass().add("add-icon-view");
         MenuItem addBefore = new MenuItem("Add before", addBeforeIcon);

         addBefore.setOnAction(e2 ->
                               {
                                  T newItem = addAction.apply(selectedIndex);
                                  if (newItem != null)
                                     items.add(selectedIndex, newItem);
                               });

         return addBefore;
      };
   }

   public static <T> Function<ListView<T>, MenuItem> addAfterMenuItemFactory(Supplier<T> addAction)
   {
      return addBeforeMenuItemFactory(index -> addAction.get());
   }

   public static <T> Function<ListView<T>, MenuItem> addAfterMenuItemFactory(IntFunction<T> addAction)
   {
      return listView ->
      {
         ObservableList<T> items = listView.getItems();
         int selectedIndex = listView.getSelectionModel().getSelectedIndex();

         FontIcon addAfterIcon = new FontIcon();
         addAfterIcon.getStyleClass().add("add-icon-view");
         MenuItem addAfter = new MenuItem("Add after", addAfterIcon);

         addAfter.setOnAction(e2 ->
                              {
                                 T newItem = addAction.apply(selectedIndex + 1);
                                 if (newItem != null)
                                    items.add(selectedIndex + 1, newItem);
                              });

         return addAfter;
      };
   }
}

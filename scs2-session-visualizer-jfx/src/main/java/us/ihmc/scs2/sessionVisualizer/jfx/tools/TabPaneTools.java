package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.List;
import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.jfoenix.controls.JFXTextField;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.collections.ObservableList;
import javafx.scene.control.Label;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;

public class TabPaneTools
{
   public static Label editableTabHeader(Tab tab)
   {
      Label headerLabel = new Label();
      JFXTextField headerEditor = new JFXTextField();

      headerLabel.setText(tab.getText());
      tab.setGraphic(headerLabel);
      tab.setText(null);

      headerLabel.setOnMouseClicked(e ->
      {
         if (e.getClickCount() != 2)// || e.getButton() != MouseButton.PRIMARY)
            return;

         headerEditor.setText(headerLabel.getText());
         headerEditor.setPrefWidth(2.0 * headerLabel.getWidth());
         tab.setGraphic(headerEditor);
         headerEditor.selectAll();
         headerEditor.requestFocus();
      });

      headerEditor.setOnAction(e ->
      {
         headerLabel.setText(headerEditor.getText());
         tab.setGraphic(headerLabel);
      });

      headerEditor.focusedProperty().addListener((o, oldValue, newValue) ->
      {
         if (newValue)
            return;

         headerLabel.setText(headerEditor.getText());
         tab.setGraphic(headerLabel);
      });

      return headerLabel;
   }

   public static Function<TabPane, MenuItem> removeMenuItemFactory()
   {
      return tabPane ->
      {
         FontAwesomeIconView removeIcon = new FontAwesomeIconView();
         removeIcon.getStyleClass().add("remove-icon-view");
         MenuItem removeMenuItem = new MenuItem("Remove", removeIcon);

         Tab selectedItem = tabPane.getSelectionModel().getSelectedItem();
         if (selectedItem == null)
            return null;
         removeMenuItem.setOnAction(e2 -> tabPane.getTabs().remove(selectedItem));

         return removeMenuItem;
      };
   }

   public static Function<TabPane, MenuItem> removeAllMenuItemFactory()
   {
      return removeAllMenuItemFactory(false);
   }

   public static Function<TabPane, MenuItem> removeAllMenuItemFactory(boolean removeOnlyClosableTabs)
   {
      return removeAllMenuItemFactory("Remove all", removeOnlyClosableTabs);
   }

   public static Function<TabPane, MenuItem> removeAllMenuItemFactory(String text, boolean removeOnlyClosableTabs)
   {
      return tabPane ->
      {
         FontAwesomeIconView removeAllIcon = new FontAwesomeIconView();
         removeAllIcon.getStyleClass().add("remove-icon-view");
         MenuItem removeMenuItem = new MenuItem(text, removeAllIcon);

         if (tabPane.getTabs().isEmpty())
            return null;

         List<Tab> tabsToclose;
         if (removeOnlyClosableTabs)
            tabsToclose = tabPane.getTabs().stream().filter(Tab::isClosable).collect(Collectors.toList());
         else
            tabsToclose = tabPane.getTabs();

         if (tabsToclose.isEmpty())
            return null;
         removeMenuItem.setOnAction(e2 -> tabPane.getTabs().removeAll(tabsToclose));
         return removeMenuItem;
      };
   }

   public static Function<TabPane, MenuItem> addBeforeMenuItemFactory(Supplier<Tab> addAction)
   {
      return addBeforeMenuItemFactory(i -> addAction.get());
   }

   public static Function<TabPane, MenuItem> addBeforeMenuItemFactory(IntFunction<Tab> addAction)
   {
      return tabPane ->
      {
         ObservableList<Tab> tabs = tabPane.getTabs();
         int selectedIndex = tabPane.getSelectionModel().getSelectedIndex();

         FontAwesomeIconView addBeforeIcon = new FontAwesomeIconView();
         addBeforeIcon.getStyleClass().add("add-icon-view");
         MenuItem addBefore = new MenuItem("Add before", addBeforeIcon);

         addBefore.setOnAction(e2 ->
         {
            Tab newTab = addAction.apply(selectedIndex);
            if (newTab != null)
               tabs.add(selectedIndex, newTab);
         });

         return addBefore;
      };
   }

   public static Function<TabPane, MenuItem> addAfterMenuItemFactory(Supplier<Tab> addAction)
   {
      return addAfterMenuItemFactory(i -> addAction.get());
   }

   public static Function<TabPane, MenuItem> addAfterMenuItemFactory(IntFunction<Tab> addAction)
   {
      return tabPane ->
      {
         ObservableList<Tab> tabs = tabPane.getTabs();
         int selectedIndex = tabPane.getSelectionModel().getSelectedIndex();

         FontAwesomeIconView addAfterIcon = new FontAwesomeIconView();
         addAfterIcon.getStyleClass().add("add-icon-view");
         MenuItem addAfter = new MenuItem("Add after", addAfterIcon);

         addAfter.setOnAction(e2 ->
         {
            Tab newTab = addAction.apply(selectedIndex + 1);
            if (newTab != null)
               tabs.add(selectedIndex + 1, newTab);
         });

         return addAfter;
      };
   }
}

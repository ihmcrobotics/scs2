package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.Stream;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Node;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;

public class ContextMenuTools
{
   @SuppressWarnings("unchecked")
   @SafeVarargs
   public static <T extends Node> void setupContextMenu(T owner, Function<T, MenuItem>... menuItemFactories)
   {
      BiFunction<T, MouseEvent, MenuItem>[] factoryAdapters = Stream.of(menuItemFactories)
                                                                    .map(factory -> (BiFunction<T, MouseEvent, MenuItem>) (t, u) -> factory.apply(t))
                                                                    .toArray(BiFunction[]::new);

      setupContextMenu(owner, factoryAdapters);
   }

   @SafeVarargs
   public static <T extends Node> void setupContextMenu(T owner, BiFunction<T, MouseEvent, MenuItem>... menuItemFactories)
   {
      ObjectProperty<ContextMenu> activeContexMenu = new SimpleObjectProperty<>(owner, "activeContextMenu", null);

      owner.addEventHandler(MouseEvent.MOUSE_RELEASED, e ->
      {
         if (e.getButton() != MouseButton.SECONDARY || !e.isStillSincePress())
            return;

         if (activeContexMenu.get() != null)
         {
            activeContexMenu.get().hide();
            activeContexMenu.set(null);
         }

         MenuItem[] menuItems = Stream.of(menuItemFactories).map(factory -> factory.apply(owner, e)).filter(item -> item != null).toArray(MenuItem[]::new);

         if (menuItems.length == 0)
            return;

         ContextMenu contextMenu = new ContextMenu(menuItems);
         contextMenu.show(owner, e.getScreenX(), e.getScreenY());
         contextMenu.setAutoFix(true);
         contextMenu.setAutoHide(true);
         activeContexMenu.set(contextMenu);

         owner.addEventFilter(MouseEvent.MOUSE_PRESSED, e2 ->
         {
            contextMenu.hide();
            activeContexMenu.set(null);
         });

         e.consume();
      });
   }
}

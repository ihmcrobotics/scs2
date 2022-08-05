package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.Stream;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.CustomMenuItem;
import javafx.scene.control.MenuItem;
import javafx.scene.control.TextField;
import javafx.scene.input.InputEvent;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;

public class MenuTools
{
   /**
    * This is for improving the behavior of a {@code TextField} implemented as part of a
    * {@code CustomMenuItem}.
    * <p>
    * By default, the text field is always losing focus because of the weird event mechanics. This
    * method helps addressing that issue as well as incorporating canceling and validating the edits
    * typed in the text field.
    * </p>
    * 
    * @param owner
    * @param textField
    */
   public static void configureTextFieldForCustomMenuItem(CustomMenuItem owner, TextField textField)
   {
      BooleanProperty editMode = new SimpleBooleanProperty(false);
      EventHandler<? super MouseEvent> eventConsumer = e -> e.consume();

      textField.addEventFilter(InputEvent.ANY, inputEvent ->
      {
         Parent menuItemContainer = owner.getContent().getParent();

         if (inputEvent instanceof MouseEvent)
         {
            MouseEvent mouseEvent = (MouseEvent) inputEvent;

            if (mouseEvent.getEventType() == MouseEvent.MOUSE_PRESSED)
            {
               editMode.set(true);
               textField.requestFocus();
               // Disable the auto request focus (see ContextMenuContent line 1213) causing to lose focus for the text input field.
               menuItemContainer.addEventFilter(MouseEvent.MOUSE_ENTERED, eventConsumer);
            }

            if (mouseEvent.getEventType() == MouseEvent.MOUSE_EXITED)
            {
               if (mouseEvent.getPickResult().getIntersectedNode() != textField && !isAncestor(mouseEvent.getPickResult().getIntersectedNode(), textField))
               {
                  editMode.set(false);
                  textField.cancelEdit();
               }
            }

         }

         if (inputEvent.getEventType() == KeyEvent.KEY_PRESSED)
         {
            KeyEvent keyEvent = (KeyEvent) inputEvent;
            if (keyEvent.getCode() == KeyCode.ESCAPE)
            {
               editMode.set(false);
               textField.cancelEdit();
               keyEvent.consume();
            }
            else if (keyEvent.getCode() == KeyCode.ENTER)
            {
               editMode.set(false);
               textField.commitValue();
               menuItemContainer.requestFocus();
               keyEvent.consume();
            }
         }

         if (inputEvent instanceof MouseEvent && editMode.get())
            inputEvent.consume();

         if (!editMode.get())
            menuItemContainer.removeEventFilter(MouseEvent.MOUSE_ENTERED, eventConsumer);
      });
   }

   private static boolean isAncestor(Node query, Node ancestor)
   {
      if (query == null)
         return false;

      Node parent = query.getParent();

      if (parent == ancestor)
         return true;

      return isAncestor(parent, ancestor);
   }

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

package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import javafx.scene.control.ListCell;
import javafx.scene.layout.Pane;

public class ControllerListCell<T extends UIElement> extends ListCell<T>
{
   public ControllerListCell()
   {
   }

   @Override
   protected void updateItem(T item, boolean empty)
   {
      super.updateItem(item, empty);

      setText(null);
      if (empty)
      {
         setGraphic(null);
      }
      else
      {
         Pane mainPane = item.getMainPane();
         setGraphic(mainPane);
         // Somehow the width of the cell is too large...
         mainPane.prefWidthProperty().bind(widthProperty().subtract(20));
      }
   }
}

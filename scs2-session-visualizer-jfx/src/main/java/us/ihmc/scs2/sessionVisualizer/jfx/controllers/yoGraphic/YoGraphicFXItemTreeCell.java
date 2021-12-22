package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic;

import java.util.List;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIcon;
import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.control.cell.CheckBoxTreeCell;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.HBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX2D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

public class YoGraphicFXItemTreeCell extends CheckBoxTreeCell<YoGraphicFXItem>
{
   // TODO Make the cell editable so the user can change the name of the item.
   private final YoGroupFX rootGroup;

   public YoGraphicFXItemTreeCell(YoGroupFX rootGroup)
   {
      this.rootGroup = rootGroup;
      setOnDragOver(this::handleDragOver);
      setOnDragDropped(this::handleDragDropped);
      setOnDragEntered(this::handleDragEntered);
      setOnDragExited(this::handleDragExited);
   }

   @Override
   public void updateItem(YoGraphicFXItem item, boolean empty)
   {
      textProperty().unbind();

      super.updateItem(item, empty);

      if (empty)
      {
         setText(null);
         setGraphic(null);
      }
      else
      {
         setText(item.getName());
         textProperty().bind(item.nameProperty());

         Node graphic = null;

         if (item instanceof YoGroupFX)
         {
            graphic = new FontAwesomeIconView(FontAwesomeIcon.FOLDER_OPEN_ALT);
         }
         else if (item instanceof YoGraphicFX2D)
         {
            Label icon = new Label("2D");
            icon.setFont(Font.font("Century Schoolbook", 10.0));
            icon.setTextFill(Color.DARKGREEN);
            graphic = icon;
         }
         else if (item instanceof YoGraphicFX3D)
         {
            Label icon = new Label("3D");
            icon.setFont(Font.font("Century Schoolbook", 10.0));
            icon.setTextFill(Color.DARKRED);
            graphic = icon;
         }
         if (getGraphic() != null)
         {
            HBox container = new HBox(5, getGraphic(), graphic);
            container.setAlignment(Pos.CENTER);
            setGraphic(container);
         }
         else
         {
            setGraphic(graphic);
         }
      }
   }

   public void handleDragEntered(DragEvent event)
   {
      if (!event.isAccepted() && acceptDragEventForDrop(event))
         setSelectionHighlight(true);
      event.consume();
   }

   public void handleDragExited(DragEvent event)
   {
      if (acceptDragEventForDrop(event))
         setSelectionHighlight(false);
      event.consume();
   }

   public void handleDragOver(DragEvent event)
   {
      if (!event.isAccepted() && acceptDragEventForDrop(event))
         event.acceptTransferModes(TransferMode.ANY);
      event.consume();
   }

   public void handleDragDropped(DragEvent event)
   {
      if (!event.isAccepted())
         return;

      Dragboard db = event.getDragboard();
      boolean success = false;

      if (db.hasContent(DragAndDropTools.YO_GRAPHIC_ITEMS_REFERENCE))
      {
         List<YoGraphicFXItem> items = DragAndDropTools.retrieveYoGraphicFXItemsFromDragBoard(db, rootGroup);
         if (items != null && getItem() instanceof YoGroupFX)
         {
            YoGroupFX newParent = (YoGroupFX) getItem();
            items.forEach(newParent::addYoGraphicFXItem);

            setSelectionHighlight(false);
            success = true;
         }
      }
      event.setDropCompleted(success);
      event.consume();
   }

   public void setSelectionHighlight(boolean isSelected)
   {
      if (isSelected)
         setStyle("-fx-border-color:green; -fx-border-radius:5;");
      else
         setStyle("-fx-border-color: null;");
   }

   private boolean acceptDragEventForDrop(DragEvent event)
   {
      if (event.getGestureSource() == this || !(getItem() instanceof YoGroupFX))
         return false;
      List<YoGraphicFXItem> items = DragAndDropTools.retrieveYoGraphicFXItemsFromDragBoard(event.getDragboard(), rootGroup);
      if (items == null)
         return false;
      return !items.stream().filter(item -> item == getItem()).findFirst().isPresent();
   }
}

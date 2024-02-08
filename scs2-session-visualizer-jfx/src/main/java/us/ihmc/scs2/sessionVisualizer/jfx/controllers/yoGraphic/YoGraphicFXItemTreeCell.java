package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic;

import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.control.cell.CheckBoxTreeCell;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.HBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import org.kordamp.ikonli.javafx.FontIcon;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX2D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

import java.io.InputStream;
import java.util.List;

public class YoGraphicFXItemTreeCell extends CheckBoxTreeCell<YoGraphicFXItem>
{
   private static final boolean ADD_YO_GRAPHIC_ICONS = false;

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
            graphic = new FontIcon("fa-folder-open-o");
         }
         else if (item instanceof YoGraphicFXItem)
         {
            boolean is2D = item instanceof YoGraphicFX2D;
            Label iconLabel = new Label(is2D ? "2D" : "3D");
            iconLabel.setFont(Font.font("Century Schoolbook", 10.0));
            iconLabel.setTextFill(is2D ? Color.DARKGREEN : Color.DARKRED);

            if (ADD_YO_GRAPHIC_ICONS)
            {
               InputStream resource = SessionVisualizerIOTools.getYoGraphicFXIconResource(item.getClass());
               ImageView iconImage = new ImageView(new Image(resource, 0, 25, true, true));
               iconImage.setPreserveRatio(true);
               iconImage.setFitHeight(20.0);
               graphic = new HBox(5, iconLabel, iconImage);
            }
            else
            {
               graphic = iconLabel;
            }
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

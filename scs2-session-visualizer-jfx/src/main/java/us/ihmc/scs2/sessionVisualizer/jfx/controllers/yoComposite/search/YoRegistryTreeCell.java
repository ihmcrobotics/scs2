package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import javafx.scene.control.TreeCell;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoRegistryTreeCell extends TreeCell<YoRegistry>
{
   public YoRegistryTreeCell()
   {
      getStyleClass().add("yo-variable-registry-list-cell");
   }

   @Override
   protected void updateItem(YoRegistry registry, boolean empty)
   {
      super.updateItem(registry, empty);

      if (empty)
      {
         setText(null);
      }
      else
      {
         setText(registry.getName());
      }
   }
}

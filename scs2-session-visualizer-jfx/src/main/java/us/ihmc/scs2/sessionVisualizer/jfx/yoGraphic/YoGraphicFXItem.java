package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.StringProperty;
import javafx.collections.ObservableSet;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;

public interface YoGraphicFXItem
{
   public static final String YO_GRAPHICFX_ITEM_KEY = YoGraphicFXItem.class.getSimpleName();

   StringProperty nameProperty();

   default void setName(String name)
   {
      nameProperty().set(name);
   }

   default String getName()
   {
      return nameProperty() == null ? null : nameProperty().get();
   }

   default String getNamespace()
   {
      return getParentGroup() == null ? "" : getParentGroup().getFullname();
   }

   BooleanProperty visibleProperty();

   default void setVisible(boolean visible)
   {
      visibleProperty().set(visible);
   }

   default boolean isVisible()
   {
      return visibleProperty().get();
   }

   ObservableSet<YoGraphicFXItem> getItemChildren();

   default YoGroupFX getRootGroup()
   {
      if (this instanceof YoGroupFX)
         return this.getRootGroup();
      else if (getParentGroup() == null)
         return null;
      else
         return getParentGroup().getRootGroup();
   }

   default YoGroupFX getParentGroup()
   {
      return parentGroupProperty() == null ? null : parentGroupProperty().get();
   }

   ObjectProperty<YoGroupFX> parentGroupProperty();

   void detachFromParent();

   void clear();

   YoGraphicFXItem clone();

   default String getFullname()
   {
      return getParentGroup() == null ? getName() : getParentGroup().getFullname() + YoGraphicDefinition.SEPARATOR + getName();
   }

   void render();

   default void computeBackground()
   {
      // Override to perform heavier computation on a background thread.
   }
}

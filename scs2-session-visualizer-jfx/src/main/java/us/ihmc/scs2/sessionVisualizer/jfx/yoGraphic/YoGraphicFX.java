package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableSet;
import javafx.scene.Node;

public abstract class YoGraphicFX implements YoGraphicFXItem
{
   private final StringProperty nameProperty = new SimpleStringProperty(this, "name", null);
   private final ObjectProperty<YoGroupFX> parentGroupProperty = new SimpleObjectProperty<>(this, "parent", null);

   public YoGraphicFX()
   {
   }

   @Override
   public abstract YoGraphicFX clone();

   public abstract Node getNode();

   @Override
   public final BooleanProperty visibleProperty()
   {
      return getNode().visibleProperty();
   }

   @Override
   public final void setName(String name)
   {
      nameProperty.set(name);
   }

   @Override
   public final StringProperty nameProperty()
   {
      return nameProperty;
   }

   // TODO Add change listener to the parent group property
   @Override
   public final ObjectProperty<YoGroupFX> parentGroupProperty()
   {
      return parentGroupProperty;
   }

   @Override
   public final ObservableSet<YoGraphicFXItem> getItemChildren()
   {
      return FXCollections.emptyObservableSet();
   }
}

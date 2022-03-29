package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.LinkedHashSet;
import java.util.Objects;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.binding.Bindings;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableBooleanValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableSet;
import javafx.collections.SetChangeListener;
import javafx.scene.Group;
import javafx.scene.Node;

public class YoGroupFX implements YoGraphicFXItem
{
   private final StringProperty nameProperty = new SimpleStringProperty(this, "name", null);
   private final ObjectProperty<YoGroupFX> parentGroupProperty;
   private final ObservableSet<YoGroupFX> children = FXCollections.synchronizedObservableSet(FXCollections.observableSet(new LinkedHashSet<>()));
   private final ObservableSet<YoGraphicFXItem> itemChildren = FXCollections.synchronizedObservableSet(FXCollections.observableSet(new LinkedHashSet<>()));
   private final ObservableSet<YoGraphicFX2D> yoGraphicFX2DSet = FXCollections.synchronizedObservableSet(FXCollections.observableSet(new LinkedHashSet<>()));
   private final ObservableSet<YoGraphicFX3D> yoGraphicFX3DSet = FXCollections.synchronizedObservableSet(FXCollections.observableSet(new LinkedHashSet<>()));

   private final Group group2D = new Group();
   private final Group group3D = new Group();

   private final BooleanProperty visibleProperty = new SimpleBooleanProperty(this, "visible", true);

   public static YoGroupFX createGUIRoot()
   {
      return new YoGroupFX(YoGraphicTools.GUI_ROOT_NAME, true);
   }

   public static YoGroupFX createSessionRoot()
   {
      return new YoGroupFX(YoGraphicTools.SESSION_ROOT_NAME, true);
   }

   private YoGroupFX(String name, boolean isRoot)
   {
      if (isRoot)
      {
         nameProperty.set(name);
         parentGroupProperty = null;

         setupChildrenListener();
         setupYoGraphicFXsListener(yoGraphicFX2DSet, group2D);
         setupYoGraphicFXsListener(yoGraphicFX3DSet, group3D);
      }
      else
      {
         Objects.requireNonNull(name);
         nameProperty.set(name);

         parentGroupProperty = new SimpleObjectProperty<>(this, "parent", null);
         parentGroupProperty.addListener((observable, oldValue, newValue) ->
         {
            if (oldValue == null || newValue == oldValue)
               return;

            if (oldValue != null)
               oldValue.children.remove(YoGroupFX.this);

            if (newValue != null && !newValue.children.contains(this))
               newValue.children.add(this);
         });

         setupChildrenListener();
         setupYoGraphicFXsListener(yoGraphicFX2DSet, group2D);
         setupYoGraphicFXsListener(yoGraphicFX3DSet, group3D);

         setupVisibilityBindings();
         group2D.visibleProperty().bind(visibleProperty);
         group3D.visibleProperty().bind(visibleProperty);
      }
   }

   public YoGroupFX(String name)
   {
      this(name, false);
   }

   private void setupVisibilityBindings()
   {
      MutableBoolean updating = new MutableBoolean(false);

      visibleProperty.addListener((o, oldValue, newValue) ->
      {
         if (updating.isTrue())
            return;

         updating.setTrue();
         itemChildren.forEach(child -> child.setVisible(newValue));
         updating.setFalse();
      });

      itemChildren.addListener(new SetChangeListener<YoGraphicFXItem>()
      {
         ObservableBooleanValue observable = null;
         ChangeListener<? super Boolean> changeListener = (o, oldValue, newValue) -> updateVisibility();

         @Override
         public void onChanged(Change<? extends YoGraphicFXItem> change)
         {
            if (observable != null)
               observable.removeListener(changeListener);

            if (change.getSet().isEmpty())
            {
               observable = null;
               updateVisibility();
               return;
            }

            for (YoGraphicFXItem itemChild : change.getSet())
            {
               if (observable == null)
                  observable = itemChild.visibleProperty();
               else
                  observable = Bindings.or(observable, itemChild.visibleProperty());
            }

            observable.addListener(changeListener);
            updateVisibility();
         }

         private void updateVisibility()
         {
            if (updating.isTrue())
               return;

            updating.setTrue();
            visibleProperty.set(observable == null ? false : observable.get());
            updating.setFalse();
         }
      });
   }

   private void setupChildrenListener()
   {
      children.addListener((SetChangeListener<YoGroupFX>) change ->
      {
         YoGroupFX elementAdded = change.getElementAdded();
         YoGroupFX elementRemoved = change.getElementRemoved();

         if (change.wasAdded() && elementAdded.getParentGroup() != this)
         {
            if (elementAdded == this)
               throw new IllegalArgumentException("Cannot add a group to its own children, group name: " + elementAdded.getFullname());
            elementAdded.detachFromParent();
            elementAdded.parentGroupProperty().set(this);
            group3D.getChildren().add(elementAdded.getNode3D());
            group2D.getChildren().add(elementAdded.getNode2D());
            itemChildren.add(elementAdded);
         }

         if (change.wasRemoved() && elementRemoved.getParentGroup() == this)
         {
            elementRemoved.parentGroupProperty.set(null);
            group3D.getChildren().remove(elementRemoved.getNode3D());
            group2D.getChildren().remove(elementRemoved.getNode2D());
            itemChildren.remove(elementRemoved);
         }
      });
   }

   private void setupYoGraphicFXsListener(ObservableSet<? extends YoGraphicFX> yoGraphicFXSet, Group groupToRegisterGraphicFX)
   {
      yoGraphicFXSet.addListener((SetChangeListener<YoGraphicFX>) change ->
      {
         YoGraphicFX elementAdded = change.getElementAdded();
         YoGraphicFX elementRemoved = change.getElementRemoved();

         if (change.wasAdded() && elementAdded.getParentGroup() != this)
         {
            long numberOfGraphicWithSameName = yoGraphicFXSet.stream().filter(graphic -> graphic.getName().equals(elementAdded.getName())).count();

            if (numberOfGraphicWithSameName > 1)
            {
               throw new IllegalArgumentException(elementAdded.getClass().getSimpleName() + " should have a unique name within a group, graphic causing issue: "
                     + elementAdded.getFullname());
            }

            elementAdded.detachFromParent();
            elementAdded.parentGroupProperty().set(this);
            groupToRegisterGraphicFX.getChildren().add(elementAdded.getNode());
            itemChildren.add(elementAdded);
         }

         if (change.wasRemoved() && elementRemoved.getParentGroup() == this)
         {
            elementRemoved.parentGroupProperty().set(null);
            groupToRegisterGraphicFX.getChildren().remove(elementRemoved.getNode());
            itemChildren.remove(elementRemoved);
         }
      });
   }

   @Override
   public void render()
   {
      yoGraphicFX2DSet.forEach(YoGraphicFX2D::render);
      yoGraphicFX3DSet.forEach(YoGraphicFX3D::render);
      children.forEach(YoGroupFX::render);
   }

   @Override
   public void computeBackground()
   {
      yoGraphicFX2DSet.forEach(YoGraphicFX2D::computeBackground);
      yoGraphicFX3DSet.forEach(YoGraphicFX3D::computeBackground);
      children.forEach(YoGroupFX::computeBackground);
   }

   @Override
   public void clear()
   {
      children.forEach(child -> child.clear());
      yoGraphicFX2DSet.forEach(graphic -> graphic.clear());
      yoGraphicFX3DSet.forEach(graphic -> graphic.clear());
      children.clear();
      yoGraphicFX2DSet.clear();
      yoGraphicFX3DSet.clear();
   }

   /**
    * All graphic items registered to this group including {@link YoGroupFX}, {@link YoGraphicFX2D},
    * and {@link YoGraphicFX3D}.
    * <p>
    * Should be accessed for read-only operations.
    * </p>
    */
   @Override
   public ObservableSet<YoGraphicFXItem> getItemChildren()
   {
      return itemChildren;
   }

   public ObservableSet<YoGraphicFX2D> getYoGraphicFX2DSet()
   {
      return yoGraphicFX2DSet;
   }

   public boolean addYoGraphicFXItem(YoGraphicFXItem yoGraphicFXItem)
   {
      if (yoGraphicFXItem instanceof YoGraphicFX2D)
         return addYoGraphicFX2D((YoGraphicFX2D) yoGraphicFXItem);
      else if (yoGraphicFXItem instanceof YoGraphicFX3D)
         return addYoGraphicFX3D((YoGraphicFX3D) yoGraphicFXItem);
      else if (yoGraphicFXItem instanceof YoGroupFX)
         return addChild((YoGroupFX) yoGraphicFXItem);
      else
         throw new RuntimeException("Unexpected item type: " + yoGraphicFXItem.getClass().getSimpleName());
   }

   public boolean addYoGraphicFX2D(YoGraphicFX2D yoGraphicFX)
   {
      if (containsYoGraphicFX2D(yoGraphicFX.getName()))
         return false;
      return yoGraphicFX2DSet.add(yoGraphicFX);
   }

   public boolean removeYoGraphicFX2D(YoGraphicFX2D yoGraphicFX)
   {
      return yoGraphicFX2DSet.remove(yoGraphicFX);
   }

   public boolean containsYoGraphicFX2D(String graphicName)
   {
      return yoGraphicFX2DSet.stream().filter(graphic -> graphic.getName().equals(graphicName)).findFirst().isPresent();
   }

   public YoGraphicFX2D getYoGraphicFX2D(String graphicName)
   {
      return yoGraphicFX2DSet.stream().filter(graphic -> graphic.getName().equals(graphicName)).findFirst().orElse(null);
   }

   public ObservableSet<YoGraphicFX3D> getYoGraphicFX3DSet()
   {
      return yoGraphicFX3DSet;
   }

   public boolean addYoGraphicFX3D(YoGraphicFX3D yoGraphicFX)
   {
      if (containsYoGraphicFX3D(yoGraphicFX.getName()))
         return false;
      return yoGraphicFX3DSet.add(yoGraphicFX);
   }

   public boolean removeYoGraphicFX3D(YoGraphicFX3D yoGraphicFX)
   {
      return yoGraphicFX3DSet.remove(yoGraphicFX);
   }

   public boolean containsYoGraphicFX3D(String graphicName)
   {
      return yoGraphicFX3DSet.stream().filter(graphic -> graphic.getName().equals(graphicName)).findFirst().isPresent();
   }

   public YoGraphicFX3D getYoGraphicFX3D(String graphicName)
   {
      return yoGraphicFX3DSet.stream().filter(graphic -> graphic.getName().equals(graphicName)).findFirst().orElse(null);
   }

   @Override
   public ObjectProperty<YoGroupFX> parentGroupProperty()
   {
      return parentGroupProperty;
   }

   public void setParent(YoGroupFX yoGraphicFXGroup)
   {
      if (isRoot())
         throw new UnsupportedOperationException("Cannot set the parent to the root group.");

      parentGroupProperty.set(yoGraphicFXGroup);
   }

   @Override
   public void detachFromParent()
   {
      if (isRoot())
         return;

      YoGroupFX parentGroup = getParentGroup();

      if (parentGroup != null)
      {
         parentGroup.removeChild(this);
         parentGroupProperty.set(null);
      }
   }

   public ObservableSet<YoGroupFX> getChildren()
   {
      return children;
   }

   public boolean addChild(YoGroupFX yoGraphicFXGroup)
   {
      if (containsChild(yoGraphicFXGroup.getName()))
         return false;
      return children.add(yoGraphicFXGroup);
   }

   public boolean removeChild(YoGroupFX yoGraphicFXGroup)
   {
      return children.remove(yoGraphicFXGroup);
   }

   public YoGroupFX getChild(String childName)
   {
      return children.stream().filter(child -> child.getName().equals(childName)).findFirst().orElse(null);
   }

   public boolean containsChild(String childName)
   {
      return children.stream().filter(child -> childName.equals(child.getName())).findFirst().isPresent();
   }

   @Override
   public YoGroupFX clone()
   {
      YoGroupFX clone = new YoGroupFX(nameProperty.get());
      return clone;
   }

   @Override
   public StringProperty nameProperty()
   {
      return nameProperty;
   }

   @Override
   public BooleanProperty visibleProperty()
   {
      return visibleProperty;
   }

   public boolean isRoot()
   {
      return parentGroupProperty == null;
   }

   @Override
   public YoGroupFX getRootGroup()
   {
      if (isRoot() || parentGroupProperty.get() == null)
         return this;
      else
         return parentGroupProperty.get().getRootGroup();
   }

   public Node getNode2D()
   {
      return group2D;
   }

   public Node getNode3D()
   {
      return group3D;
   }
}

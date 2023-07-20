package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import java.util.Arrays;
import java.util.stream.DoubleStream;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class CompositeProperty implements ReferenceFrameHolder
{
   protected final String type;
   protected final String[] componentIdentifiers;
   protected DoubleProperty[] componentValueProperties;
   protected Property<ReferenceFrame> referenceFrameProperty;

   public CompositeProperty(String type, String[] componentIdentifiers)
   {
      this.type = type;
      this.componentIdentifiers = componentIdentifiers;
   }

   public CompositeProperty(String type, String[] componentIdentifiers, double... componentValues)
   {
      this(type, componentIdentifiers, null, componentValues);
   }

   public CompositeProperty(String type, String[] componentIdentifiers, ReferenceFrame referenceFrame, double... componentValues)
   {
      this(type, componentIdentifiers);
      set(referenceFrame, componentValues);
   }

   public CompositeProperty(String type, String[] componentIdentifiers, DoubleProperty... componentValueProperties)
   {
      this(type, componentIdentifiers, null, componentValueProperties);
   }

   public CompositeProperty(String type,
                            String[] componentIdentifiers,
                            Property<ReferenceFrame> referenceFrameProperty,
                            DoubleProperty... componentValueProperties)
   {
      this(type, componentIdentifiers);
      set(referenceFrameProperty, componentValueProperties);
   }

   public CompositeProperty(CompositeProperty other)
   {
      type = other.type;
      componentIdentifiers = other.componentIdentifiers;
      set(other);
   }

   public void set(CompositeProperty other)
   {
      if (!type.equals(other.type))
         throw new IllegalArgumentException("Incompatible composite type: this=" + type + ", other=" + other.type);
      if (!Arrays.deepEquals(componentIdentifiers, other.componentIdentifiers))
         throw new IllegalArgumentException("Incompatible composite identifiers: this=" + Arrays.toString(componentIdentifiers) + ", other="
               + Arrays.toString(componentIdentifiers));

      componentValueProperties = Arrays.copyOf(other.componentValueProperties, other.componentValueProperties.length);
      referenceFrameProperty = other.referenceFrameProperty;
   }

   public final void set(ReferenceFrame referenceFrame, double... componentValues)
   {
      setComponentValues(componentValues);
      setReferenceFrame(referenceFrame);
   }

   public final void set(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty... componentValueProperties)
   {
      setComponentValueProperties(componentValueProperties);
      setReferenceFrameProperty(referenceFrameProperty);
   }

   public final void setComponentValues(double... componentValues)
   {
      setComponentValueProperties(DoubleStream.of(componentValues).mapToObj(SimpleDoubleProperty::new).toArray(DoubleProperty[]::new));
   }

   public final void setComponentValueProperties(DoubleProperty... componentValueProperties)
   {
      this.componentValueProperties = componentValueProperties;
   }

   public final void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      setReferenceFrameProperty(new SimpleObjectProperty<>(referenceFrame));
   }

   public final void setReferenceFrameProperty(Property<ReferenceFrame> referenceFrameProperty)
   {
      this.referenceFrameProperty = referenceFrameProperty;
   }

   public final String getType()
   {
      return type;
   }

   public final String[] getComponentIdentifiers()
   {
      return componentIdentifiers;
   }

   public final DoubleProperty[] componentValueProperties()
   {
      return componentValueProperties;
   }

   public final Property<ReferenceFrame> referenceFrameProperty()
   {
      return referenceFrameProperty;
   }

   @Override
   public final ReferenceFrame getReferenceFrame()
   {
      return referenceFrameProperty().getValue();
   }

   @Override
   public CompositeProperty clone()
   {
      return new CompositeProperty(this);
   }

   public Point2D toWorld2D(Point2DReadOnly transformable)
   {
      Point2D result = new Point2D(transformable);

      if (referenceFrameProperty() == null || getReferenceFrame() == null || getReferenceFrame().isRootFrame())
         return result;

      getReferenceFrame().getTransformToRoot().transform(result, false);
      return result;
   }

   public Vector2D toWorld2D(Vector2DReadOnly transformable)
   {
      Vector2D result = new Vector2D(transformable);

      if (referenceFrameProperty() == null || getReferenceFrame() == null || getReferenceFrame().isRootFrame())
         return result;

      getReferenceFrame().getTransformToRoot().transform(result, false);
      return result;
   }

   public <T extends Transformable> T toWorld(T transformable)
   {
      if (referenceFrameProperty() == null || getReferenceFrame() == null || getReferenceFrame().isRootFrame())
         return transformable;

      getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame().getRootFrame(), transformable);
      return transformable;
   }

   @Override
   public final String toString()
   {
      String description = "[" + type;
      for (int i = 0; i < componentIdentifiers.length; i++)
         description += ", " + componentIdentifiers[i] + ": " + componentValueProperties[i];
      description += ", frame: " + referenceFrameProperty + "]";
      return description;
   }
}

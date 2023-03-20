package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import static us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition.YoTuple2D;
import static us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition.YoTuple2DIdentifiers;

import java.util.Objects;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class Tuple2DProperty extends CompositeProperty implements FrameTuple2DReadOnly
{
   public Tuple2DProperty()
   {
      super(YoTuple2D, YoTuple2DIdentifiers);
   }

   public Tuple2DProperty(double x, double y)
   {
      super(YoTuple2D, YoTuple2DIdentifiers, x, y);
   }

   public Tuple2DProperty(ReferenceFrame referenceFrame, double x, double y)
   {
      super(YoTuple2D, YoTuple2DIdentifiers, referenceFrame, x, y);
   }

   public Tuple2DProperty(DoubleProperty[] xyProperties)
   {
      super(YoTuple2D, YoTuple2DIdentifiers, xyProperties);
   }

   public Tuple2DProperty(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty[] xyProperties)
   {
      super(YoTuple2D, YoTuple2DIdentifiers, referenceFrameProperty, xyProperties);
   }

   public Tuple2DProperty(DoubleProperty xProperty, DoubleProperty yProperty)
   {
      super(YoTuple2D, YoTuple2DIdentifiers, xProperty, yProperty);
   }

   public Tuple2DProperty(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty xProperty, DoubleProperty yProperty)
   {
      super(YoTuple2D, YoTuple2DIdentifiers, referenceFrameProperty, xProperty, yProperty);
   }

   public Tuple2DProperty(CompositeProperty other)
   {
      this();
      set(other);
   }

   public void setX(double x)
   {
      setXProperty(new SimpleDoubleProperty(x));
   }

   public void setXProperty(DoubleProperty xProperty)
   {
      componentValueProperties[0] = xProperty;
   }

   public void setY(double y)
   {
      setYProperty(new SimpleDoubleProperty(y));
   }

   public void setYProperty(DoubleProperty yProperty)
   {
      componentValueProperties[1] = yProperty;
   }

   public void set(double x, double y)
   {
      setComponentValues(x, y);
   }

   public void set(ReferenceFrame referenceFrame, double x, double y)
   {
      set(referenceFrame, x, y);
   }

   public void set(DoubleProperty xProperty, DoubleProperty yProperty)
   {
      setComponentValueProperties(xProperty, yProperty);
   }

   public void set(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty xProperty, DoubleProperty yProperty)
   {
      set(referenceFrameProperty, xProperty, yProperty);
   }

   @Override
   public void set(CompositeProperty other)
   {
      Objects.requireNonNull(other.getType());
      if (!getType().equals(other.getType()))
         throw new IllegalArgumentException("Cannot set a " + getClass().getSimpleName() + " to a " + other.getType());
      super.set(other);
   }

   public void set(Tuple2DProperty other)
   {
      set((CompositeProperty) other);
   }

   @Override
   public double getX()
   {
      return xProperty().get();
   }

   public DoubleProperty xProperty()
   {
      return componentValueProperties[0];
   }

   @Override
   public double getY()
   {
      return yProperty().get();
   }

   public DoubleProperty yProperty()
   {
      return componentValueProperties[1];
   }

   public Point2D toPoint2DInWorld()
   {
      return toWorld2D(new Point2D(this));
   }

   public Vector2D toVector2DInWorld()
   {
      return toWorld2D(new Vector2D(this));
   }

   @Override
   public Tuple2DProperty clone()
   {
      return new Tuple2DProperty(this);
   }

   @Override
   public boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      return epsilonEquals(geometry, epsilon);
   }
}
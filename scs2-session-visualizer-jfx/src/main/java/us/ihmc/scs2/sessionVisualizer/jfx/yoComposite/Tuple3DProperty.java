package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import static us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition.YoTuple3D;
import static us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition.YoTuple3DIdentifiers;

import java.util.Objects;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Tuple3DProperty extends CompositeProperty implements FrameTuple3DReadOnly
{
   public Tuple3DProperty()
   {
      super(YoTuple3D, YoTuple3DIdentifiers);
   }

   public Tuple3DProperty(double x, double y, double z)
   {
      super(YoTuple3D, YoTuple3DIdentifiers, x, y, z);
   }

   public Tuple3DProperty(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(YoTuple3D, YoTuple3DIdentifiers, referenceFrame, x, y, z);
   }

   public Tuple3DProperty(DoubleProperty[] xyzProperties)
   {
      super(YoTuple3D, YoTuple3DIdentifiers, xyzProperties);
   }

   public Tuple3DProperty(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty[] xyzProperties)
   {
      super(YoTuple3D, YoTuple3DIdentifiers, referenceFrameProperty, xyzProperties);
   }

   public Tuple3DProperty(DoubleProperty xProperty, DoubleProperty yProperty, DoubleProperty zProperty)
   {
      super(YoTuple3D, YoTuple3DIdentifiers, xProperty, yProperty, zProperty);
   }

   public Tuple3DProperty(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty xProperty, DoubleProperty yProperty, DoubleProperty zProperty)
   {
      super(YoTuple3D, YoTuple3DIdentifiers, referenceFrameProperty, xProperty, yProperty, zProperty);
   }

   public Tuple3DProperty(CompositeProperty other)
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

   public void setZ(double z)
   {
      setZProperty(new SimpleDoubleProperty(z));
   }

   public void setZProperty(DoubleProperty zProperty)
   {
      componentValueProperties[2] = zProperty;
   }

   public void set(double x, double y, double z)
   {
      setComponentValues(x, y, z);
   }

   public void set(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      set(referenceFrame, x, y, z);
   }

   public void set(DoubleProperty xProperty, DoubleProperty yProperty, DoubleProperty zProperty)
   {
      setComponentValueProperties(xProperty, yProperty, zProperty);
   }

   public void set(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty xProperty, DoubleProperty yProperty, DoubleProperty zProperty)
   {
      set(referenceFrameProperty, xProperty, yProperty, zProperty);
   }

   @Override
   public void set(CompositeProperty other)
   {
      Objects.requireNonNull(other.getType());
      if (!getType().equals(other.getType()))
         throw new IllegalArgumentException("Cannot set a " + getClass().getSimpleName() + " to a " + other.getType());
      super.set(other);
   }

   public void set(Tuple3DProperty other)
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

   @Override
   public double getZ()
   {
      return zProperty().get();
   }

   public DoubleProperty zProperty()
   {
      return componentValueProperties[2];
   }

   public Point3D toPoint3DInWorld()
   {
      return toWorld(new Point3D(this));
   }

   public Vector3D toVector3DInWorld()
   {
      return toWorld(new Vector3D(this));
   }

   @Override
   public Tuple3DProperty clone()
   {
      return new Tuple3DProperty(this);
   }

   @Override
   public boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      return epsilonEquals(geometry, epsilon);
   }
}
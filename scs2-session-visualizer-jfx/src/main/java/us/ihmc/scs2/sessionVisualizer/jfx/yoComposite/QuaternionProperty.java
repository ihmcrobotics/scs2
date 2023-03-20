package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import static us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition.YoQuaternion;
import static us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition.YoQuaternionIdentifiers;

import java.util.Objects;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;

public class QuaternionProperty extends Orientation3DProperty implements FrameQuaternionReadOnly
{
   public QuaternionProperty()
   {
      super(YoQuaternion, YoQuaternionIdentifiers);
   }

   public QuaternionProperty(double x, double y, double z, double s)
   {
      super(YoQuaternion, YoQuaternionIdentifiers, x, y, z, s);
   }

   public QuaternionProperty(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      super(YoQuaternion, YoQuaternionIdentifiers, referenceFrame, x, y, z, s);
   }

   public QuaternionProperty(DoubleProperty[] xyzsProperties)
   {
      super(YoQuaternion, YoQuaternionIdentifiers, xyzsProperties);
   }

   public QuaternionProperty(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty[] xyzsProperties)
   {
      super(YoQuaternion, YoQuaternionIdentifiers, referenceFrameProperty, xyzsProperties);
   }

   public QuaternionProperty(DoubleProperty xProperty, DoubleProperty yProperty, DoubleProperty zProperty, DoubleProperty sProperty)
   {
      super(YoQuaternion, YoQuaternionIdentifiers, xProperty, yProperty, zProperty, sProperty);
   }

   public QuaternionProperty(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty xProperty, DoubleProperty yProperty,
                                   DoubleProperty zProperty, DoubleProperty sProperty)
   {
      super(YoQuaternion, YoQuaternionIdentifiers, referenceFrameProperty, xProperty, yProperty, zProperty, sProperty);
   }

   public QuaternionProperty(CompositeProperty other)
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

   public void setS(double s)
   {
      setSProperty(new SimpleDoubleProperty(s));
   }

   public void setSProperty(DoubleProperty sProperty)
   {
      componentValueProperties[3] = sProperty;
   }

   public void set(double x, double y, double z, double s)
   {
      setComponentValues(x, y, z, s);
   }

   public void set(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      set(referenceFrame, x, y, z, s);
   }

   public void set(DoubleProperty xProperty, DoubleProperty yProperty, DoubleProperty zProperty, DoubleProperty sProperty)
   {
      setComponentValueProperties(xProperty, yProperty, zProperty, sProperty);
   }

   public void set(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty xProperty, DoubleProperty yProperty, DoubleProperty zProperty,
                   DoubleProperty sProperty)
   {
      set(referenceFrameProperty, xProperty, yProperty, zProperty, sProperty);
   }

   @Override
   public void set(CompositeProperty other)
   {
      Objects.requireNonNull(other.getType());
      if (!getType().equals(other.getType()))
         throw new IllegalArgumentException("Cannot set a " + getClass().getSimpleName() + " to a " + other.getType());
      super.set(other);
   }

   public void set(QuaternionProperty other)
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

   @Override
   public double getS()
   {
      return sProperty().get();
   }

   public DoubleProperty sProperty()
   {
      return componentValueProperties[3];
   }

   @Override
   public QuaternionProperty clone()
   {
      return new QuaternionProperty(this);
   }
}

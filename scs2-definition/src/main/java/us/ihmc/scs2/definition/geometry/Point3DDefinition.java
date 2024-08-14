package us.ihmc.scs2.definition.geometry;

import java.util.Objects;

import jakarta.xml.bind.annotation.XmlAttribute;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class Point3DDefinition extends GeometryDefinition implements Point3DBasics
{
   private final Point3D position = new Point3D();

   public Point3DDefinition()
   {
   }

   public Point3DDefinition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public Point3DDefinition(Point3DDefinition other)
   {
      setName(other.getName());
      position.set(other.position);
   }

   public Point3D getPosition()
   {
      return position;
   }

   @XmlAttribute
   @Override
   public void setX(double x)
   {
      position.setX(x);
   }

   @XmlAttribute
   @Override
   public void setY(double y)
   {
      position.setY(y);
   }

   @XmlAttribute
   @Override
   public void setZ(double z)
   {
      position.setZ(z);
   }

   @Override
   public double getX()
   {
      return position.getX();
   }

   @Override
   public double getY()
   {
      return position.getY();
   }

   @Override
   public double getZ()
   {
      return position.getZ();
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, position);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      Point3DDefinition other = (Point3DDefinition) object;

      if (!Objects.equals(position, other.position))
         return false;

      return true;
   }

   @Override
   public Point3DDefinition copy()
   {
      return new Point3DDefinition(this);
   }
}

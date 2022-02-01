package us.ihmc.scs2.definition.geometry;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public class Point2DDefinition extends GeometryDefinition implements Point2DBasics
{
   private final Point2D position = new Point2D();

   public Point2DDefinition()
   {
   }

   public Point2DDefinition(Tuple2DReadOnly position)
   {
      this.position.set(position);
   }

   public Point2DDefinition(Point2DDefinition other)
   {
      setName(other.getName());
      position.set(other.position);
   }

   public Point2D getPosition()
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

      Point2DDefinition other = (Point2DDefinition) object;

      if (!Objects.equals(position, other.position))
         return false;

      return true;
   }

   @Override
   public Point2DDefinition copy()
   {
      return new Point2DDefinition(this);
   }
}

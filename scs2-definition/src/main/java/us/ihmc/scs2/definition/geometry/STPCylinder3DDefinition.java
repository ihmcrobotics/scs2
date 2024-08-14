package us.ihmc.scs2.definition.geometry;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

import jakarta.xml.bind.annotation.XmlElement;

public class STPCylinder3DDefinition extends Cylinder3DDefinition
{
   private double minimumMargin, maximumMargin;

   public STPCylinder3DDefinition()
   {
      super();
   }

   public STPCylinder3DDefinition(Cylinder3DDefinition other)
   {
      super(other);
   }

   public STPCylinder3DDefinition(STPCylinder3DDefinition other)
   {
      super(other);
      setMargins(other.minimumMargin, other.maximumMargin);
   }

   public STPCylinder3DDefinition(double length, double radius, boolean centered, int resolution)
   {
      super(length, radius, centered, resolution);
   }

   public STPCylinder3DDefinition(double length, double radius, boolean centered)
   {
      super(length, radius, centered);
   }

   public STPCylinder3DDefinition(double length, double radius, int resolution)
   {
      super(length, radius, resolution);
   }

   public STPCylinder3DDefinition(double length, double radius)
   {
      super(length, radius);
   }

   public void setMargins(double minimumMargin, double maximumMargin)
   {
      setMinimumMargin(minimumMargin);
      setMaximumMargin(maximumMargin);
   }

   @XmlElement
   public void setMinimumMargin(double minimumMargin)
   {
      this.minimumMargin = minimumMargin;
   }

   @XmlElement
   public void setMaximumMargin(double maximumMargin)
   {
      this.maximumMargin = maximumMargin;
   }

   public double getMinimumMargin()
   {
      return minimumMargin;
   }

   public double getMaximumMargin()
   {
      return maximumMargin;
   }

   @Override
   public STPCylinder3DDefinition copy()
   {
      return new STPCylinder3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, minimumMargin);
      bits = EuclidHashCodeTools.addToHashCode(bits, maximumMargin);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      STPCylinder3DDefinition other = (STPCylinder3DDefinition) object;

      if (!EuclidCoreTools.equals(minimumMargin, other.minimumMargin))
         return false;
      if (!EuclidCoreTools.equals(maximumMargin, other.maximumMargin))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "STP" + super.toString().replace("]", "") + EuclidCoreIOTools.getStringOf(", margins: (", ")]", ", ", minimumMargin, maximumMargin);
   }
}

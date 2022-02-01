package us.ihmc.scs2.definition.geometry;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class STPBox3DDefinition extends Box3DDefinition
{
   private double minimumMargin, maximumMargin;

   public STPBox3DDefinition()
   {
      super();
   }

   public STPBox3DDefinition(Box3DDefinition other)
   {
      super(other);
   }

   public STPBox3DDefinition(STPBox3DDefinition other)
   {
      super(other);
      setMargins(other.minimumMargin, other.maximumMargin);
   }

   public STPBox3DDefinition(double sizeX, double sizeY, double sizeZ, boolean centered)
   {
      super(sizeX, sizeY, sizeZ, centered);
   }

   public STPBox3DDefinition(double sizeX, double sizeY, double sizeZ)
   {
      super(sizeX, sizeY, sizeZ);
   }

   public STPBox3DDefinition(Tuple3DReadOnly size)
   {
      super(size);
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
   public STPBox3DDefinition copy()
   {
      return new STPBox3DDefinition(this);
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

      STPBox3DDefinition other = (STPBox3DDefinition) object;

      if (Double.doubleToLongBits(minimumMargin) != Double.doubleToLongBits(other.minimumMargin))
         return false;
      if (Double.doubleToLongBits(maximumMargin) != Double.doubleToLongBits(other.maximumMargin))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "STP" + super.toString().replace("]", "") + EuclidCoreIOTools.getStringOf(", margins: (", ")]", ", ", minimumMargin, maximumMargin);
   }
}

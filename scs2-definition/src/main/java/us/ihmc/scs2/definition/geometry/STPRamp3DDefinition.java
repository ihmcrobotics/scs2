package us.ihmc.scs2.definition.geometry;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class STPRamp3DDefinition extends Ramp3DDefinition
{
   private double minimumMargin, maximumMargin;

   public STPRamp3DDefinition()
   {
      super();
   }

   public STPRamp3DDefinition(double sizeX, double sizeY, double sizeZ)
   {
      super(sizeX, sizeY, sizeZ);
   }

   public STPRamp3DDefinition(Tuple3DReadOnly size)
   {
      super(size);
   }

   public STPRamp3DDefinition(STPRamp3DDefinition other)
   {
      super(other);
      setMargins(other.minimumMargin, other.maximumMargin);
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
   public STPRamp3DDefinition copy()
   {
      return new STPRamp3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, getSizeX());
      bits = EuclidHashCodeTools.addToHashCode(bits, getSizeY());
      bits = EuclidHashCodeTools.addToHashCode(bits, getSizeZ());
      bits = EuclidHashCodeTools.addToHashCode(bits, minimumMargin);
      bits = EuclidHashCodeTools.addToHashCode(bits, maximumMargin);
      bits = EuclidHashCodeTools.addToHashCode(bits, getName());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof STPRamp3DDefinition)
      {
         STPRamp3DDefinition other = (STPRamp3DDefinition) object;
         if (minimumMargin != other.minimumMargin)
            return false;
         if (maximumMargin != other.maximumMargin)
            return false;
         return super.equals(object);
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "STP" + super.toString().replace("]", "") + EuclidCoreIOTools.getStringOf(", margins: (", ")]", ", ", minimumMargin, maximumMargin);
   }
}

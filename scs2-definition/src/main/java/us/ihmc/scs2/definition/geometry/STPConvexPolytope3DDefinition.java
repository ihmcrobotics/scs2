package us.ihmc.scs2.definition.geometry;

import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class STPConvexPolytope3DDefinition extends ConvexPolytope3DDefinition
{
   private double minimumMargin, maximumMargin;

   public STPConvexPolytope3DDefinition()
   {
      super();
   }

   public STPConvexPolytope3DDefinition(ConvexPolytope3DDefinition other)
   {
      super(other);
   }

   public STPConvexPolytope3DDefinition(STPConvexPolytope3DDefinition other)
   {
      super(other);
      setMargins(other.minimumMargin, other.maximumMargin);
   }

   public STPConvexPolytope3DDefinition(ConvexPolytope3DReadOnly convexPolytope)
   {
      super(convexPolytope);
   }

   public void setMargins(double minimumMargin, double maximumMargin)
   {
      setMinimumMargin(minimumMargin);
      setMaximumMargin(maximumMargin);
   }

   public void setMinimumMargin(double minimumMargin)
   {
      this.minimumMargin = minimumMargin;
   }

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
   public STPConvexPolytope3DDefinition copy()
   {
      return new STPConvexPolytope3DDefinition(this);
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

      STPConvexPolytope3DDefinition other = (STPConvexPolytope3DDefinition) object;

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

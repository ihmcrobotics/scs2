package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class ChartDoubleBounds
{
   private final double lower;
   private final double upper;
   private int hashCode = 0;

   public ChartDoubleBounds(double lower, double upper)
   {
      this.lower = lower;
      this.upper = upper;
   }

   public ChartDoubleBounds(ChartDoubleBounds other)
   {
      this.lower = other.lower;
      this.upper = other.upper;
   }

   public ChartDoubleBounds union(ChartDoubleBounds other)
   {
      return union(this, other);
   }

   public ChartDoubleBounds include(double value)
   {
      return include(this, value);
   }

   public ChartDoubleBounds negate()
   {
      return negate(this);
   }

   public double getLower()
   {
      return lower;
   }

   public double getUpper()
   {
      return upper;
   }

   public double length()
   {
      return upper - lower;
   }

   public boolean isInside(double value)
   {
      return value >= lower && value <= upper;
   }

   @Override
   public int hashCode()
   {
      if (hashCode == 0)
      {
         long hash = EuclidHashCodeTools.addToHashCode(0L, lower);
         hash = EuclidHashCodeTools.addToHashCode(hash, upper);
         hashCode = EuclidHashCodeTools.toIntHashCode(hash);
      }
      return hashCode;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ChartDoubleBounds)
      {
         ChartDoubleBounds other = (ChartDoubleBounds) object;
         return lower == other.lower && upper == other.upper;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "Lower bound: " + lower + ", upper bound: " + upper;
   }

   public static ChartDoubleBounds include(ChartDoubleBounds bounds, double value)
   {
      return new ChartDoubleBounds(Math.min(bounds.lower, value), Math.max(bounds.upper, value));
   }

   public static ChartDoubleBounds union(ChartDoubleBounds bounds1, ChartDoubleBounds bounds2)
   {
      return new ChartDoubleBounds(Math.min(bounds1.lower, bounds2.lower), Math.max(bounds1.upper, bounds2.upper));
   }

   public static ChartDoubleBounds negate(ChartDoubleBounds bounds)
   {
      return new ChartDoubleBounds(-bounds.upper, -bounds.lower);
   }
}

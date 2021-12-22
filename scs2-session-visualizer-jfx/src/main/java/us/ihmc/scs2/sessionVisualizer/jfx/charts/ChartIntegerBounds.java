package us.ihmc.scs2.sessionVisualizer.jfx.charts;

public class ChartIntegerBounds
{
   private final int lower;
   private final int upper;
   private int hashCode = 0;

   public ChartIntegerBounds(int lower, int upper)
   {
      this.lower = lower;
      this.upper = upper;
   }

   public ChartIntegerBounds(ChartIntegerBounds other)
   {
      this.lower = other.lower;
      this.upper = other.upper;
   }

   public ChartIntegerBounds center(int currentIndex, int minIndex, int maxIndex)
   {
      return zoom(currentIndex, 0, minIndex, maxIndex, 1.0);
   }

   public ChartIntegerBounds zoom(int currentIndex, int minLength, int minIndex, int maxIndex, double zoomFactor)
   {
      return zoom(this, currentIndex, minLength, minIndex, maxIndex, zoomFactor);
   }

   public ChartIntegerBounds union(ChartIntegerBounds other)
   {
      return union(this, other);
   }

   public ChartIntegerBounds include(int value)
   {
      return include(this, value);
   }

   public int getLower()
   {
      return lower;
   }

   public int getUpper()
   {
      return upper;
   }

   public int length()
   {
      return upper - lower;
   }

   public boolean isInside(int index)
   {
      return index >= lower && index <= upper;
   }

   @Override
   public int hashCode()
   {
      if (hashCode == 0)
         hashCode = 31 * lower + upper;
      return hashCode;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ChartIntegerBounds)
      {
         ChartIntegerBounds other = (ChartIntegerBounds) object;
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

   public static ChartIntegerBounds include(ChartIntegerBounds bounds, int value)
   {
      return new ChartIntegerBounds(Math.min(bounds.lower, value), Math.max(bounds.upper, value));
   }

   public static ChartIntegerBounds union(ChartIntegerBounds bounds1, ChartIntegerBounds bounds2)
   {
      return new ChartIntegerBounds(Math.min(bounds1.lower, bounds2.lower), Math.max(bounds1.upper, bounds2.upper));
   }

   public static ChartIntegerBounds zoom(ChartIntegerBounds current, int currentIndex, int minLength, int minIndex, int maxIndex, double zoomFactor)
   {
      int oldLength = current.upper - current.lower;
      int newLength = (int) (oldLength / zoomFactor);

      if (newLength < minLength)
         return current;

      int newLower = currentIndex - newLength / 2;
      int newUpper = newLower + newLength;

      if (newLower < minIndex)
      {
         newLower = minIndex;
         newUpper = Math.min(newLower + newLength, maxIndex);
      }
      else if (newUpper > maxIndex)
      {
         newUpper = maxIndex;
         newLower = Math.max(newUpper - newLength, minIndex);
      }

      return new ChartIntegerBounds(newLower, newUpper);
   }
}

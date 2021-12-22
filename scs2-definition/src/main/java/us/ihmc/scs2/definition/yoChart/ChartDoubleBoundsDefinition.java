package us.ihmc.scs2.definition.yoChart;

import javax.xml.bind.annotation.XmlAttribute;

public class ChartDoubleBoundsDefinition
{
   private double lower;
   private double upper;

   public ChartDoubleBoundsDefinition()
   {
   }

   public ChartDoubleBoundsDefinition(double lower, double upper)
   {
      setLower(lower);
      setUpper(upper);
   }

   @XmlAttribute
   public void setLower(double lower)
   {
      this.lower = lower;
   }

   @XmlAttribute
   public void setUpper(double upper)
   {
      this.upper = upper;
   }

   public double getLower()
   {
      return lower;
   }

   public double getUpper()
   {
      return upper;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ChartDoubleBoundsDefinition)
      {
         ChartDoubleBoundsDefinition other = (ChartDoubleBoundsDefinition) object;
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
      return "lower: " + lower + ", upper: " + upper;
   }
}

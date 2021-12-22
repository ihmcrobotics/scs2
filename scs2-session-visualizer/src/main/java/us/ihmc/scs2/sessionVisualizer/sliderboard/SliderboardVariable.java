package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import us.ihmc.commons.MathTools;

public class SliderboardVariable
{
   private final int min, max;
   private final IntegerProperty value = new SimpleIntegerProperty(this, "value", -1);

   public SliderboardVariable(int min, int max)
   {
      this.min = min;
      this.max = max;
   }

   public void setValue(int value)
   {
      this.value.set(value);
   }

   public int getMin()
   {
      return min;
   }

   public int getMax()
   {
      return max;
   }

   public int getValue()
   {
      return value.get();
   }

   public IntegerProperty valueProperty()
   {
      return value;
   }

   @Override
   public String toString()
   {
      return "value: " + getValue() + ", min: " + getMin() + ", max: " + getMax();
   }

   public static int doubleToInt(double valueDouble, double minDouble, double maxDouble, int minInt, int maxInt)
   {
      double percent = (valueDouble - minDouble) / (maxDouble - minDouble);
      percent = MathTools.clamp(percent, 0.0, 1.0);
      return (int) (Math.round(percent * (maxInt - minInt)) + minInt);
   }

   public static double intToDouble(int valueInt, int minInt, int maxInt, double minDouble, double maxDouble)
   {
      double percent = (double) (valueInt - minInt) / (double) (maxInt - minInt);
      percent = MathTools.clamp(percent, 0.0, 1.0);
      return percent * (maxDouble - minDouble) + minDouble;
   }

   public static int longToInt(long valueLong, double minLong, double maxLong, int minInt, int maxInt)
   {
      double percent = (double) (valueLong - minLong) / (double) (maxLong - minLong);
      percent = MathTools.clamp(percent, 0.0, 1.0);
      return (int) (Math.round(percent * (maxInt - minInt)) + minInt);
   }

   public static long intToLong(int valueInt, int minInt, int maxInt, long minLong, long maxLong)
   {
      double percent = (double) (valueInt - minInt) / (double) (maxInt - minInt);
      percent = MathTools.clamp(percent, 0.0, 1.0);
      return (long) (percent * (maxLong - minLong) + minLong);
   }

   public static int booleanToInt(boolean valueBoolean, int minInt, int maxInt)
   {
      return valueBoolean ? maxInt : minInt;
   }

   public static boolean intToBoolean(int valueInt, int minInt, int maxInt)
   {
      double percent = (double) (valueInt - minInt) / (double) (maxInt - minInt);
      return  percent > 0.5 ? true : false;
   }
}

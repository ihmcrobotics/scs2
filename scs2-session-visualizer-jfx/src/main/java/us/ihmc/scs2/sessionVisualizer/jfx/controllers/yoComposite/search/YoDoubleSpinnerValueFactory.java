package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import us.ihmc.javaFXExtensions.control.UnboundedDoubleSpinnerValueFactory;

public class YoDoubleSpinnerValueFactory extends UnboundedDoubleSpinnerValueFactory
{
   static final double SPINNER_STEP_SIZE = 0.1;

   public YoDoubleSpinnerValueFactory(double initialValue)
   {
      super(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, initialValue, SPINNER_STEP_SIZE);
   }

   @Override
   public void decrement(int steps)
   {
      setValue(decrement(getValue(), getMin(), getMax(), isWrapAround(), steps, getAmountToStepBy()));
   }

   @Override
   public void increment(int steps)
   {
      setValue(increment(getValue(), getMin(), getMax(), isWrapAround(), steps, getAmountToStepBy()));
   }

   public static double decrement(double currentValue, double min, double max, boolean wrapAround, int steps, double amountToStepBy)
   {
      double newValue = roundToULP(currentValue - amountToStepBy * steps);

      if (newValue < min)
      {
         if (wrapAround)
            newValue = wrapValue(newValue, min, max);
         else
            newValue = min;
      }

      return newValue;
   }

   public static double increment(double currentValue, double min, double max, boolean wrapAround, int steps, double amountToStepBy)
   {
      double newValue = roundToULP(currentValue + amountToStepBy * steps);

      if (newValue > max)
      {
         if (wrapAround)
            newValue = wrapValue(newValue, min, max);
         else
            newValue = max;
      }

      return newValue;
   }

   /**
    * Rounds the given value to the specified number of decimal places.
    * 
    * @param value     the value to be rounded.
    * @param precision number of digits to the right of the decimal point.
    * @return the rounded value.
    */
   public static double round(double value, long precision)
   {
      if (precision < 0)
         throw new IllegalArgumentException("precision cannot be negative: " + precision);

      double scale = Math.round(Math.pow(10, precision));
      return Math.round(value * scale) / scale;
   }

   /**
    * Rounds the value in an attempt to discard any accumulated numerical error by looking at the
    * value's ulp.
    */
   public static double roundToULP(double value)
   {
      double ulp = Math.ulp(value);
      long precisionToULP = (long) Math.floor(-Math.log10(2.0 * ulp));
      return round(value, precisionToULP);
   }

   /*
    * Convenience method to support wrapping values around their min / max constraints. Used by the
    * SpinnerValueFactory implementations when the Spinner wrapAround property is true.
    */
   static double wrapValue(double value, double min, double max)
   {
      if (max == 0.0)
         throw new RuntimeException();

      if (value < min)
         return max;
      else if (value > max)
         return min;
      else
         return value;
   }
}

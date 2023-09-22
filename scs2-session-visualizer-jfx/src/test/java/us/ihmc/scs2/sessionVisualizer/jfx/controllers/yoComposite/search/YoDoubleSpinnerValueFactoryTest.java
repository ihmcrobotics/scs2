package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search.YoDoubleSpinnerValueFactory.decrement;
import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search.YoDoubleSpinnerValueFactory.increment;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class YoDoubleSpinnerValueFactoryTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testRound()
   {
      Random random = new Random(234245);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int precision = 3;
         double scale = 1000.0;
         double valueRaw = EuclidCoreRandomTools.nextDouble(random, 1000.0);
         double expected = Math.round(valueRaw * scale) / scale;
         double actual = YoDoubleSpinnerValueFactory.round(valueRaw, precision);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int precision = 2;
         double scale = 100.0;
         double valueRaw = EuclidCoreRandomTools.nextDouble(random, 1000.0);
         double expected = Math.round(valueRaw * scale) / scale;
         double actual = YoDoubleSpinnerValueFactory.round(valueRaw, precision);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int precision = 1;
         double scale = 10.0;
         double valueRaw = EuclidCoreRandomTools.nextDouble(random, 1000.0);
         double expected = Math.round(valueRaw * scale) / scale;
         double actual = YoDoubleSpinnerValueFactory.round(valueRaw, precision);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int precision = 0;
         double valueRaw = EuclidCoreRandomTools.nextDouble(random, 1000.0);
         double expected = Math.round(valueRaw);
         double actual = YoDoubleSpinnerValueFactory.round(valueRaw, precision);
         assertEquals(expected, actual);
      }

      {
         int precision = 1;
         double valueRaw = 123456.7890123;
         double expected = 123456.8;
         double actual = YoDoubleSpinnerValueFactory.round(valueRaw, precision);
         assertEquals(expected, actual);
      }

      {
         int precision = 2;
         double valueRaw = 123456.7890123;
         double expected = 123456.79;
         double actual = YoDoubleSpinnerValueFactory.round(valueRaw, precision);
         assertEquals(expected, actual);
      }

      {
         int precision = 3;
         double valueRaw = 123456.7890123;
         double expected = 123456.789;
         double actual = YoDoubleSpinnerValueFactory.round(valueRaw, precision);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testDecrement()
   {
      double currentValue;
      double min = Double.NEGATIVE_INFINITY;
      double max = Double.POSITIVE_INFINITY;
      boolean wrapAround = false;
      int steps;
      double amountToStepBy;

      {
         currentValue = 0.0;
         amountToStepBy = 0.1;
         steps = 1;
         assertEquals(-0.1, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.2, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.3, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.4, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.5, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.6, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.7, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.8, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.9, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.0, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.1, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.2, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.3, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.4, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.5, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.6, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.7, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.8, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.9, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
      }

      {
         currentValue = 100.555;
         amountToStepBy = 0.1;
         steps = 1;
         assertEquals(100.455, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.355, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.255, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.155, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.055, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.955, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.855, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.755, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.655, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.555, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.455, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
      }

      {
         currentValue = -99.355;
         amountToStepBy = 0.1;
         steps = 1;
         assertEquals(-99.455, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.555, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.655, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.755, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.855, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.955, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.055, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.155, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.255, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.355, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.455, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
      }

      {
         currentValue = 1000.355456987563;
         amountToStepBy = 0.1;
         steps = 1;
         assertEquals(1000.255456987563, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(1000.155456987563, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(1000.055456987563, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.955456987563, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.855456987563, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.755456987563, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.655456987563, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.555456987563, currentValue = decrement(currentValue, min, max, wrapAround, steps, amountToStepBy));
      }
   }

   @Test
   public void testIncrement()
   {
      double currentValue;
      double min = Double.NEGATIVE_INFINITY;
      double max = Double.POSITIVE_INFINITY;
      boolean wrapAround = false;
      int steps;
      double amountToStepBy;

      {
         currentValue = -2.0;
         amountToStepBy = 0.1;
         steps = 1;
         assertEquals(-1.9, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.8, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.7, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.6, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.5, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.4, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.3, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.2, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.1, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-1.0, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.9, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.8, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.7, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.6, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.5, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.4, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.3, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.2, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-0.1, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(0.0, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(0.1, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(0.2, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(0.3, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
      }

      {
         currentValue = 99.355;
         amountToStepBy = 0.1;
         steps = 1;
         assertEquals(99.455, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.555, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.655, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.755, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.855, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(99.955, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.055, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.155, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.255, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.355, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(100.455, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
      }

      {
         currentValue = -100.555;
         amountToStepBy = 0.1;
         steps = 1;
         assertEquals(-100.455, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.355, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.255, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.155, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-100.055, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.955, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.855, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.755, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.655, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.555, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(-99.455, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
      }

      {
         currentValue = 999.455456987563;
         amountToStepBy = 0.1;
         steps = 1;
         assertEquals(999.555456987563, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.655456987563, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.755456987563, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.855456987563, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(999.955456987563, currentValue =  increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(1000.055456987563, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(1000.155456987563, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
         assertEquals(1000.255456987563, currentValue = increment(currentValue, min, max, wrapAround, steps, amountToStepBy));
      }
   }
}

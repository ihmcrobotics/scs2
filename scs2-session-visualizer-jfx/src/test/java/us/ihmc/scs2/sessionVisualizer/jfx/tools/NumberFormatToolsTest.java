package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class NumberFormatToolsTest
{

   @Test
   public void testDoubleToString()
   {
      assertEquals("0.0", NumberFormatTools.doubleToString(0.0, 3));
      assertEquals("0.0", NumberFormatTools.doubleToString(-0.0, 3));
      assertEquals("1.000", NumberFormatTools.doubleToString(1.0, 3));
      assertEquals("-1.000", NumberFormatTools.doubleToString(-1.0, 3));
      assertEquals("10.000", NumberFormatTools.doubleToString(10.0, 3));
      assertEquals("-10.000", NumberFormatTools.doubleToString(-10.0, 3));
      assertEquals("100.000", NumberFormatTools.doubleToString(100.0, 3));
      assertEquals("-100.000", NumberFormatTools.doubleToString(-100.0, 3));
      assertEquals("1000.000", NumberFormatTools.doubleToString(1000.0, 3));
      assertEquals("-1000.000", NumberFormatTools.doubleToString(-1000.0, 3));
      assertEquals("1.000e+04", NumberFormatTools.doubleToString(10000.0, 3));
      assertEquals("-1.000e+04", NumberFormatTools.doubleToString(-10000.0, 3));
      assertEquals("0.100", NumberFormatTools.doubleToString(0.1, 3));
      assertEquals("-0.100", NumberFormatTools.doubleToString(-0.1, 3));
      assertEquals("0.010", NumberFormatTools.doubleToString(0.01, 3));
      assertEquals("-0.010", NumberFormatTools.doubleToString(-0.01, 3));
      assertEquals("0.001", NumberFormatTools.doubleToString(0.001, 3));
      assertEquals("-0.001", NumberFormatTools.doubleToString(-0.001, 3));
      assertEquals("1.000e-04", NumberFormatTools.doubleToString(0.0001, 3));
      assertEquals("-1.000e-04", NumberFormatTools.doubleToString(-0.0001, 3));

      assertEquals("1.234", NumberFormatTools.doubleToString(1.234, 3));
      assertEquals("-1.234", NumberFormatTools.doubleToString(-1.234, 3));
      assertEquals("12.345", NumberFormatTools.doubleToString(12.345, 3));
      assertEquals("-12.345", NumberFormatTools.doubleToString(-12.345, 3));
      assertEquals("123.456", NumberFormatTools.doubleToString(123.456, 3));
      assertEquals("-123.456", NumberFormatTools.doubleToString(-123.456, 3));
      assertEquals("1.234e+03", NumberFormatTools.doubleToString(1234.0, 3));
      assertEquals("-1.234e+03", NumberFormatTools.doubleToString(-1234.0, 3));
      assertEquals("1.234e+04", NumberFormatTools.doubleToString(12343.0, 3));
      assertEquals("-1.234e+04", NumberFormatTools.doubleToString(-12343.0, 3));
      assertEquals("0.123", NumberFormatTools.doubleToString(0.123, 3));
      assertEquals("-0.123", NumberFormatTools.doubleToString(-0.123, 3));
      assertEquals("0.012", NumberFormatTools.doubleToString(0.012, 3));
      assertEquals("-0.012", NumberFormatTools.doubleToString(-0.012, 3));
      assertEquals("0.001", NumberFormatTools.doubleToString(0.001, 3));
      assertEquals("-0.001", NumberFormatTools.doubleToString(-0.001, 3));
      assertEquals("1.234e-04", NumberFormatTools.doubleToString(0.0001234, 3));
      assertEquals("-1.234e-04", NumberFormatTools.doubleToString(-0.0001234, 3));
   }
}

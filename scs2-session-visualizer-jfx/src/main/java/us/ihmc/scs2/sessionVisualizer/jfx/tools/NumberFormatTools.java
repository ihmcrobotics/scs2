package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.text.DecimalFormatSymbols;
import java.util.Locale;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

public class NumberFormatTools
{
   private static final DecimalFormatSymbols symbols = DecimalFormatSymbols.getInstance(Locale.getDefault(Locale.Category.FORMAT));

   public static String doubleToString(double value, int precision)
   {
      if (Double.isNaN(value))
         return symbols.getNaN();
      if (Double.isInfinite(value))
         return value > 0.0 ? symbols.getInfinity() : "-" + symbols.getInfinity();

      double lowerBoundBeforeScientific = 1.0;
      double upperBoundBeforeScientific = 1.0;

      for (int i = 0; i < precision; i++)
         upperBoundBeforeScientific *= 10.0;
      lowerBoundBeforeScientific /= upperBoundBeforeScientific;

      if (value == 0.0)
      {
         return "0.0";
      }
      else if (Math.abs(value) > upperBoundBeforeScientific || Math.abs(value) < lowerBoundBeforeScientific)
      {
         String format = "%1." + precision + "e";
         return String.format(format, value);
      }
      else
      {
         String format = EuclidCoreIOTools.getStringFormat(1, precision);
         return String.format(format, value);
      }
   }

   public static Double parseDouble(String string)
   {
      try
      {
         if (string == null)
            return null;

         string = string.trim();

         if (string.length() < 1)
            return null;

         return Double.valueOf(string);
      }
      catch (NumberFormatException e)
      {
         string = string.toLowerCase();

         if (string.equals(symbols.getInfinity()) || string.equals("infinity") || string.equals("infy"))
            return Double.POSITIVE_INFINITY;
         if (string.equals(symbols.getMinusSign() + symbols.getInfinity()) || string.equals("-infinity") || string.equals("-infy"))
            return Double.POSITIVE_INFINITY;
         if (string.equals(symbols.getNaN()) || string.equals("nan"))
            return Double.NaN;
         return null;
      }
   }
}

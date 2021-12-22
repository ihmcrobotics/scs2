package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import com.sun.javafx.binding.IntegerConstant;

import javafx.beans.binding.IntegerExpression;
import javafx.beans.property.Property;
import javafx.beans.value.ObservableIntegerValue;
import javafx.util.StringConverter;

public class ScientificDoubleStringConverter extends StringConverter<Double>
{
   private final ObservableIntegerValue precision;

   public ScientificDoubleStringConverter(int precision)
   {
      this.precision = IntegerConstant.valueOf(precision);
   }

   public ScientificDoubleStringConverter(ObservableIntegerValue precision)
   {
      this.precision = precision;
   }

   public ScientificDoubleStringConverter(Property<Integer> precision)
   {
      this.precision = IntegerExpression.integerExpression(precision);
   }

   @Override
   public Double fromString(String string)
   {
      return NumberFormatTools.parseDouble(string);
   }

   @Override
   public String toString(Double value)
   {
      if (value == null)
         return "";

      return NumberFormatTools.doubleToString(value, precision.get());
   }
}

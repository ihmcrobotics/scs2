package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import javafx.util.converter.IntegerStringConverter;

public class IntegerConverter extends IntegerStringConverter
{
   @Override
   public Integer fromString(String value)
   {
      if (value == null || value.isEmpty())
         return 0;
      return super.fromString(value);
   }
}
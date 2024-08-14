package us.ihmc.scs2.definition.yoVariable;

import java.util.Arrays;

import jakarta.xml.bind.annotation.XmlElement;

public class YoEnumDefinition extends YoVariableDefinition
{
   private boolean allowNullValue;
   private String[] enumValuesAsString;

   public YoEnumDefinition()
   {
   }

   @XmlElement
   public void setAllowNullValue(boolean allowNullValue)
   {
      this.allowNullValue = allowNullValue;
   }

   @XmlElement
   public void setEnumValuesAsString(String[] enumValuesAsString)
   {
      this.enumValuesAsString = enumValuesAsString;
   }

   public boolean isAllowNullValue()
   {
      return allowNullValue;
   }

   public String[] getEnumValuesAsString()
   {
      return enumValuesAsString;
   }

   @Override
   public String toString()
   {
      return super.toString() + ", allowNullValue: " + allowNullValue + ", enumValuesAsString: " + Arrays.toString(enumValuesAsString);
   }
}

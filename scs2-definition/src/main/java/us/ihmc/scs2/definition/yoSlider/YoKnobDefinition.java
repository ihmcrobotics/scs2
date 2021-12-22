package us.ihmc.scs2.definition.yoSlider;

import javax.xml.bind.annotation.XmlAttribute;

public class YoKnobDefinition
{
   private String variableName;
   private String minValue;
   private String maxValue;

   @XmlAttribute
   public void setVariableName(String variableName)
   {
      this.variableName = variableName;
   }

   @XmlAttribute
   public void setMinValue(String minValue)
   {
      this.minValue = minValue;
   }

   @XmlAttribute
   public void setMaxValue(String maxValue)
   {
      this.maxValue = maxValue;
   }

   public String getVariableName()
   {
      return variableName;
   }

   public String getMinValue()
   {
      return minValue;
   }

   public String getMaxValue()
   {
      return maxValue;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoKnobDefinition)
      {
         YoKnobDefinition other = (YoKnobDefinition) object;

         if (variableName == null ? other.variableName != null : variableName.equals(other.variableName))
            return false;
         if (minValue == null ? other.minValue != null : minValue.equals(other.minValue))
            return false;
         if (maxValue == null ? other.maxValue != null : maxValue.equals(other.maxValue))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "variableName:" + variableName + ", minValue:" + minValue + ", maxValue:" + maxValue;
   }
}

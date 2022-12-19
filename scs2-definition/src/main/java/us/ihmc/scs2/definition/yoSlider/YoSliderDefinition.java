package us.ihmc.scs2.definition.yoSlider;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;

public class YoSliderDefinition
{
   private String variableName;
   private String minValue;
   private String maxValue;
   private int index = -1;

   public YoSliderDefinition()
   {
   }

   public YoSliderDefinition(String variableName, int index)
   {
      this.variableName = variableName;
      this.index = index;
   }

   @XmlAttribute
   public void setVariableName(String variableName)
   {
      this.variableName = variableName;
   }

   public void setMinValue(double minValue)
   {
      this.minValue = Double.toString(minValue);
   }

   @XmlAttribute
   public void setMinValue(String minValue)
   {
      this.minValue = minValue;
   }

   public void setMaxValue(double maxValue)
   {
      this.maxValue = Double.toString(maxValue);
   }

   @XmlAttribute
   public void setMaxValue(String maxValue)
   {
      this.maxValue = maxValue;
   }

   @XmlAttribute
   public void setIndex(int index)
   {
      this.index = index;
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

   public int getIndex()
   {
      return index;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoSliderDefinition)
      {
         YoSliderDefinition other = (YoSliderDefinition) object;

         if (!Objects.equals(variableName, other.variableName))
            return false;
         if (!Objects.equals(minValue, other.minValue))
            return false;
         if (!Objects.equals(maxValue, other.maxValue))
            return false;
         if (index != other.index)
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
      return "variableName:" + variableName + ", minValue:" + minValue + ", maxValue:" + maxValue + ", index:" + index;
   }
}

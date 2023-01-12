package us.ihmc.scs2.definition.yoSlider;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;

/**
 * Definition for a slider of a sliderboard.
 * <p>
 * For the BCF2000 sliderboard, there are 8 sliders indexed here from 0 (left slider) to 7 (right
 * slider).
 * </p>
 * 
 * @author Sylvain Bertrand
 * @see YoSliderboardDefinition
 */
public class YoSliderDefinition
{
   /**
    * The name of the {@code YoVariable} to link to the slider.
    * <p>
    * It can be either the fullname (including namespace, e.g. {@code "root.Controller.myVariable"}) or
    * the simple name (without namespace, e.g. {@code "myVariable"}). In case of name duplicates
    * between {@code YoVariable}s, prefer using the fullname to guarantee which variable is linked.
    * </p>
    */
   private String variableName;
   /**
    * The lowest value as a {@code String} of the variable when the slider is at its lowest.
    * <p>
    * If this is {@code null}, a valid arbitrary lower value will be determined at runtime.
    * </p>
    */
   private String minValue;
   /**
    * The highest value as a {@code String} of the variable when the slider is at its highest.
    * <p>
    * If this is {@code null}, a valid arbitrary lower value will be determined at runtime.
    * </p>
    */
   private String maxValue;
   /**
    * The index in [0, 7] of the slider. For the BCF2000:
    * <ul>
    * <li>0 is the left most slider.
    * <li>7 is the right most slider.
    * <li>when -1 is given, the position of {@code this} in {@link YoSliderboardDefinition#getKnobs()}
    * is used to determine the slider index. This is for backward compatibility, prefer defining the
    * actual index.
    * </ul>
    */
   private int index = -1;

   public YoSliderDefinition()
   {
   }

   public YoSliderDefinition(String variableName, int index)
   {
      this.variableName = variableName;
      this.index = index;
   }

   public YoSliderDefinition(String variableName, int index, double minValue, double maxValue)
   {
      this.variableName = variableName;
      this.index = index;
      setMinValue(minValue);
      setMaxValue(maxValue);
   }

   public YoSliderDefinition(YoSliderDefinition other)
   {
      set(other);
   }

   public void set(YoSliderDefinition other)
   {
      variableName = other.variableName;
      minValue = other.minValue;
      maxValue = other.maxValue;
      index = other.index;
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

package us.ihmc.scs2.definition.yoSlider;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;

/**
 * Definition for a knob of a sliderboard.
 * <p>
 * For the BCF2000 sliderboard, there are 8 knobs indexed here from 0 (left knob) to 7 (right knob).
 * </p>
 *
 * @author Sylvain Bertrand
 * @see YoSliderboardDefinition
 */
public class YoKnobDefinition
{
   /**
    * The fullname (including namespace, e.g. {@code "root.Controller.myVariable"}) of the
    * {@code YoVariable} to link to the knob.
    */
   private String variableName;
   /**
    * The lowest value as a {@code String} of the variable when the knob is at its lowest.
    * <p>
    * If this is {@code null}, a valid arbitrary lower value will be determined at runtime.
    * </p>
    */
   private String minValue;
   /**
    * The highest value as a {@code String} of the variable when the knob is at its highest.
    * <p>
    * If this is {@code null}, a valid arbitrary lower value will be determined at runtime.
    * </p>
    */
   private String maxValue;
   /**
    * The index in [0, 7] of the knob. For the BCF2000:
    * <ul>
    * <li>0 is the left most knob.
    * <li>7 is the right most knob.
    * <li>when -1 is given, the position of {@code this} in {@link YoSliderboardDefinition#getKnobs()}
    * is used to determine the knob index.
    * </ul>
    */
   private int index = -1;

   public YoKnobDefinition()
   {
   }

   public YoKnobDefinition(String variableName, int index)
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
      else if (object instanceof YoKnobDefinition)
      {
         YoKnobDefinition other = (YoKnobDefinition) object;

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

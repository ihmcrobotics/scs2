package us.ihmc.scs2.definition.yoSlider;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;

/**
 * Definition for a button of a sliderboard.
 * <p>
 * For the BCF2000, there are 16 buttons indexed here from 0 (top row left button) to 15 (bottom row
 * right button).
 * </p>
 * 
 * @author Sylvain Bertrand
 * @see YoSliderboardDefinition
 */
public class YoButtonDefinition
{
   /**
    * The name of the {@code YoVariable} to link to the button.
    * <p>
    * It can be either the fullname (including namespace, e.g. {@code "root.Controller.myVariable"}) or
    * the simple name (without namespace, e.g. {@code "myVariable"}). In case of name duplicates
    * between {@code YoVariable}s, prefer using the fullname to guarantee which variable is linked.
    * </p>
    * <p>
    * Note that is the variable is not of type boolean, this definition will be ignored.
    * </p>
    */
   private String variableName;
   /**
    * The index in [0, 15] of the button. For the BCF2000:
    * <ul>
    * <li>0 is the top row left most button.
    * <li>7 is the top row right most button.
    * <li>8 is the bottom row left most button.
    * <li>15 is the bottom row right most button.
    * <li>when -1 is given, the position of {@code this} in
    * {@link YoSliderboardDefinition#getButtons()} is used to determine the button index. This is for
    * backward compatibility, prefer defining the actual index.
    * </ul>
    */
   private int index = -1;

   public YoButtonDefinition()
   {
   }

   public YoButtonDefinition(String variableName, int index)
   {
      this.variableName = variableName;
      this.index = index;
   }

   public YoButtonDefinition(YoButtonDefinition other)
   {
      set(other);
   }

   public void set(YoButtonDefinition other)
   {
      variableName = other.variableName;
      index = other.index;
   }

   @XmlAttribute
   public void setVariableName(String variableName)
   {
      this.variableName = variableName;
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
      else if (object instanceof YoButtonDefinition)
      {
         YoButtonDefinition other = (YoButtonDefinition) object;

         if (!Objects.equals(variableName, other.variableName))
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
      return "variableName:" + variableName + ", index:" + index;
   }
}

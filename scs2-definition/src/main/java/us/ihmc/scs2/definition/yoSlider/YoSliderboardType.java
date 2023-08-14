package us.ihmc.scs2.definition.yoSlider;

/**
 * Enum for referring to the type of sliderboard to use.
 */
public enum YoSliderboardType
{
   BCF2000("BCF2000"), XTOUCHCOMPACT("X-TOUCH COMPACT");

   private final String typeString;

   YoSliderboardType(String typeString)
   {
      this.typeString = typeString;
   }

   public String getTypeString()
   {
      return typeString;
   }
}
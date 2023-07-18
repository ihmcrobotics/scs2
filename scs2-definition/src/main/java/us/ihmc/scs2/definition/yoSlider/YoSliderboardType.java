package us.ihmc.scs2.definition.yoSlider;

/**
 * Type value for the BCF2000.
 */
public enum YoSliderboardType
{
   BCF2000("BCF2000"),
   XTOUCHCOMPACT("X-TOUCH COMPACT");
   
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
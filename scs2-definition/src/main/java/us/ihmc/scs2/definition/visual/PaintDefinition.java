package us.ihmc.scs2.definition.visual;

import jakarta.xml.bind.annotation.XmlSeeAlso;

import us.ihmc.scs2.definition.yoComposite.YoColorRGBADoubleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBAIntDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;

/**
 * {@code PaintDefinition} is used to indicate that implementations of this interface represent a
 * color.
 * <p>
 * It allows, when permitted, to create visuals using different color representations which for
 * instance can be either static color or dynamic colors using {@code YoVariable}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
@XmlSeeAlso({ColorDefinition.class, YoColorRGBADoubleDefinition.class, YoColorRGBAIntDefinition.class, YoColorRGBASingleDefinition.class})
public abstract class PaintDefinition
{
   /**
    * Returns a deep copy of {@code this}.
    * 
    * @return the copy.
    */
   public abstract PaintDefinition copy();

   /**
    * Returns a {@code String} representation of this color.
    * <p>
    * The returned string can later be used for parsing the color back using {@link #parse(String)}.
    * </p>
    */
   @Override
   public abstract String toString();

   /**
    * Parses a new {@code PaintDefinition} from the given {@code String} representation.
    * <p>
    * This method tries to parse the paint using every known implementation and will return the first
    * one that corresponds to the {@code String} representation.
    * </p>
    * 
    * @param value the {@code String} representation of the paint.
    * @return the new paint.
    */
   public static PaintDefinition parse(String value)
   {
      if (value == null || value.equalsIgnoreCase("null"))
         return null;

      if (value.startsWith("YoDoubleRGB"))
         return YoColorRGBADoubleDefinition.parse(value);
      if (value.startsWith("YoIntRGB"))
         return YoColorRGBAIntDefinition.parse(value);
      if (value.startsWith("YoRGBA"))
         return YoColorRGBASingleDefinition.parse(value);
      return ColorDefinition.parse(value);
   }
}

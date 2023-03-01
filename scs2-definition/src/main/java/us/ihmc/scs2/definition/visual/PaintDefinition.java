package us.ihmc.scs2.definition.visual;

import javax.xml.bind.annotation.XmlSeeAlso;

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
   public abstract PaintDefinition copy();

   public static PaintDefinition parse(String value)
   {
      if (value == null)//TODO || value.equalsIgnoreCase("null"))
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

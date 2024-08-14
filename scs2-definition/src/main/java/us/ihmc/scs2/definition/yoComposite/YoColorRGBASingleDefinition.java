package us.ihmc.scs2.definition.yoComposite;

import java.util.Objects;

import jakarta.xml.bind.annotation.XmlAttribute;

import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.PaintDefinition;

/**
 * {@code YoColorRGBASingleDefinition} represents a color defined with a single integer component
 * {@code rgba} that can be backed by {@code YoVariable}s.
 * <p>
 * This implementation allows to create color that are dynamically changing at runtime.
 * </p>
 * <p>
 * The four components (red, green, blue, alpha) are retrieved at runtime using
 * {@link ColorDefinitions#rgba(int)}.
 * </p>
 * 
 * @author Sylvain Bertrand
 * @see ColorDefinition#toRGBA()
 * @see ColorDefinitions#rgba(int)
 */
public class YoColorRGBASingleDefinition extends PaintDefinition
{
   /**
    * The color is defined by a single 32-bit integer.
    * 
    * @see ColorDefinition#toRGBA()
    * @see ColorDefinition#rgba(int)
    */
   private String rgba;

   /**
    * Creates a new color which component needs to be initialized.
    */
   public YoColorRGBASingleDefinition()
   {
   }

   /**
    * Creates a new color.
    * 
    * @param rgba the name/fullname of the {@code YoVariable} to back the component. Can also be a
    *             constant by using {@link Integer#toString(int)} for instance.
    * @see ColorDefinition#toRGBA()
    * @see ColorDefinition#rgba(int)
    */
   public YoColorRGBASingleDefinition(String rgba)
   {
      this.rgba = rgba;
   }

   /**
    * Copy constructor.
    * 
    * @param other the other color to make a copy of.
    */
   public YoColorRGBASingleDefinition(YoColorRGBASingleDefinition other)
   {
      rgba = other.rgba;
   }

   /**
    * Sets the information for the data backing up the component.
    * 
    * @param rgba the name/fullname of the {@code YoVariable} to back the component. Can also be a
    *             constant by using {@link Integer#toString(int)} for instance.
    * @see ColorDefinition#toRGBA()
    * @see ColorDefinition#rgba(int)
    */
   @XmlAttribute
   public void setRGBA(String rgba)
   {
      this.rgba = rgba;
   }

   /**
    * Gets the information for the data backing up the component.
    * 
    * @return the rgba value as {@code String}.
    * @see ColorDefinition#toRGBA()
    * @see ColorDefinition#rgba(int)
    */
   public String getRGBA()
   {
      return rgba;
   }

   @Override
   public YoColorRGBASingleDefinition copy()
   {
      return new YoColorRGBASingleDefinition(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoColorRGBASingleDefinition other)
      {
         if (!Objects.equals(rgba, other.rgba))
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
      return "YoRGBA=%s".formatted(rgba);
   }

   /**
    * Parses the given {@code value} into a {@link YoColorRGBASingleDefinition}. The given
    * {@code String} representation is expected to have been generated using {@link #toString()}. If
    * the format differs, this method will throw an {code IllegalArgumentException}.
    * 
    * @param value the {@code String} representation of a {@link YoColorRGBASingleDefinition}.
    * @return the parsed color object.
    */
   public static YoColorRGBASingleDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith("YoRGBA="))
      {
         String rgba = value.substring(7);

         if (rgba.equalsIgnoreCase("null"))
            rgba = null;

         return new YoColorRGBASingleDefinition(rgba);
      }
      else
      {
         throw new IllegalArgumentException("Unknown color format: " + value);
      }
   }
}

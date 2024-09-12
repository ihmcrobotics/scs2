package us.ihmc.scs2.definition.yoComposite;

import java.util.Objects;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlType;

import us.ihmc.scs2.definition.visual.PaintDefinition;

/**
 * {@code YoColorRGBADoubleDefinition} represents a color defined as (red, green, blue, alpha) and
 * can be backed by {@code YoVariable}s.
 * <p>
 * This implementation allows to create color that are dynamically changing at runtime.
 * </p>
 * <p>
 * The value for each component is expected to be in [0.0-1.0].
 * </p>
 * <p>
 * The alpha component is typically used to represent the opacity.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
@XmlType(propOrder = {"red", "green", "blue", "alpha"})
public class YoColorRGBADoubleDefinition extends PaintDefinition
{
   /**
    * The 4 variables for controlling separately the 3 primary colors and the opacity.
    * <p>
    * The values/variables are expected to all be doubles and each component range is in [0.0-1.0].
    * </p>
    */
   private String red, green, blue, alpha;

   /**
    * Creates a new color which components need to be initialized.
    */
   public YoColorRGBADoubleDefinition()
   {
   }

   /**
    * Creates a new opaque color.
    * 
    * @param red   the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    * @param green the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    * @param blue  the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    */
   public YoColorRGBADoubleDefinition(String red, String green, String blue)
   {
      this(red, green, blue, null);
   }

   /**
    * Creates a new color.
    * 
    * @param red   the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    * @param green the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    * @param blue  the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    * @param alpha the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    */
   public YoColorRGBADoubleDefinition(String red, String green, String blue, String alpha)
   {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.alpha = alpha;
   }

   /**
    * Copy constructor.
    * 
    * @param other the other color to make a copy of.
    */
   public YoColorRGBADoubleDefinition(YoColorRGBADoubleDefinition other)
   {
      this.red = other.red;
      this.green = other.green;
      this.blue = other.blue;
      this.alpha = other.alpha;
   }

   /**
    * Sets the red component to the given constant value.
    * 
    * @param red the constant value for red in the range [0.0-1.0].
    */
   public void setRed(double red)
   {
      setRed(Double.toString(red));
   }

   /**
    * Sets the red component to the given constant value.
    * 
    * @param red the constant value for red in the range [0-255].
    */
   public void setRed(int red)
   {
      setRed(Double.toString(red / 255.0));
   }

   /**
    * Sets the information for the data backing up the red component.
    * 
    * @param red the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *            constant by using {@link Double#toString(double)} for instance. The value range is
    *            expected to be [0.0-1.0].
    */
   @XmlAttribute
   public void setRed(String red)
   {
      this.red = red;
   }

   /**
    * Sets the green component to the given constant value.
    * 
    * @param green the constant value for green in the range [0.0-1.0].
    */
   public void setGreen(double green)
   {
      setGreen(Double.toString(green));
   }

   /**
    * Sets the green component to the given constant value.
    * 
    * @param green the constant value for green in the range [0-255].
    */
   public void setGreen(int green)
   {
      setGreen(Double.toString(green / 255.0));
   }

   /**
    * Sets the information for the data backing up the green component.
    * 
    * @param green the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    */
   @XmlAttribute
   public void setGreen(String green)
   {
      this.green = green;
   }

   /**
    * Sets the blue component to the given constant value.
    * 
    * @param blue the constant value for blue in the range [0.0-1.0].
    */
   public void setBlue(double blue)
   {
      setBlue(Double.toString(blue));
   }

   /**
    * Sets the blue component to the given constant value.
    * 
    * @param blue the constant value for blue in the range [0-255].
    */
   public void setBlue(int blue)
   {
      setBlue(Double.toString(blue / 255.0));
   }

   /**
    * Sets the information for the data backing up the blue component.
    * 
    * @param blue the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *             constant by using {@link Double#toString(double)} for instance. The value range is
    *             expected to be [0.0-1.0].
    */
   @XmlAttribute
   public void setBlue(String blue)
   {
      this.blue = blue;
   }

   /**
    * Sets the alpha component to the given constant value.
    * 
    * @param alpha the constant value for alpha in the range [0.0-1.0].
    */
   public void setAlpha(double alpha)
   {
      setAlpha(Double.toString(alpha));
   }

   /**
    * Sets the alpha component to the given constant value.
    * 
    * @param alpha the constant value for alpha in the range [0-255].
    */
   public void setAlpha(int alpha)
   {
      setAlpha(Double.toString(alpha / 255.0));
   }

   /**
    * Sets the information for the data backing up the alpha component.
    * 
    * @param alpha the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Double#toString(double)} for instance. The value range is
    *              expected to be [0.0-1.0].
    */
   @XmlAttribute
   public void setAlpha(String alpha)
   {
      this.alpha = alpha;
   }

   /**
    * Gets the information for the data backing up the red component.
    * 
    * @return the red value as {@code String}.
    */
   public String getRed()
   {
      return red;
   }

   /**
    * Gets the information for the data backing up the green component.
    * 
    * @return the green value as {@code String}.
    */
   public String getGreen()
   {
      return green;
   }

   /**
    * Gets the information for the data backing up the blue component.
    * 
    * @return the blue value as {@code String}.
    */
   public String getBlue()
   {
      return blue;
   }

   /**
    * Gets the information for the data backing up the alpha component.
    * 
    * @return the alpha value as {@code String}.
    */
   public String getAlpha()
   {
      return alpha;
   }

   @Override
   public YoColorRGBADoubleDefinition copy()
   {
      return new YoColorRGBADoubleDefinition(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoColorRGBADoubleDefinition other)
      {
         if (!Objects.equals(red, other.red))
            return false;
         if (!Objects.equals(green, other.green))
            return false;
         if (!Objects.equals(blue, other.blue))
            return false;
         if (!Objects.equals(alpha, other.alpha))
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
      if (alpha == null)
         return "YoDoubleRGB(red=%s, green=%s, blue=%s)".formatted(red, green, blue);
      else
         return "YoDoubleRGBA(red=%s, green=%s, blue=%s, alpha=%s)".formatted(red, green, blue, alpha);
   }

   /**
    * Parses the given {@code value} into a {@link YoColorRGBADoubleDefinition}. The given
    * {@code String} representation is expected to have been generated using {@link #toString()}. If
    * the format differs, this method will throw an {code IllegalArgumentException}.
    * 
    * @param value the {@code String} representation of a {@link YoColorRGBADoubleDefinition}.
    * @return the parsed color object.
    */
   public static YoColorRGBADoubleDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith("YoDoubleRGB"))
      {
         value = value.substring(11, value.length() - 1);
         boolean parseAlpha = value.charAt(0) == 'A';

         value = value.substring(value.indexOf("=") + 1);
         String red = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1);
         String green = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1);
         String blue;
         String alpha;

         if (parseAlpha)
         {
            blue = value.substring(0, value.indexOf(","));
            alpha = value.substring(value.indexOf("=") + 1);
         }
         else
         {
            blue = value;
            alpha = null;
         }

         if (red.equalsIgnoreCase("null"))
            red = null;
         if (green.equalsIgnoreCase("null"))
            green = null;
         if (blue.equalsIgnoreCase("null"))
            blue = null;
         if (parseAlpha && alpha.equalsIgnoreCase("null"))
            alpha = null;

         return new YoColorRGBADoubleDefinition(red, green, blue, alpha);
      }
      else
      {
         throw new IllegalArgumentException("Unknown color format: " + value);
      }
   }
}

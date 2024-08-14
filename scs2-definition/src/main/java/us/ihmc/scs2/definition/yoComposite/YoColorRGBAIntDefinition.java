package us.ihmc.scs2.definition.yoComposite;

import java.util.Objects;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlType;

import us.ihmc.scs2.definition.visual.PaintDefinition;

/**
 * {@code YoColorRGBAIntDefinition} represents a color defined as (red, green, blue, alpha) and can
 * be backed by {@code YoVariable}s.
 * <p>
 * This implementation allows to create color that are dynamically changing at runtime.
 * </p>
 * <p>
 * The value for each component is expected to be in [0-255].
 * </p>
 * <p>
 * The alpha component is typically used to represent the opacity.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
@XmlType(propOrder = {"red", "green", "blue", "alpha"})
public class YoColorRGBAIntDefinition extends PaintDefinition
{
   /**
    * The 4 variables for controlling separately the 3 primary colors and the opacity.
    * <p>
    * The values/variables are expected to all be integers and each component range is in [0-255].
    * </p>
    */
   private String red, green, blue, alpha;

   /**
    * Creates a new color which components need to be initialized.
    */
   public YoColorRGBAIntDefinition()
   {
   }

   /**
    * Creates a new opaque color.
    * 
    * @param red   the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
    * @param green the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
    * @param blue  the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
    */
   public YoColorRGBAIntDefinition(String red, String green, String blue)
   {
      this(red, green, blue, null);
   }

   /**
    * Creates a new color.
    * 
    * @param red   the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
    * @param green the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
    * @param blue  the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
    * @param alpha the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
    */
   public YoColorRGBAIntDefinition(String red, String green, String blue, String alpha)
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
   public YoColorRGBAIntDefinition(YoColorRGBAIntDefinition other)
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
      setRed(Integer.toString((int) (red * 255.0)));
   }

   /**
    * Sets the red component to the given constant value.
    * 
    * @param red the constant value for red in the range [0-255].
    */
   public void setRed(int red)
   {
      setRed(Integer.toString(red));
   }

   /**
    * Sets the information for the data backing up the red component.
    * 
    * @param red the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *            constant by using {@link Integer#toString(int)} for instance. The value range is
    *            expected to be [0-255].
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
      setGreen(Integer.toString((int) (green * 255.0)));
   }

   /**
    * Sets the green component to the given constant value.
    * 
    * @param green the constant value for green in the range [0-255].
    */
   public void setGreen(int green)
   {
      setGreen(Integer.toString(green));
   }

   /**
    * Sets the information for the data backing up the green component.
    * 
    * @param green the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
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
      setBlue(Integer.toString((int) (blue * 255.0)));
   }

   /**
    * Sets the blue component to the given constant value.
    * 
    * @param blue the constant value for blue in the range [0-255].
    */
   public void setBlue(int blue)
   {
      setBlue(Integer.toString(blue));
   }

   /**
    * Sets the information for the data backing up the blue component.
    * 
    * @param blue the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *             constant by using {@link Integer#toString(int)} for instance. The value range is
    *             expected to be [0-255].
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
      setAlpha(Integer.toString((int) (alpha * 255.0)));
   }

   /**
    * Sets the alpha component to the given constant value.
    * 
    * @param alpha the constant value for alpha in the range [0-255].
    */
   public void setAlpha(int alpha)
   {
      setAlpha(Integer.toString(alpha));
   }

   /**
    * Sets the information for the data backing up the alpha component.
    * 
    * @param alpha the name/fullname of the {@code YoVariable} to back this component. Can also be a
    *              constant by using {@link Integer#toString(int)} for instance. The value range is
    *              expected to be [0-255].
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
   public YoColorRGBAIntDefinition copy()
   {
      return new YoColorRGBAIntDefinition(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoColorRGBAIntDefinition other)
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
         return "YoIntRGB(red=%s, green=%s, blue=%s)".formatted(red, green, blue);
      else
         return "YoIntRGBA(red=%s, green=%s, blue=%s, alpha=%s)".formatted(red, green, blue, alpha);
   }

   /**
    * Parses the given {@code value} into a {@link YoColorRGBAIntDefinition}. The given {@code String}
    * representation is expected to have been generated using {@link #toString()}. If the format
    * differs, this method will throw an {code IllegalArgumentException}.
    * 
    * @param value the {@code String} representation of a {@link YoColorRGBAIntDefinition}.
    * @return the parsed color object.
    */
   public static YoColorRGBAIntDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith("YoIntRGB"))
      {
         value = value.substring(8, value.length() - 1);
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

         return new YoColorRGBAIntDefinition(red, green, blue, alpha);
      }
      else
      {
         throw new IllegalArgumentException("Unknown color format: " + value);
      }
   }
}

package us.ihmc.scs2.definition.yoComposite;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;

import us.ihmc.scs2.definition.visual.PaintDefinition;

@XmlType(propOrder = {"red", "green", "blue", "alpha"})
public class YoColorRGBAIntDefinition extends PaintDefinition
{
   /**
    * The 4 variables for controlling separately the 3 primary colors and the opacity.
    * <p>
    * The values/variables are expected to all be integers and each component range is in [0, 255].
    * </p>
    */
   private String red, green, blue, alpha;

   public YoColorRGBAIntDefinition()
   {
   }

   public YoColorRGBAIntDefinition(String red, String green, String blue)
   {
      this(red, green, blue, null);
   }

   public YoColorRGBAIntDefinition(String red, String green, String blue, String alpha)
   {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.alpha = alpha;
   }

   public YoColorRGBAIntDefinition(YoColorRGBAIntDefinition other)
   {
      this.red = other.red;
      this.green = other.green;
      this.blue = other.blue;
      this.alpha = other.alpha;
   }

   @XmlAttribute
   public void setRed(String red)
   {
      this.red = red;
   }

   @XmlAttribute
   public void setGreen(String green)
   {
      this.green = green;
   }

   @XmlAttribute
   public void setBlue(String blue)
   {
      this.blue = blue;
   }

   @XmlAttribute
   public void setAlpha(String alpha)
   {
      this.alpha = alpha;
   }

   public String getRed()
   {
      return red;
   }

   public String getGreen()
   {
      return green;
   }

   public String getBlue()
   {
      return blue;
   }

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

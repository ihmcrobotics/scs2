package us.ihmc.scs2.definition.yoComposite;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;

import us.ihmc.scs2.definition.visual.PaintDefinition;

public class YoColorRGBADoubleDefinition extends PaintDefinition
{
   /**
    * The 4 variables for controlling separately the 3 primary colors and the opacity.
    * <p>
    * The values/variables are expected to all be doubles and each component range is in [0, 1].
    * </p>
    */
   private String red, green, blue, alpha;

   public YoColorRGBADoubleDefinition()
   {
   }

   public YoColorRGBADoubleDefinition(String red, String green, String blue)
   {
      this(red, green, blue, null);
   }

   public YoColorRGBADoubleDefinition(String red, String green, String blue, String alpha)
   {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.alpha = alpha;
   }

   public YoColorRGBADoubleDefinition(YoColorRGBADoubleDefinition other)
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
      return "[red=" + red + ", green=" + green + ", blue=" + blue + ", alpha=" + alpha + "]";
   }
}

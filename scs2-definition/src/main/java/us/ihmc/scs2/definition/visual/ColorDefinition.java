package us.ihmc.scs2.definition.visual;

import java.util.Arrays;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;

import us.ihmc.euclid.tools.EuclidCoreTools;

/**
 * {@code ColorDefinition} represents a color defined as (red, green, blue, alpha) and is backed by
 * {@code double}s all defined in the range [0.0-1.0]. The alpha value is typically used to
 * represent the opacity.
 * 
 * @author Sylvain Bertrand
 * @see ColorDefinitions
 */
@XmlType(propOrder = {"red", "green", "blue", "alpha"})
public class ColorDefinition extends PaintDefinition
{
   private double red, green, blue, alpha;

   /**
    * Creates a new color initialized as opaque black.
    */
   public ColorDefinition()
   {
      this(0.0, 0.0, 0.0, 1.0);
   }

   /**
    * Copy constructor.
    * 
    * @param other the other color to copy. Not modified.
    */
   public ColorDefinition(ColorDefinition other)
   {
      this(other.red, other.green, other.blue, other.alpha);
   }

   /**
    * Creates a new color initialized from RGB components and opaque.
    * 
    * @param red   red component in range [0-255].
    * @param green green component in range [0-255].
    * @param blue  blue component in range [0-255].
    */
   public ColorDefinition(int red, int green, int blue)
   {
      this(red, green, blue, 1.0);
   }

   /**
    * Creates a new color initialized from RGB components.
    * 
    * @param red   red component in range [0-255].
    * @param green green component in range [0-255].
    * @param blue  blue component in range [0-255].
    * @param alpha alpha component in range [0.0-1.0], 0.0 being fully transparent and 1.0 fully
    *              opaque.
    */
   public ColorDefinition(int red, int green, int blue, double alpha)
   {
      setRed(red);
      setGreen(green);
      setBlue(blue);
      setAlpha(alpha);
   }

   /**
    * Creates a new color initialized from RGB components.
    * 
    * @param red   red component in range [0.0-255].
    * @param green green component in range [0-255].
    * @param blue  blue component in range [0-255].
    * @param alpha alpha component in range [0-255], 0 being fully transparent and 255 fully opaque.
    */
   public ColorDefinition(int red, int green, int blue, int alpha)
   {
      setRed(red);
      setGreen(green);
      setBlue(blue);
      setAlpha(alpha);
   }

   /**
    * Creates a new color initialized from RGB components and opaque.
    * 
    * @param red   red component in range [0.0-1.0].
    * @param green green component in range [0.0-1.0].
    * @param blue  blue component in range [0.0-1.0].
    */
   public ColorDefinition(double red, double green, double blue)
   {
      this(red, green, blue, 1.0);
   }

   /**
    * Creates a new color initialized from RGB components.
    * 
    * @param red   red component in range [0.0-1.0].
    * @param green green component in range [0.0-1.0].
    * @param blue  blue component in range [0.0-1.0].
    * @param alpha alpha component in range [0.0-1.0], 0.0 being fully transparent and 1.0 fully
    *              opaque.
    */
   public ColorDefinition(double red, double green, double blue, double alpha)
   {
      setRed(red);
      setGreen(green);
      setBlue(blue);
      setAlpha(alpha);
   }

   /**
    * Sets the red component for this color.
    * 
    * @param red red component in range [0.0-255].
    */
   public void setRed(int red)
   {
      setRed(red / 255.0);
   }

   /**
    * Sets the red component for this color.
    * 
    * @param red red component in range [0.0-1.0].
    */
   @XmlAttribute
   public void setRed(double red)
   {
      if (red < 0.0)
         this.red = 0.0;
      else if (red > 1.0)
         this.red = 1.0;
      else
         this.red = red;
   }

   /**
    * Sets the green component for this color.
    * 
    * @param green green component in range [0.0-255].
    */
   public void setGreen(int green)
   {
      setGreen(green / 255.0);
   }

   /**
    * Sets the green component for this color.
    * 
    * @param green green component in range [0.0-1.0].
    */
   @XmlAttribute
   public void setGreen(double green)
   {
      if (green < 0.0)
         this.green = 0.0;
      else if (green > 1.0)
         this.green = 1.0;
      else
         this.green = green;
   }

   /**
    * Sets the blue component for this color.
    * 
    * @param blue blue component in range [0.0-255].
    */
   public void setBlue(int blue)
   {
      setBlue(blue / 255.0);
   }

   /**
    * Sets the blue component for this color.
    * 
    * @param blue blue component in range [0.0-1.0].
    */
   @XmlAttribute
   public void setBlue(double blue)
   {
      if (blue < 0.0)
         this.blue = 0.0;
      else if (blue > 1.0)
         this.blue = 1.0;
      else
         this.blue = blue;
   }

   /**
    * Sets the alpha component for this color.
    * 
    * @param alpha alpha component in range [0-255], 0 being fully transparent and 255 fully opaque.
    */
   public void setAlpha(int alpha)
   {
      setAlpha(alpha / 255.0);
   }

   /**
    * Sets the alpha component for this color.
    * 
    * @param alpha alpha component in range [0.0-1.0], 0.0 being fully transparent and 1.0 fully
    *              opaque.
    */
   @XmlAttribute
   public void setAlpha(double alpha)
   {
      if (alpha < 0.0)
         this.alpha = 0.0;
      else if (alpha > 1.0)
         this.alpha = 1.0;
      else
         this.alpha = alpha;
   }

   /**
    * Returns the value for the red component.
    * 
    * @return the value for the red component in range [0.0-1.0].
    */
   public double getRed()
   {
      return red;
   }

   /**
    * Returns the value for the red component.
    * 
    * @return the value for the red component in range [0-255].
    */
   public int getRedAsInteger()
   {
      return (int) Math.round(red * 255.0);
   }

   /**
    * Returns the value for the green component.
    * 
    * @return the value for the green component in range [0.0-1.0].
    */
   public double getGreen()
   {
      return green;
   }

   /**
    * Returns the value for the green component.
    * 
    * @return the value for the green component in range [0-255].
    */
   public int getGreenAsInteger()
   {
      return (int) Math.round(green * 255.0);
   }

   /**
    * Returns the value for the blue component.
    * 
    * @return the value for the blue component in range [0.0-1.0].
    */
   public double getBlue()
   {
      return blue;
   }

   /**
    * Returns the value for the blue component.
    * 
    * @return the value for the blue component in range [0-255].
    */
   public int getBlueAsInteger()
   {
      return (int) Math.round(blue * 255.0);
   }

   /**
    * Returns whether this color is completely opaque, i.e. {@code alpha = 1}, or transparent, i.e.
    * {@code alpha < 1}.
    * 
    * @return {@code true} if this color is opaque, {@code false} otherwise.
    */
   public boolean isOpaque()
   {
      return alpha == 1.0;
   }

   /**
    * Returns the value for the alpha component.
    * 
    * @return the value for the alpha component in range [0.0-1.0], 0.0 being fully transparent and 1.0
    *         fully opaque.
    */
   public double getAlpha()
   {
      return alpha;
   }

   /**
    * Returns the value for the alpha component.
    * 
    * @return the value for the alpha component in range [0-255], 0 being fully transparent and 255
    *         fully opaque.
    */
   public int getAlphaAsInteger()
   {
      return (int) (alpha * 255.0);
   }

   /**
    * Returns the hue component using the HSV/HSB representation for this color.
    * 
    * @return the hue component in range [0-360].
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public double getHue()
   {
      double brightness = getBrightness();
      double c = brightness - EuclidCoreTools.min(red, green, blue);

      double hue = 0.0;
      if (c == 0.0)
         hue = 0.0;
      else if (brightness == red)
         hue = 60.0 * (green - blue) / c;
      else if (brightness == green)
         hue = 60.0 * (2.0 + (blue - red) / c);
      else if (brightness == blue)
         hue = 60.0 * (4.0 + (red - green) / c);
      if (hue < 0.0)
         hue += 360.0;
      hue %= 360.0;
      return hue;
   }

   /**
    * Returns the saturation component using the HSV/HSB representation for this color.
    * 
    * @return the saturation component in range [0.0-1.0].
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public double getSaturation()
   {
      double brightness = getBrightness();
      if (brightness == 0.0)
         return 0.0;
      else
         return 1.0 - EuclidCoreTools.min(red, green, blue) / brightness;
   }

   /**
    * Returns the value/brightness component using the HSV/HSB representation for this color.
    * 
    * @return the value/brightness component in range [0.0-1.0].
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public double getBrightness()
   {
      return EuclidCoreTools.max(red, green, blue);
   }

   /**
    * Convenience method for inverting all three color components. Note that the alpha is unchanged.
    * 
    * @return the new inverted color.
    */
   public ColorDefinition invert()
   {
      return new ColorDefinition(1.0 - red, 1.0 - green, 1.0 - blue, alpha);
   }

   private static final double BRIGHTNESS_SCALE = 0.7;
   private static final double SATURATION_SCALE = 0.7;

   /**
    * Computes and returns a new color that is brighter than {@code this}.
    * 
    * @return the new brighter color.
    */
   public ColorDefinition brighter()
   {
      return derive(0, 1.0, 1.0 / BRIGHTNESS_SCALE, 1.0);
   }

   /**
    * Computes and returns a new color that is darker than {@code this}.
    * 
    * @return the new darker color.
    */
   public ColorDefinition darker()
   {
      return derive(0, 1.0, BRIGHTNESS_SCALE, 1.0);
   }

   /**
    * Computes and returns a new color that is more saturated than {@code this}.
    * 
    * @return the new color more saturated.
    */
   public ColorDefinition saturate()
   {
      return derive(0.0, 1.0 / SATURATION_SCALE, 1.0, 1.0);
   }

   /**
    * Computes and returns a new color that is less saturated than {@code this}.
    * 
    * @return the new color less saturated.
    */
   public ColorDefinition desaturate()
   {
      return derive(0.0, SATURATION_SCALE, 1.0, 1.0);
   }

   /**
    * Convenience method for creating a new color based on {@code this} with modifiers applied in the
    * HSB domain.
    * 
    * @param hueOffset       angle shift to add to the hue. The hue is defined in [0.0-360.0].
    * @param saturationScale scale factor to apply to the saturation. The saturation is defined in
    *                        [0.0-1.0]
    * @param brightnessScale scale factor to apply to the brightness. The brightness is defined in
    *                        [0.0-1.0]
    * @param opacityScale    scale factor to apply to the opacity (alpha). The opacity is defined in
    *                        [0.0-1.0]
    * @return the new color.
    */
   public ColorDefinition derive(double hueOffset, double saturationScale, double brightnessScale, double opacityScale)
   {
      double outputHue = getHue() + hueOffset;

      double outputSaturation = getSaturation();
      if (saturationScale > 1.0 && outputSaturation <= 1.0e-6)
         outputSaturation = 0.05;
      outputSaturation *= saturationScale;

      double outputBrightness = getBrightness();
      if (brightnessScale > 1.0 && outputBrightness <= 1.0e-6)
         outputBrightness = 0.05;
      outputBrightness *= brightnessScale;

      double outputAlpha = getAlpha();
      if (opacityScale > 1.0 && outputAlpha <= 1.0e-6)
         outputAlpha = 0.05;
      outputAlpha *= opacityScale;

      return hsba(outputHue, outputSaturation, outputBrightness, outputAlpha);
   }

   /**
    * Returns the RGB, "#00RRGGBB", value representing this color.
    * <p>
    * The components are stored as follows:
    * <ul>
    * <li>Bits [16-23] are used to store red,
    * <li>Bits [8-15] are used to store green,
    * <li>Bits [0-7] are used to store blue.
    * </ul>
    * </p>
    * 
    * @return the RGB value representing this color.
    */
   public int toRGB()
   {
      return ColorDefinitions.toRGB(red, green, blue);
   }

   /**
    * Returns the ARGB, "#AARRGGBB", value representing this color.
    * <p>
    * The components are stored as follows:
    * <ul>
    * <li>Bits [24-31] are used to store alpha,
    * <li>Bits [16-23] are used to store red,
    * <li>Bits [8-15] are used to store green,
    * <li>Bits [0-7] are used to store blue.
    * </ul>
    * </p>
    * 
    * @return the ARGB value representing this color.
    */
   public int toARGB()
   {
      return ColorDefinitions.toARGB(red, green, blue, alpha);
   }

   /**
    * Returns the RGBA, "#RRGGBBAA", value representing this color.
    * <p>
    * The components are stored as follows:
    * <ul>
    * <li>Bits [24-31] are used to store red,
    * <li>Bits [16-23] are used to store green,
    * <li>Bits [8-15] are used to store blue,
    * <li>Bits [0-7] are used to store alpha.
    * </ul>
    * </p>
    * 
    * @return the RGBA value representing this color.
    */
   public int toRGBA()
   {
      return ColorDefinitions.toRGBA(red, green, blue, alpha);
   }

   /**
    * Returns the three components red, green, and blue in order as an array.
    * <p>
    * Each component is expressed in the range [0.0-1.0].
    * </p>
    * 
    * @return the three components red, green, and blue in order as an array.
    */
   public double[] toRGBDoubleArray()
   {
      return new double[] {getRed(), getGreen(), getBlue()};
   }

   /**
    * Returns the three components red, green, and blue in order as an array.
    * <p>
    * Each component is expressed in the range [0-255].
    * </p>
    * 
    * @return the three components red, green, and blue in order as an array.
    */
   public int[] toRGBIntArray()
   {
      return new int[] {getRedAsInteger(), getGreenAsInteger(), getBlueAsInteger()};
   }

   /**
    * Returns the four components red, green, blue, and alpha in order as an array.
    * <p>
    * Each component is expressed in the range [0.0-1.0].
    * </p>
    * 
    * @return the three components red, green, blue, and alpha in order as an array.
    */
   public double[] toRGBADoubleArray()
   {
      return new double[] {getRed(), getGreen(), getBlue(), getAlpha()};
   }

   /**
    * Returns the four components red, green, blue, and alpha in order as an array.
    * <p>
    * Each component is expressed in the range [0-255].
    * </p>
    * 
    * @return the three components red, green, blue, and alpha in order as an array.
    */
   public int[] toRGBAIntArray()
   {
      return new int[] {getRedAsInteger(), getGreenAsInteger(), getBlueAsInteger(), getAlphaAsInteger()};
   }

   /**
    * Returns the three components, using the HSB/HSV representation of this color, hue, saturation,
    * and brightness in order as an array.
    * <p>
    * The hue is expressed in the range [0-360] while the saturation and brightness are expressed in
    * the range [0.0-1.0].
    * </p>
    * 
    * @return the three components hue, saturation, and brightness in order as an array.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public double[] toHSBDoubleArray()
   {
      return new double[] {getHue(), getSaturation(), getBrightness()};
   }

   /**
    * Returns the four components, using the HSB/HSV representation of this color, hue, saturation, and
    * brightness, alpha in order as an array.
    * <p>
    * The hue is expressed in the range [0-360] while the saturation, brightness, and alpha are
    * expressed in the range [0.0-1.0].
    * </p>
    * 
    * @return the four components hue, saturation, brightness, and alpha in order as an array.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public double[] toHSBADoubleArray()
   {
      return new double[] {getHue(), getSaturation(), getBrightness(), getAlpha()};
   }

   /**
    * Returns the three components, using the HSL representation of this color, hue, saturation, and
    * lightness in order as an array.
    * <p>
    * The hue is expressed in the range [0-360] while the saturation and lightness are expressed in the
    * range [0.0-1.0].
    * </p>
    * 
    * @return the three components hue, saturation, lightness in order as an array.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSL_color_solid_cylinder_saturation_gray.png">HSL
    *      representation</a>
    */
   public double[] toHSLDoubleArray()
   {
      return Arrays.copyOf(toHSLADoubleArray(), 3);
   }

   /**
    * Returns the four components, using the HSL representation of this color, hue, saturation,
    * lightness, and alpha in order as an array.
    * <p>
    * The hue is expressed in the range [0-360] while the saturation, lightness, and alpha are
    * expressed in the range [0.0-1.0].
    * </p>
    * 
    * @return the four components hue, saturation, lightness, and alpha in order as an array.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSL_color_solid_cylinder_saturation_gray.png">HSL
    *      representation</a>
    */
   public double[] toHSLADoubleArray()
   {
      double xmax = EuclidCoreTools.max(red, green, blue);
      double xmin = EuclidCoreTools.min(red, green, blue);

      double c = xmax - xmin;
      double lightness = 0.5 * (xmax + xmin);

      double hue = 0.0;
      if (c == 0.0)
         hue = 0.0;
      else if (xmax == red)
         hue = 60.0 * (green - blue) / c;
      else if (xmax == green)
         hue = 60.0 * (2.0 + (blue - red) / c);
      else if (xmax == blue)
         hue = 60.0 * (4.0 + (red - green) / c);
      if (hue < 0.0)
         hue += 360.0;
      hue %= 360.0;

      double saturation = 0.0;
      if (lightness > 0.0 && lightness < 1.0)
         saturation = (xmax - lightness) / Math.min(lightness, 1.0 - lightness);

      return new double[] {hue, saturation, lightness, alpha};
   }

   public ColorDefinition copy()
   {
      return new ColorDefinition(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ColorDefinition)
      {
         ColorDefinition other = (ColorDefinition) object;
         return red == other.red && green == other.green && blue == other.blue && alpha == other.alpha;
      }
      else
      {
         return false;
      }
   }

   @Override
   public int hashCode()
   {
      return toARGB();
   }

   @Override
   public String toString()
   {
      if (alpha == 1.0)
         return String.format("RGB(%d, %d, %d)", getRedAsInteger(), getGreenAsInteger(), getBlueAsInteger());
      else
         return String.format("RGBA(%d, %d, %d, %d)", getRedAsInteger(), getGreenAsInteger(), getBlueAsInteger(), getAlphaAsInteger());
   }

   /**
    * Creates a new opaque color from the combined RGB value.
    * <p>
    * The components are assumed to be stored as follows:
    * <ul>
    * <li>Bits [16-23] are used for red,
    * <li>Bits [8-15] are used for green,
    * <li>Bits [0-7] are used for blue.
    * </ul>
    * </p>
    * 
    * @param rgb the combined RGB value.
    * @return the new color.
    */
   public static ColorDefinition rgb(int rgb)
   {
      return ColorDefinitions.rgb(rgb);
   }

   /**
    * Creates a new color from the combined ARGB value.
    * <p>
    * The components are assumed to be stored as follows:
    * <ul>
    * <li>Bits [24-31] are used for alpha,
    * <li>Bits [16-23] are used for red,
    * <li>Bits [8-15] are used for green,
    * <li>Bits [0-7] are used for blue.
    * </ul>
    * </p>
    * 
    * @param argb the combined ARGB value.
    * @return the new color.
    */
   public static ColorDefinition argb(int argb)
   {
      return ColorDefinitions.argb(argb);
   }

   /**
    * Creates a new color from the combined RGBA value.
    * <p>
    * The components are assumed to be stored as follows:
    * <ul>
    * <li>Bits [24-31] are used for red,
    * <li>Bits [16-23] are used for green,
    * <li>Bits [8-15] are used for blue,
    * <li>Bits [0-7] are used for alpha.
    * </ul>
    * </p>
    * 
    * @param rgba the combined RGBA value.
    * @return the new color.
    */
   public static ColorDefinition rgba(int rgba)
   {
      return ColorDefinitions.rgba(rgba);
   }

   /**
    * Creates a new opaque color from the given RGB values.
    * <p>
    * The components are assumed to be ordered as red, green, and blue and expressed in the range
    * [0-255].
    * </p>
    * 
    * @param rgb the array containing the RGB components.
    * @return the new color.
    */
   public static ColorDefinition rgb(int[] rgb)
   {
      return ColorDefinitions.rgb(rgb);
   }

   /**
    * Creates a new opaque color from the given RGB values.
    * <p>
    * The components are assumed to be ordered as red, green, and blue and expressed in the range
    * [0.0-1.0].
    * </p>
    * 
    * @param rgb the array containing the RGB components.
    * @return the new color.
    */
   public static ColorDefinition rgb(double[] rgb)
   {
      return ColorDefinitions.rgb(rgb);
   }

   /**
    * Creates a new opaque color from the given RGBA values.
    * <p>
    * The components are assumed to be ordered as red, green, blue, and alpha and expressed in the
    * range [0-255].
    * </p>
    * 
    * @param rgba the array containing the RGBA components.
    * @return the new color.
    */
   public static ColorDefinition rgba(int[] rgba)
   {
      return ColorDefinitions.rgba(rgba);
   }

   /**
    * Creates a new opaque color from the given RGBA values.
    * <p>
    * The components are assumed to be ordered as red, green, blue, and alpha and expressed in the
    * range [0.0-1.0].
    * </p>
    * 
    * @param rgba the array containing the RGBA components.
    * @return the new color.
    */
   public static ColorDefinition rgba(double[] rgba)
   {
      return ColorDefinitions.rgba(rgba);
   }

   /**
    * Creates a new opaque color from the given HSB/HSV values.
    * <p>
    * The components are assumed to be ordered as hue [0-360], saturation [0.0-1.0], and
    * brightness/value [0.0-1.0].
    * </p>
    * 
    * @param hsb the array containing the HSB/HSV components.
    * @return the new color.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public static ColorDefinition hsb(double[] hsb)
   {
      return ColorDefinitions.hsb(hsb);
   }

   /**
    * Creates a new opaque color from the given HSB/HSV values.
    * 
    * @param hue        the hue component in range [0-360].
    * @param saturation the saturation component in range [0.0-1.0].
    * @param brightness the brightness/value component in range [0.0-1.0].
    * @return the new color.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public static ColorDefinition hsb(double hue, double saturation, double brightness)
   {
      return ColorDefinitions.hsb(hue, saturation, brightness);
   }

   /**
    * Creates a new color from the given HSBA/HSVA values.
    * <p>
    * The components are assumed to be ordered as hue [0-360], saturation [0.0-1.0], brightness/value
    * [0.0-1.0], and alpha [0.0-1.0].
    * </p>
    * 
    * @param hsba the array containing the HSBA/HSVA components.
    * @return the new color.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public static ColorDefinition hsba(double[] hsba)
   {
      return ColorDefinitions.hsba(hsba);
   }

   /**
    * Creates a new opaque color from the given HSBA/HSVA values.
    * 
    * @param hue        the hue component in range [0-360].
    * @param saturation the saturation component in range [0.0-1.0].
    * @param brightness the brightness/value component in range [0.0-1.0].
    * @param alpha      the alpha component in range [0.0-1.0], 0 being fully transparent and 255 fully
    *                   opaque.
    * @return the new color.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public static ColorDefinition hsba(double hue, double saturation, double brightness, double alpha)
   {
      return ColorDefinitions.hsba(hue, saturation, brightness, alpha);
   }

   /**
    * Creates a new opaque color from the given HSL values.
    * <p>
    * The components are assumed to be ordered as hue [0-360], saturation [0.0-1.0], and lightness
    * [0.0-1.0].
    * </p>
    * 
    * @param hsl the array containing the HSL components.
    * @return the new color.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSL_color_solid_cylinder_saturation_gray.png">HSL
    *      representation</a>
    */
   public static ColorDefinition hsl(double[] hsl)
   {
      return ColorDefinitions.hsl(hsl);
   }

   /**
    * Creates a new opaque color from the given HSL values.
    * 
    * @param hue        the hue component in range [0-360].
    * @param saturation the saturation component in range [0.0-1.0].
    * @param lightness  the lightness component in range [0.0-1.0].
    * @return the new color.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSL_color_solid_cylinder_saturation_gray.png">HSL
    *      representation</a>
    */
   public static ColorDefinition hsl(double hue, double saturation, double lightness)
   {
      return ColorDefinitions.hsl(hue, saturation, lightness);
   }

   /**
    * Creates a new color from the given HSLA values.
    * <p>
    * The components are assumed to be ordered as hue [0-360], saturation [0.0-1.0], lightness
    * [0.0-1.0], and alpha [0.0-1.0].
    * </p>
    * 
    * @param hsla the array containing the HSLA components.
    * @return the new color.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSL_color_solid_cylinder_saturation_gray.png">HSL
    *      representation</a>
    */
   public static ColorDefinition hsla(double[] hsla)
   {
      return ColorDefinitions.hsla(hsla);
   }

   /**
    * Creates a new opaque color from the given HSLA values.
    * 
    * @param hue        the hue component in range [0-360].
    * @param saturation the saturation component in range [0.0-1.0].
    * @param lightness  the lightness component in range [0.0-1.0].
    * @param alpha      the alpha component in range [0.0-1.0], 0 being fully transparent and 255 fully
    *                   opaque.
    * @return the new color.
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSL_color_solid_cylinder_saturation_gray.png">HSL
    *      representation</a>
    */
   public static ColorDefinition hsla(double hue, double saturation, double lightness, double alpha)
   {
      return ColorDefinitions.hsla(hue, saturation, lightness, alpha);
   }

   /**
    * Attempts to parse the given string to create a new color.
    * <p>
    * Accepted formats for parsing RGB colors:
    * <ul>
    * <li>rgb(255, 0, 0) => integer range 0 - 255
    * <li>rgb(100%, 0%, 0%) => float range 0.0% - 100.0%
    * <li>rgba(100%, 0%, 0%, 0.5) => 0.5 opacity, semi-transparent
    * <li>rgba(255, 0, 0, 0.5) => 0.5 opacity, semi-transparent
    * <li>rgba(255, 0, 0, 127) => 127/255 opacity, semi-transparent
    * </ul>
    * Note: a value of "1" for alpha, will be parsed in the range [0-255] (very transparent), while a
    * value of "1.0" will be parse in the range [0.0-1.0] (opaque).
    * </p>
    * <p>
    * Accepted formats for parsing HSV colors:
    * <ul>
    * <li>hsv(120, 50%, 50%) => hue in [0-360], saturation and value in [0.0%-100.0%]
    * <li>hsv(120, 0.5, 0.5) => hue in [0-360], saturation and value in [0.0-1.0]
    * <li>hsva(120, 100%, 50%, 0.5) => 0.5 opacity, semi-transparent
    * <li>hsva(120, 1.0, 0.5, 0.5) => 0.5 opacity, semi-transparent
    * </ul>
    * </p>
    * <p>
    * Accepted formats for parsing HSB colors:
    * <ul>
    * <li>hsb(120, 50%, 50%) => hue in [0-360], saturation and brightness in [0.0%-100.0%]
    * <li>hsb(120, 0.5, 0.5) => hue in [0-360], saturation and brightness in [0.0-1.0]
    * <li>hsba(120, 100%, 50%, 0.5) => 0.5 opacity, semi-transparent
    * <li>hsba(120, 1.0, 0.5, 0.5) => 0.5 opacity, semi-transparent
    * </ul>
    * </p>
    * <p>
    * Accepted formats for parsing HSL colors:
    * <ul>
    * <li>hsl(120, 50%, 50%) => hue in [0-360], saturation and lightness in [0.0%-100.0%]
    * <li>hsl(120, 0.5, 0.5) => hue in [0-360], saturation and lightness in [0.0-1.0]
    * <li>hsla(120, 100%, 50%, 0.5) => 0.5 opacity, semi-transparent
    * <li>hsla(120, 1.0, 0.5, 0.5) => 0.5 opacity, semi-transparent
    * </ul>
    * </p>
    * <p>
    * Accepted formats for Hex RGB colors:
    * <ul>
    * <li>#00F or 0x00F for opaque {@link #Blue()}.
    * <li>#F00F or 0xF00F for opaque {@link #Blue()}.
    * <li>#000F or 0x000F for transparent {@link #Blue()}.
    * <li>#00FF00 or 0xFF0000 for opaque {@link #Lime()}.
    * <li>#FF00FF00 or 0xFFFF0000 for opaque {@link #Red()}.
    * <li>#FF00FF00 or 0x0000FF00 for transparent {@link #Lime()}.
    * <li>
    * </ul>
    * </p>
    * <p>
    * Finally, the given string can also be the name of one of the 147 CSS named colors such as
    * "AliceBlue".
    * </p>
    * 
    * @param webColor the string representing the color to parse.
    * @return the new color.
    * @see <a href="http://www.colors.commutercreative.com/grid/">147 CSS Named Colors</a>
    * @see <a href= "https://en.wikipedia.org/wiki/Web_colors">Web colors</a>
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSL_color_solid_cylinder_saturation_gray.png">HSL
    *      representation</a>
    * @see <a href=
    *      "https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV_color_solid_cylinder_saturation_gray.png">HSB/HSV
    *      representation</a>
    */
   public static ColorDefinition parse(String webColor)
   {
      return ColorDefinitions.parse(webColor);
   }
}
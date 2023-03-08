package us.ihmc.scs2.definition.visual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.scs2.definition.DefinitionRandomTools;

public class ColorDefinitionTest
{
   private static final double EPSILON = 1.0e-9;
   private static final double LARGE_EPSILON = 1.0 / 255.0;
   private static final int ITERATIONS = 10000;

   @Test
   public void testArrayConversion()
   {
      Random random = new Random(304985);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int redInt = random.nextInt(256);
         int greenInt = random.nextInt(256);
         int blueInt = random.nextInt(256);
         int alphaInt = random.nextInt(256);
         double red = redInt / 255.0;
         double green = greenInt / 255.0;
         double blue = blueInt / 255.0;
         double alpha = alphaInt / 255.0;

         assertRGBAEquals(red, green, blue, 1, ColorDefinition.rgb(new double[] {red, green, blue}), EPSILON);
         assertRGBAEquals(red, green, blue, alpha, ColorDefinition.rgba(new double[] {red, green, blue, alpha}), EPSILON);

         assertRGBAEquals(red, green, blue, 1, ColorDefinition.rgb(new int[] {redInt, greenInt, blueInt}), EPSILON);
         assertRGBAEquals(red, green, blue, alpha, ColorDefinition.rgba(new int[] {redInt, greenInt, blueInt, alphaInt}), EPSILON);

         assertRGBEquals(red, green, blue, new ColorDefinition(red, green, blue).toRGBDoubleArray(), EPSILON);
         assertRGBAEquals(red, green, blue, alpha, new ColorDefinition(red, green, blue, alpha).toRGBADoubleArray(), EPSILON);

         assertRGBEquals(redInt, greenInt, blueInt, new ColorDefinition(redInt, greenInt, blueInt).toRGBIntArray());
         assertRGBAEquals(redInt, greenInt, blueInt, alphaInt, new ColorDefinition(redInt, greenInt, blueInt, alphaInt).toRGBAIntArray());
      }
   }

   @Test
   public void testHexConversion()
   {
      Random random = new Random(4752);

      for (int i = 0; i < 256; i++)
      {
         assertRGBAEquals(0, 0, i / 255.0, 1, ColorDefinition.rgb(i), EPSILON);
         assertRGBAEquals(0, i / 255.0, 0, 1, ColorDefinition.rgb(i << 8), EPSILON);
         assertRGBAEquals(i / 255.0, 0, 0, 1, ColorDefinition.rgb(i << 16), EPSILON);

         assertRGBAEquals(0, 0, i / 255.0, 0, ColorDefinition.argb(i), EPSILON);
         assertRGBAEquals(0, i / 255.0, 0, 0, ColorDefinition.argb(i << 8), EPSILON);
         assertRGBAEquals(i / 255.0, 0, 0, 0, ColorDefinition.argb(i << 16), EPSILON);
         assertRGBAEquals(0, 0, 0, i / 255.0, ColorDefinition.argb(i << 24), EPSILON);

         assertRGBAEquals(0, 0, i / 255.0, 0, ColorDefinition.rgba(i << 8), EPSILON);
         assertRGBAEquals(0, i / 255.0, 0, 0, ColorDefinition.rgba(i << 16), EPSILON);
         assertRGBAEquals(i / 255.0, 0, 0, 0, ColorDefinition.rgba(i << 24), EPSILON);
         assertRGBAEquals(0, 0, 0, i / 255.0, ColorDefinition.rgba(i), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RGB -> Hex -> RGB
         double red = random.nextInt(256) / 255.0;
         double green = random.nextInt(256) / 255.0;
         double blue = random.nextInt(256) / 255.0;
         double alpha = 1.0;
         ColorDefinition color = ColorDefinition.rgb(new ColorDefinition(red, green, blue).toRGB());
         assertRGBAEquals(red, green, blue, alpha, color, EPSILON);

         alpha = random.nextInt(256) / 255.0;
         color = ColorDefinition.argb(new ColorDefinition(red, green, blue, alpha).toARGB());
         assertRGBAEquals(red, green, blue, alpha, color, EPSILON);

         color = ColorDefinition.rgba(new ColorDefinition(red, green, blue, alpha).toRGBA());
         assertRGBAEquals(red, green, blue, alpha, color, EPSILON);
      }

      // RGB
      assertRGBAEquals(0, 0, 0, 1, ColorDefinition.rgb(0x000000), EPSILON);
      assertRGBAEquals(1, 0, 0, 1, ColorDefinition.rgb(0xff0000), EPSILON);
      assertRGBAEquals(0, 1, 0, 1, ColorDefinition.rgb(0x00ff00), EPSILON);
      assertRGBAEquals(0, 0, 1, 1, ColorDefinition.rgb(0x0000ff), EPSILON);
      assertRGBAEquals(1, 1, 1, 1, ColorDefinition.rgb(0xffffff), EPSILON);
      assertRGBAEquals(1.0 / 255.0, 0, 0, 1, ColorDefinition.rgb(0x010000), EPSILON);
      assertRGBAEquals(0, 1.0 / 255.0, 0, 1, ColorDefinition.rgb(0x000100), EPSILON);
      assertRGBAEquals(0, 0, 1.0 / 255.0, 1, ColorDefinition.rgb(0x000001), EPSILON);

      // ARGB
      assertRGBAEquals(0, 0, 0, 0, ColorDefinition.argb(0x00000000), EPSILON);
      assertRGBAEquals(0, 0, 0, 1, ColorDefinition.argb(0xff000000), EPSILON);
      assertRGBAEquals(1, 0, 0, 0, ColorDefinition.argb(0x00ff0000), EPSILON);
      assertRGBAEquals(0, 1, 0, 0, ColorDefinition.argb(0x0000ff00), EPSILON);
      assertRGBAEquals(0, 0, 1, 0, ColorDefinition.argb(0x000000ff), EPSILON);
      assertRGBAEquals(1, 1, 1, 1, ColorDefinition.argb(0xffffffff), EPSILON);
      assertRGBAEquals(1.0 / 255.0, 0, 0, 0, ColorDefinition.argb(0x00010000), EPSILON);
      assertRGBAEquals(0, 1.0 / 255.0, 0, 0, ColorDefinition.argb(0x00000100), EPSILON);
      assertRGBAEquals(0, 0, 1.0 / 255.0, 0, ColorDefinition.argb(0x00000001), EPSILON);
      assertRGBAEquals(0, 0, 0, 1.0 / 255.0, ColorDefinition.argb(0x01000000), EPSILON);

      // RGBA
      assertRGBAEquals(0, 0, 0, 0, ColorDefinition.rgba(0x00000000), EPSILON);
      assertRGBAEquals(1, 0, 0, 0, ColorDefinition.rgba(0xff000000), EPSILON);
      assertRGBAEquals(0, 1, 0, 0, ColorDefinition.rgba(0x00ff0000), EPSILON);
      assertRGBAEquals(0, 0, 1, 0, ColorDefinition.rgba(0x0000ff00), EPSILON);
      assertRGBAEquals(0, 0, 0, 1, ColorDefinition.rgba(0x000000ff), EPSILON);
      assertRGBAEquals(1, 1, 1, 1, ColorDefinition.rgba(0xffffffff), EPSILON);
      assertRGBAEquals(1.0 / 255.0, 0, 0, 0, ColorDefinition.rgba(0x01000000), EPSILON);
      assertRGBAEquals(0, 1.0 / 255.0, 0, 0, ColorDefinition.rgba(0x00010000), EPSILON);
      assertRGBAEquals(0, 0, 1.0 / 255.0, 0, ColorDefinition.rgba(0x00000100), EPSILON);
      assertRGBAEquals(0, 0, 0, 1.0 / 255.0, ColorDefinition.rgba(0x00000001), EPSILON);
   }

   @Test
   public void testHSBConversion()
   {
      Random random = new Random(23545);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test HSB -> RGB -> HSB
         double hue = EuclidCoreRandomTools.nextDouble(random, 0.0, 360.0);
         double saturation = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double brightness = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);

         ColorDefinition color = ColorDefinition.hsb(hue, saturation, brightness);
         assertHSBAEquals(hue, saturation, brightness, 1.0, color, EPSILON);

         double[] hsb = color.toHSBDoubleArray();
         assertHSBEquals(hue, saturation, brightness, hsb, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test HSBA -> RGBA -> HSBA
         double hue = EuclidCoreRandomTools.nextDouble(random, 0.0, 360.0);
         double saturation = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double brightness = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);

         ColorDefinition color = ColorDefinition.hsba(hue, saturation, brightness, alpha);
         assertHSBAEquals(hue, saturation, brightness, alpha, color, EPSILON);

         double[] hsba = color.toHSBADoubleArray();
         assertHSBAEquals(hue, saturation, brightness, alpha, hsba, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RGB -> HSB -> RGB
         double red = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double green = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double blue = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alpha = 1.0;

         ColorDefinition color = ColorDefinition.hsb(new ColorDefinition(red, green, blue).toHSBDoubleArray());
         assertRGBAEquals(red, green, blue, alpha, color, EPSILON);
         alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         color = ColorDefinition.hsba(new ColorDefinition(red, green, blue, alpha).toHSBADoubleArray());
         assertRGBAEquals(red, green, blue, alpha, color, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test varying red only
         double red = i / (ITERATIONS - 1.0);
         double green = 0.0;
         double blue = 0.0;

         double expectedHue = 0.0;
         double expectedSaturation = red == 0.0 ? 0.0 : 1.0;
         double expectedBrightness = red;
         ColorDefinition color = new ColorDefinition(red, green, blue);
         assertHSBEquals(expectedHue, expectedSaturation, expectedBrightness, color, EPSILON);

         green = 1.0;
         blue = 1.0;

         expectedHue = red == 1.0 ? 0.0 : 180.0;
         expectedSaturation = 1.0 - red;
         expectedBrightness = 1.0;
         color = new ColorDefinition(red, green, blue);
         assertHSBEquals(expectedHue, expectedSaturation, expectedBrightness, color, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test varying green only
         double red = 0.0;
         double green = i / (ITERATIONS - 1.0);
         double blue = 0.0;

         double expectedHue = green == 0.0 ? 0.0 : 120.0;
         double expectedSaturation = green == 0.0 ? 0.0 : 1.0;
         double expectedBrightness = green;
         ColorDefinition color = new ColorDefinition(red, green, blue);
         assertHSBEquals(expectedHue, expectedSaturation, expectedBrightness, color, EPSILON);

         red = 1.0;
         blue = 1.0;

         expectedHue = green == 1.0 ? 0.0 : 300.0;
         expectedSaturation = 1.0 - green;
         expectedBrightness = 1.0;
         color = new ColorDefinition(red, green, blue);
         assertHSBEquals(expectedHue, expectedSaturation, expectedBrightness, color, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test varying blue only
         double red = 0.0;
         double green = 0.0;
         double blue = i / (ITERATIONS - 1.0);

         double expectedHue = blue == 0.0 ? 0.0 : 240.0;
         double expectedSaturation = blue == 0.0 ? 0.0 : 1.0;
         double expectedBrightness = blue;
         ColorDefinition color = new ColorDefinition(red, green, blue);
         assertHSBEquals(expectedHue, expectedSaturation, expectedBrightness, color, EPSILON);

         red = 1.0;
         green = 1.0;

         expectedHue = blue == 1.0 ? 0.0 : 60.0;
         expectedSaturation = 1.0 - blue;
         expectedBrightness = 1.0;
         color = new ColorDefinition(red, green, blue);
         assertHSBEquals(expectedHue, expectedSaturation, expectedBrightness, color, EPSILON);
      }
   }

   @Test
   public void testHSLConversion()
   {
      Random random = new Random(23545);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test HSL -> RGB -> HSL
         double hue = EuclidCoreRandomTools.nextDouble(random, 0.0, 360.0);
         double saturation = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double lightness = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);

         ColorDefinition color = ColorDefinition.hsl(hue, saturation, lightness);
         assertHSLAEquals(hue, saturation, lightness, 1.0, color, EPSILON);
         assertHSLEquals(hue, saturation, lightness, color, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test HSLA -> RGBA -> HSLA
         double hue = EuclidCoreRandomTools.nextDouble(random, 0.0, 360.0);
         double saturation = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double lightness = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);

         ColorDefinition color = ColorDefinition.hsla(hue, saturation, lightness, alpha);
         assertHSLAEquals(hue, saturation, lightness, alpha, color, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RGB -> HSL -> RGB
         double red = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double green = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double blue = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alpha = 1.0;

         ColorDefinition color = ColorDefinition.hsl(new ColorDefinition(red, green, blue).toHSLDoubleArray());
         assertRGBAEquals(red, green, blue, alpha, color, EPSILON);
         alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         color = ColorDefinition.hsla(new ColorDefinition(red, green, blue, alpha).toHSLADoubleArray());
         assertRGBAEquals(red, green, blue, alpha, color, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test varying red only
         double red = i / (ITERATIONS - 1.0);
         double green = 0.0;
         double blue = 0.0;

         double expectedHue = 0.0;
         double expectedSaturation = red == 0.0 ? 0.0 : 1.0;
         double expectedLightness = 0.5 * red;
         ColorDefinition color = new ColorDefinition(red, green, blue);
         assertHSLEquals(expectedHue, expectedSaturation, expectedLightness, color, EPSILON);

         green = 1.0;
         blue = 1.0;

         expectedHue = red == 1.0 ? 0.0 : 180.0;
         expectedSaturation = red == 1.0 ? 0.0 : 1.0;
         expectedLightness = 0.5 * (1.0 + red);
         color = new ColorDefinition(red, green, blue);
         assertHSLEquals(expectedHue, expectedSaturation, expectedLightness, color, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test varying green only
         double red = 0.0;
         double green = i / (ITERATIONS - 1.0);
         double blue = 0.0;

         double expectedHue = green == 0.0 ? 0.0 : 120.0;
         double expectedSaturation = green == 0.0 ? 0.0 : 1.0;
         double expectedLightness = 0.5 * green;
         ColorDefinition color = new ColorDefinition(red, green, blue);
         assertHSLEquals(expectedHue, expectedSaturation, expectedLightness, color, EPSILON);

         red = 1.0;
         blue = 1.0;

         expectedHue = green == 1.0 ? 0.0 : 300.0;
         expectedSaturation = green == 1.0 ? 0.0 : 1.0;
         expectedLightness = 0.5 * (1.0 + green);
         color = new ColorDefinition(red, green, blue);
         assertHSLEquals(expectedHue, expectedSaturation, expectedLightness, color, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test varying blue only
         double red = 0.0;
         double green = 0.0;
         double blue = i / (ITERATIONS - 1.0);

         double expectedHue = blue == 0.0 ? 0.0 : 240.0;
         double expectedSaturation = blue == 0.0 ? 0.0 : 1.0;
         double expectedLightness = 0.5 * blue;
         ColorDefinition color = new ColorDefinition(red, green, blue);
         assertHSLEquals(expectedHue, expectedSaturation, expectedLightness, color, EPSILON);

         red = 1.0;
         green = 1.0;

         expectedHue = blue == 1.0 ? 0.0 : 60.0;
         expectedSaturation = blue == 1.0 ? 0.0 : 1.0;
         expectedLightness = 0.5 * (1.0 + blue);
         color = new ColorDefinition(red, green, blue);
         assertHSLEquals(expectedHue, expectedSaturation, expectedLightness, color, EPSILON);
      }
   }

   @Test
   public void testBasicColors()
   {
      // https://en.wikipedia.org/wiki/Web_colors#Basic_colors
      assertRGBEquals(1.0, 1.0, 1.0, ColorDefinitions.White(), LARGE_EPSILON);
      assertHSLEquals(0.0, 0.0, 1.0, ColorDefinitions.White(), LARGE_EPSILON);
      assertHSBEquals(0.0, 0.0, 1.0, ColorDefinitions.White(), LARGE_EPSILON);

      assertRGBEquals(0.75, 0.75, 0.75, ColorDefinitions.Silver(), LARGE_EPSILON);
      assertHSLEquals(0.00, 0.00, 0.75, ColorDefinitions.Silver(), LARGE_EPSILON);
      assertHSBEquals(0.00, 0.00, 0.75, ColorDefinitions.Silver(), LARGE_EPSILON);

      assertRGBEquals(0.50, 0.50, 0.50, ColorDefinitions.Gray(), LARGE_EPSILON);
      assertHSLEquals(0.00, 0.00, 0.50, ColorDefinitions.Gray(), LARGE_EPSILON);
      assertHSBEquals(0.00, 0.00, 0.50, ColorDefinitions.Gray(), LARGE_EPSILON);

      assertRGBEquals(0.00, 0.00, 0.00, ColorDefinitions.Black(), LARGE_EPSILON);
      assertHSLEquals(0.00, 0.00, 0.00, ColorDefinitions.Black(), LARGE_EPSILON);
      assertHSBEquals(0.00, 0.00, 0.00, ColorDefinitions.Black(), LARGE_EPSILON);

      assertRGBEquals(1.00, 0.00, 0.00, ColorDefinitions.Red(), LARGE_EPSILON);
      assertHSLEquals(0.00, 1.00, 0.50, ColorDefinitions.Red(), LARGE_EPSILON);
      assertHSBEquals(0.00, 1.00, 1.00, ColorDefinitions.Red(), LARGE_EPSILON);

      assertRGBEquals(0.5, 0.0, 0.00, ColorDefinitions.Maroon(), LARGE_EPSILON);
      assertHSLEquals(0.0, 1.0, 0.25, ColorDefinitions.Maroon(), LARGE_EPSILON);
      assertHSBEquals(0.0, 1.0, 0.50, ColorDefinitions.Maroon(), LARGE_EPSILON);

      assertRGBEquals(01.00, 1.00, 0.00, ColorDefinitions.Yellow(), LARGE_EPSILON);
      assertHSLEquals(60.00, 1.00, 0.50, ColorDefinitions.Yellow(), LARGE_EPSILON);
      assertHSBEquals(60.00, 1.00, 1.00, ColorDefinitions.Yellow(), LARGE_EPSILON);

      assertRGBEquals(00.5, 0.5, 0.00, ColorDefinitions.Olive(), LARGE_EPSILON);
      assertHSLEquals(60.0, 1.0, 0.25, ColorDefinitions.Olive(), LARGE_EPSILON);
      assertHSBEquals(60.0, 1.0, 0.50, ColorDefinitions.Olive(), LARGE_EPSILON);

      assertRGBEquals(000.0, 1.0, 0.0, ColorDefinitions.Lime(), LARGE_EPSILON);
      assertHSLEquals(120.0, 1.0, 0.5, ColorDefinitions.Lime(), LARGE_EPSILON);
      assertHSBEquals(120.0, 1.0, 1.0, ColorDefinitions.Lime(), LARGE_EPSILON);

      assertRGBEquals(000.0, 0.5, 0.00, ColorDefinitions.Green(), LARGE_EPSILON);
      assertHSLEquals(120.0, 1.0, 0.25, ColorDefinitions.Green(), LARGE_EPSILON);
      assertHSBEquals(120.0, 1.0, 0.50, ColorDefinitions.Green(), LARGE_EPSILON);

      assertRGBEquals(000.0, 1.0, 1.0, ColorDefinitions.Aqua(), LARGE_EPSILON);
      assertHSLEquals(180.0, 1.0, 0.5, ColorDefinitions.Aqua(), LARGE_EPSILON);
      assertHSBEquals(180.0, 1.0, 1.0, ColorDefinitions.Aqua(), LARGE_EPSILON);

      assertRGBEquals(000.0, 0.5, 0.50, ColorDefinitions.Teal(), LARGE_EPSILON);
      assertHSLEquals(180.0, 1.0, 0.25, ColorDefinitions.Teal(), LARGE_EPSILON);
      assertHSBEquals(180.0, 1.0, 0.50, ColorDefinitions.Teal(), LARGE_EPSILON);

      assertRGBEquals(000.0, 0.5, 0.50, ColorDefinitions.Teal(), LARGE_EPSILON);
      assertHSLEquals(180.0, 1.0, 0.25, ColorDefinitions.Teal(), LARGE_EPSILON);
      assertHSBEquals(180.0, 1.0, 0.50, ColorDefinitions.Teal(), LARGE_EPSILON);

      assertRGBEquals(000.0, 0.0, 1.0, ColorDefinitions.Blue(), LARGE_EPSILON);
      assertHSLEquals(240.0, 1.0, 0.5, ColorDefinitions.Blue(), LARGE_EPSILON);
      assertHSBEquals(240.0, 1.0, 1.0, ColorDefinitions.Blue(), LARGE_EPSILON);

      assertRGBEquals(000.0, 0.0, 0.50, ColorDefinitions.Navy(), LARGE_EPSILON);
      assertHSLEquals(240.0, 1.0, 0.25, ColorDefinitions.Navy(), LARGE_EPSILON);
      assertHSBEquals(240.0, 1.0, 0.50, ColorDefinitions.Navy(), LARGE_EPSILON);

      assertRGBEquals(001.0, 0.0, 1.0, ColorDefinitions.Fuchsia(), LARGE_EPSILON);
      assertHSLEquals(300.0, 1.0, 0.5, ColorDefinitions.Fuchsia(), LARGE_EPSILON);
      assertHSBEquals(300.0, 1.0, 1.0, ColorDefinitions.Fuchsia(), LARGE_EPSILON);

      assertRGBEquals(000.5, 0.0, 0.50, ColorDefinitions.Purple(), LARGE_EPSILON);
      assertHSLEquals(300.0, 1.0, 0.25, ColorDefinitions.Purple(), LARGE_EPSILON);
      assertHSBEquals(300.0, 1.0, 0.50, ColorDefinitions.Purple(), LARGE_EPSILON);
   }

   @Test
   public void testParse()
   {
      Random random = new Random(234);

      for (int i = 0; i < ITERATIONS; i++)
      { // Without alpha
         ColorDefinition expected = ColorDefinition.rgb(random.nextInt(1 << 24));
         int rInt = expected.getRedAsInteger();
         int gInt = expected.getGreenAsInteger();
         int bInt = expected.getBlueAsInteger();
         double rDouble = expected.getRed();
         double gDouble = expected.getGreen();
         double bDouble = expected.getBlue();
         int rPercent = (int) (100.0 * rDouble);
         int gPercent = (int) (100.0 * gDouble);
         int bPercent = (int) (100.0 * bDouble);
         double hDouble = expected.getHue();
         double svDouble = expected.getSaturation();
         double vDouble = expected.getBrightness();
         double svPercent = 100.0 * svDouble;
         double vPercent = 100.0 * vDouble;
         double slDouble = expected.toHSLDoubleArray()[1];
         double lDouble = expected.toHSLDoubleArray()[2];
         double slPercent = 100.0 * slDouble;
         double lPercent = 100.0 * lDouble;

         // RGB(0, 128, 255)
         String rgb = "RGB(" + rInt + ", " + gInt + ", " + bInt + ")";
         assertColorDefinitionEquals(expected, ColorDefinition.parse(rgb.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(rgb.toLowerCase()), EPSILON);

         // RGB(0%, 50%, 100%)
         rgb = "RGB(" + rPercent + "%, " + gPercent + "%, " + bPercent + "%)";
         assertRGBAEquals(rPercent / 100.0, gPercent / 100.0, bPercent / 100.0, 1, ColorDefinition.parse(rgb.toUpperCase()), EPSILON);
         assertRGBAEquals(rPercent / 100.0, gPercent / 100.0, bPercent / 100.0, 1, ColorDefinition.parse(rgb.toLowerCase()), EPSILON);

         // HSV
         String hsv = "HSV(" + hDouble + ", " + svDouble + ", " + vDouble + ")";
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsv.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsv.toLowerCase()), EPSILON);
         hsv = "HSV(" + hDouble + ", " + svPercent + "%, " + vPercent + "%)";
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsv.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsv.toLowerCase()), EPSILON);
         // HSB
         String hsb = "HSB(" + hDouble + ", " + svDouble + ", " + vDouble + ")";
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsb.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsb.toLowerCase()), EPSILON);
         hsb = "HSB(" + hDouble + ", " + svPercent + "%, " + vPercent + "%)";
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsb.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsb.toLowerCase()), EPSILON);
         // HSL
         String hsl = "HSL(" + hDouble + ", " + slDouble + ", " + lDouble + ")";
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsl.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsl.toLowerCase()), EPSILON);
         hsl = "HSL(" + hDouble + ", " + slPercent + "%, " + lPercent + "%)";
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsl.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hsl.toLowerCase()), EPSILON);

         // Hex: 0x09FA3C
         String hex = String.format("0x%02x%02x%02x", rInt, gInt, bInt);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hex.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hex.toLowerCase()), EPSILON);

         // Hex: #09FA3C
         hex = String.format("#%02x%02x%02x", rInt, gInt, bInt);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hex.toUpperCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinition.parse(hex.toLowerCase()), EPSILON);
      }

      assertColorDefinitionEquals(ColorDefinitions.AliceBlue(), ColorDefinitions.parse("AliceBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.AntiqueWhite(), ColorDefinitions.parse("AntiqueWhite"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Aqua(), ColorDefinitions.parse("Aqua"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Aquamarine(), ColorDefinitions.parse("Aquamarine"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Azure(), ColorDefinitions.parse("Azure"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Beige(), ColorDefinitions.parse("Beige"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Bisque(), ColorDefinitions.parse("Bisque"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Black(), ColorDefinitions.parse("Black"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.BlanchedAlmond(), ColorDefinitions.parse("BlanchedAlmond"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Blue(), ColorDefinitions.parse("Blue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.BlueViolet(), ColorDefinitions.parse("BlueViolet"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Brown(), ColorDefinitions.parse("Brown"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.BurlyWood(), ColorDefinitions.parse("BurlyWood"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.CadetBlue(), ColorDefinitions.parse("CadetBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Chartreuse(), ColorDefinitions.parse("Chartreuse"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Chocolate(), ColorDefinitions.parse("Chocolate"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Coral(), ColorDefinitions.parse("Coral"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.CornflowerBlue(), ColorDefinitions.parse("CornflowerBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Cornsilk(), ColorDefinitions.parse("Cornsilk"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Crimson(), ColorDefinitions.parse("Crimson"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Cyan(), ColorDefinitions.parse("Cyan"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkBlue(), ColorDefinitions.parse("DarkBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkCyan(), ColorDefinitions.parse("DarkCyan"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkGoldenrod(), ColorDefinitions.parse("DarkGoldenrod"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkGray(), ColorDefinitions.parse("DarkGray"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkGreen(), ColorDefinitions.parse("DarkGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkGrey(), ColorDefinitions.parse("DarkGrey"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkKhaki(), ColorDefinitions.parse("DarkKhaki"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkMagenta(), ColorDefinitions.parse("DarkMagenta"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkOliveGreen(), ColorDefinitions.parse("DarkOliveGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkOrange(), ColorDefinitions.parse("DarkOrange"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkOrchid(), ColorDefinitions.parse("DarkOrchid"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkRed(), ColorDefinitions.parse("DarkRed"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkSalmon(), ColorDefinitions.parse("DarkSalmon"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkSeaGreen(), ColorDefinitions.parse("DarkSeaGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkSlateBlue(), ColorDefinitions.parse("DarkSlateBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkSlateGray(), ColorDefinitions.parse("DarkSlateGray"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkSlateGrey(), ColorDefinitions.parse("DarkSlateGrey"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkTurquoise(), ColorDefinitions.parse("DarkTurquoise"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DarkViolet(), ColorDefinitions.parse("DarkViolet"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DeepPink(), ColorDefinitions.parse("DeepPink"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DeepSkyBlue(), ColorDefinitions.parse("DeepSkyBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DimGray(), ColorDefinitions.parse("DimGray"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DimGrey(), ColorDefinitions.parse("DimGrey"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.DodgerBlue(), ColorDefinitions.parse("DodgerBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.FireBrick(), ColorDefinitions.parse("FireBrick"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.FloralWhite(), ColorDefinitions.parse("FloralWhite"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.ForestGreen(), ColorDefinitions.parse("ForestGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Fuchsia(), ColorDefinitions.parse("Fuchsia"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Gainsboro(), ColorDefinitions.parse("Gainsboro"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.GhostWhite(), ColorDefinitions.parse("GhostWhite"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Gold(), ColorDefinitions.parse("Gold"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Goldenrod(), ColorDefinitions.parse("Goldenrod"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Gray(), ColorDefinitions.parse("Gray"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Green(), ColorDefinitions.parse("Green"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.GreenYellow(), ColorDefinitions.parse("GreenYellow"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Grey(), ColorDefinitions.parse("Grey"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Honeydew(), ColorDefinitions.parse("Honeydew"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.HotPink(), ColorDefinitions.parse("HotPink"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.IndianRed(), ColorDefinitions.parse("IndianRed"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Indigo(), ColorDefinitions.parse("Indigo"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Ivory(), ColorDefinitions.parse("Ivory"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Khaki(), ColorDefinitions.parse("Khaki"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Lavender(), ColorDefinitions.parse("Lavender"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LavenderBlush(), ColorDefinitions.parse("LavenderBlush"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LawnGreen(), ColorDefinitions.parse("LawnGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LemonChiffon(), ColorDefinitions.parse("LemonChiffon"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightBlue(), ColorDefinitions.parse("LightBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightCoral(), ColorDefinitions.parse("LightCoral"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightCyan(), ColorDefinitions.parse("LightCyan"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightGoldenrodYellow(), ColorDefinitions.parse("LightGoldenrodYellow"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightGray(), ColorDefinitions.parse("LightGray"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightGreen(), ColorDefinitions.parse("LightGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightGrey(), ColorDefinitions.parse("LightGrey"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightPink(), ColorDefinitions.parse("LightPink"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightSalmon(), ColorDefinitions.parse("LightSalmon"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightSeaGreen(), ColorDefinitions.parse("LightSeaGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightSkyBlue(), ColorDefinitions.parse("LightSkyBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightSlateGray(), ColorDefinitions.parse("LightSlateGray"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightSlateGrey(), ColorDefinitions.parse("LightSlateGrey"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightSteelBlue(), ColorDefinitions.parse("LightSteelBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LightYellow(), ColorDefinitions.parse("LightYellow"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Lime(), ColorDefinitions.parse("Lime"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.LimeGreen(), ColorDefinitions.parse("LimeGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Linen(), ColorDefinitions.parse("Linen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Magenta(), ColorDefinitions.parse("Magenta"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Maroon(), ColorDefinitions.parse("Maroon"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumAquamarine(), ColorDefinitions.parse("MediumAquamarine"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumBlue(), ColorDefinitions.parse("MediumBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumOrchid(), ColorDefinitions.parse("MediumOrchid"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumPurple(), ColorDefinitions.parse("MediumPurple"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumSeaGreen(), ColorDefinitions.parse("MediumSeaGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumSlateBlue(), ColorDefinitions.parse("MediumSlateBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumSpringGreen(), ColorDefinitions.parse("MediumSpringGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumTurquoise(), ColorDefinitions.parse("MediumTurquoise"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MediumVioletRed(), ColorDefinitions.parse("MediumVioletRed"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MidnightBlue(), ColorDefinitions.parse("MidnightBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MintCream(), ColorDefinitions.parse("MintCream"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.MistyRose(), ColorDefinitions.parse("MistyRose"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Moccasin(), ColorDefinitions.parse("Moccasin"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.NavajoWhite(), ColorDefinitions.parse("NavajoWhite"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Navy(), ColorDefinitions.parse("Navy"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.OldLace(), ColorDefinitions.parse("OldLace"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Olive(), ColorDefinitions.parse("Olive"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.OliveDrab(), ColorDefinitions.parse("OliveDrab"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Orange(), ColorDefinitions.parse("Orange"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.OrangeRed(), ColorDefinitions.parse("OrangeRed"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Orchid(), ColorDefinitions.parse("Orchid"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.PaleGoldenrod(), ColorDefinitions.parse("PaleGoldenrod"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.PaleGreen(), ColorDefinitions.parse("PaleGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.PaleTurquoise(), ColorDefinitions.parse("PaleTurquoise"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.PaleVioletRed(), ColorDefinitions.parse("PaleVioletRed"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.PapayaWhip(), ColorDefinitions.parse("PapayaWhip"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.PeachPuff(), ColorDefinitions.parse("PeachPuff"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Peru(), ColorDefinitions.parse("Peru"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Pink(), ColorDefinitions.parse("Pink"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Plum(), ColorDefinitions.parse("Plum"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.PowderBlue(), ColorDefinitions.parse("PowderBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Purple(), ColorDefinitions.parse("Purple"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Red(), ColorDefinitions.parse("Red"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.RosyBrown(), ColorDefinitions.parse("RosyBrown"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.RoyalBlue(), ColorDefinitions.parse("RoyalBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SaddleBrown(), ColorDefinitions.parse("SaddleBrown"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Salmon(), ColorDefinitions.parse("Salmon"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SandyBrown(), ColorDefinitions.parse("SandyBrown"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SeaGreen(), ColorDefinitions.parse("SeaGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SeaShell(), ColorDefinitions.parse("SeaShell"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Sienna(), ColorDefinitions.parse("Sienna"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Silver(), ColorDefinitions.parse("Silver"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SkyBlue(), ColorDefinitions.parse("SkyBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SlateBlue(), ColorDefinitions.parse("SlateBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SlateGray(), ColorDefinitions.parse("SlateGray"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SlateGrey(), ColorDefinitions.parse("SlateGrey"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Snow(), ColorDefinitions.parse("Snow"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SpringGreen(), ColorDefinitions.parse("SpringGreen"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.SteelBlue(), ColorDefinitions.parse("SteelBlue"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Tan(), ColorDefinitions.parse("Tan"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Teal(), ColorDefinitions.parse("Teal"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Thistle(), ColorDefinitions.parse("Thistle"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Tomato(), ColorDefinitions.parse("Tomato"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Turquoise(), ColorDefinitions.parse("Turquoise"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Violet(), ColorDefinitions.parse("Violet"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Wheat(), ColorDefinitions.parse("Wheat"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.White(), ColorDefinitions.parse("White"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.WhiteSmoke(), ColorDefinitions.parse("WhiteSmoke"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.Yellow(), ColorDefinitions.parse("Yellow"), EPSILON);
      assertColorDefinitionEquals(ColorDefinitions.YellowGreen(), ColorDefinitions.parse("YellowGreen"), EPSILON);

      for (String colorName : ColorDefinitions.namedColorLowerCaseMap.keySet())
      {
         ColorDefinition expected = ColorDefinitions.namedColorLowerCaseMap.get(colorName);
         assertColorDefinitionEquals(expected, ColorDefinitions.parse(colorName.toLowerCase()), EPSILON);
         assertColorDefinitionEquals(expected, ColorDefinitions.parse(colorName.toLowerCase()), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test parsing toString output
         ColorDefinition original = DefinitionRandomTools.nextColorDefinition(random);
         ColorDefinition parsed = ColorDefinition.parse(original.toString());
         assertEquals(original, parsed);
      }
   }

   public static void assertColorDefinitionEquals(ColorDefinition expectedColor, ColorDefinition actualColor, double epsilon)
   {
      assertRGBAEquals(expectedColor.getRed(),
                       expectedColor.getGreen(),
                       expectedColor.getBlue(),
                       expectedColor.getAlpha(),
                       actualColor.getRed(),
                       actualColor.getGreen(),
                       actualColor.getBlue(),
                       actualColor.getAlpha(),
                       epsilon);
   }

   public static void assertHSBEquals(double expectedHue, double expectedSaturation, double expectedBrightness, double[] actualHSB, double epsilon)
   {
      assertHSBEquals(expectedHue, expectedSaturation, expectedBrightness, actualHSB[0], actualHSB[1], actualHSB[2], epsilon);
   }

   public static void assertHSBEquals(double expectedHue, double expectedSaturation, double expectedBrightness, ColorDefinition actualColor, double epsilon)
   {
      assertHSBEquals(expectedHue,
                      expectedSaturation,
                      expectedBrightness,
                      actualColor.getHue(),
                      actualColor.getSaturation(),
                      actualColor.getBrightness(),
                      epsilon);
   }

   private static void assertHSBEquals(double expectedHue,
                                       double expectedSaturation,
                                       double expectedBrightness,
                                       double actualHue,
                                       double actualSaturation,
                                       double actualBrightness,
                                       double epsilon)
         throws AssertionFailedError
   {
      boolean equals = EuclidCoreTools.epsilonEquals(expectedHue, actualHue, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedSaturation, actualSaturation, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedBrightness, actualBrightness, epsilon);

      if (!equals)
      {
         String message = String.format("expected:\n\t<%s>\nwas:\n\t<%s>\ndifference:\n\t<%s>",
                                        hsbToString(expectedHue, expectedSaturation, expectedBrightness),
                                        hsbToString(actualHue, actualSaturation, actualBrightness),
                                        hsbToString(Math.abs(expectedHue - actualHue),
                                                    Math.abs(expectedSaturation - actualSaturation),
                                                    Math.abs(expectedBrightness - actualBrightness)));

         throw new AssertionFailedError(message);
      }
   }

   public static void assertHSBAEquals(double expectedHue,
                                       double expectedSaturation,
                                       double expectedBrightness,
                                       double expectedAlpha,
                                       double[] actualHSBA,
                                       double epsilon)
   {
      assertHSBAEquals(expectedHue, expectedSaturation, expectedBrightness, expectedAlpha, actualHSBA[0], actualHSBA[1], actualHSBA[2], actualHSBA[3], epsilon);
   }

   public static void assertHSBAEquals(double expectedHue,
                                       double expectedSaturation,
                                       double expectedBrightness,
                                       double expectedAlpha,
                                       ColorDefinition actualColor,
                                       double epsilon)
   {
      assertHSBAEquals(expectedHue,
                       expectedSaturation,
                       expectedBrightness,
                       expectedAlpha,
                       actualColor.getHue(),
                       actualColor.getSaturation(),
                       actualColor.getBrightness(),
                       actualColor.getAlpha(),
                       epsilon);
   }

   private static void assertHSBAEquals(double expectedHue,
                                        double expectedSaturation,
                                        double expectedBrightness,
                                        double expectedAlpha,
                                        double actualHue,
                                        double actualSaturation,
                                        double actualBrightness,
                                        double actualAlpha,
                                        double epsilon)
         throws AssertionFailedError
   {
      boolean equals = EuclidCoreTools.epsilonEquals(expectedHue, actualHue, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedSaturation, actualSaturation, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedBrightness, actualBrightness, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedAlpha, actualAlpha, epsilon);

      if (!equals)
      {
         String message = String.format("expected:\n\t<%s>\nwas:\n\t<%s>\ndifference:\n\t<%s>",
                                        hsbaToString(expectedHue, expectedSaturation, expectedBrightness, expectedAlpha),
                                        hsbaToString(actualHue, actualSaturation, actualBrightness, actualAlpha),
                                        hsbaToString(Math.abs(expectedHue - actualHue),
                                                     Math.abs(expectedSaturation - actualSaturation),
                                                     Math.abs(expectedBrightness - actualBrightness),
                                                     Math.abs(expectedAlpha - actualAlpha)));

         throw new AssertionFailedError(message);
      }
   }

   public static void assertHSLEquals(double expectedHue, double expectedSaturation, double expectedLightness, double[] actualHSL, double epsilon)
   {
      assertHSLEquals(expectedHue, expectedSaturation, expectedLightness, actualHSL[0], actualHSL[1], actualHSL[2], epsilon);
   }

   public static void assertHSLEquals(double expectedHue, double expectedSaturation, double expectedLightness, ColorDefinition actualColor, double epsilon)
   {
      assertHSLEquals(expectedHue, expectedSaturation, expectedLightness, actualColor.toHSLDoubleArray(), epsilon);
   }

   private static void assertHSLEquals(double expectedHue,
                                       double expectedSaturation,
                                       double expectedLightness,
                                       double actualHue,
                                       double actualSaturation,
                                       double actualLightness,
                                       double epsilon)
         throws AssertionFailedError
   {
      boolean equals = EuclidCoreTools.epsilonEquals(expectedHue, actualHue, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedSaturation, actualSaturation, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedLightness, actualLightness, epsilon);

      if (!equals)
      {
         String message = String.format("expected:\n\t<%s>\nwas:\n\t<%s>\ndifference:\n\t<%s>",
                                        hslToString(expectedHue, expectedSaturation, expectedLightness),
                                        hslToString(actualHue, actualSaturation, actualLightness),
                                        hslToString(Math.abs(expectedHue - actualHue),
                                                    Math.abs(expectedSaturation - actualSaturation),
                                                    Math.abs(expectedLightness - actualLightness)));

         throw new AssertionFailedError(message);
      }
   }

   public static void assertHSLAEquals(double expectedHue,
                                       double expectedSaturation,
                                       double expectedLightness,
                                       double expectedAlpha,
                                       double[] actualHSLA,
                                       double epsilon)
   {
      assertHSLAEquals(expectedHue, expectedSaturation, expectedLightness, expectedAlpha, actualHSLA[0], actualHSLA[1], actualHSLA[2], actualHSLA[3], epsilon);
   }

   public static void assertHSLAEquals(double expectedHue,
                                       double expectedSaturation,
                                       double expectedLightness,
                                       double expectedAlpha,
                                       ColorDefinition actualColor,
                                       double epsilon)
   {
      assertHSLAEquals(expectedHue, expectedSaturation, expectedLightness, expectedAlpha, actualColor.toHSLADoubleArray(), epsilon);
   }

   private static void assertHSLAEquals(double expectedHue,
                                        double expectedSaturation,
                                        double expectedLightness,
                                        double expectedAlpha,
                                        double actualHue,
                                        double actualSaturation,
                                        double actualLightness,
                                        double actualAlpha,
                                        double epsilon)
         throws AssertionFailedError
   {
      boolean equals = EuclidCoreTools.epsilonEquals(expectedHue, actualHue, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedSaturation, actualSaturation, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedLightness, actualLightness, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedAlpha, actualAlpha, epsilon);

      if (!equals)
      {
         String message = String.format("expected:\n\t<%s>\nwas:\n\t<%s>\ndifference:\n\t<%s>",
                                        hslaToString(expectedHue, expectedSaturation, expectedLightness, expectedAlpha),
                                        hslaToString(actualHue, actualSaturation, actualLightness, actualAlpha),
                                        hslaToString(Math.abs(expectedHue - actualHue),
                                                     Math.abs(expectedSaturation - actualSaturation),
                                                     Math.abs(expectedLightness - actualLightness),
                                                     Math.abs(expectedAlpha - actualAlpha)));

         throw new AssertionFailedError(message);
      }
   }

   public static void assertRGBEquals(double expectedRed, double expectedGreen, double expectedBlue, double[] actualRGB, double epsilon)
   {
      assertRGBEquals(expectedRed, expectedGreen, expectedBlue, actualRGB[0], actualRGB[1], actualRGB[2], epsilon);
   }

   public static void assertRGBEquals(double expectedRed, double expectedGreen, double expectedBlue, ColorDefinition actualColor, double epsilon)
   {
      assertRGBEquals(expectedRed, expectedGreen, expectedBlue, actualColor.getRed(), actualColor.getGreen(), actualColor.getBlue(), epsilon);
   }

   public static void assertRGBEquals(double expectedRed,
                                      double expectedGreen,
                                      double expectedBlue,
                                      double actualRed,
                                      double actualGreen,
                                      double actualBlue,
                                      double epsilon)
         throws AssertionFailedError
   {
      boolean equals = EuclidCoreTools.epsilonEquals(expectedRed, actualRed, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedGreen, actualGreen, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedBlue, actualBlue, epsilon);

      if (!equals)
      {
         String message = String.format("expected:\n\t<%s>\nwas:\n\t<%s>\ndifference:\n\t<%s>",
                                        rgbToString(expectedRed, expectedGreen, expectedBlue),
                                        rgbToString(actualRed, actualGreen, actualBlue),
                                        rgbToString(Math.abs(expectedRed - actualRed),
                                                    Math.abs(expectedGreen - actualGreen),
                                                    Math.abs(expectedBlue - actualBlue)));

         throw new AssertionFailedError(message);
      }
   }

   public static void assertRGBAEquals(double expectedRed, double expectedGreen, double expectedBlue, double expectedAlpha, double[] actualRGBA, double epsilon)
   {
      assertRGBAEquals(expectedRed, expectedGreen, expectedBlue, expectedAlpha, actualRGBA[0], actualRGBA[1], actualRGBA[2], actualRGBA[3], epsilon);
   }

   public static void assertRGBAEquals(double expectedRed,
                                       double expectedGreen,
                                       double expectedBlue,
                                       double expectedAlpha,
                                       ColorDefinition actualColor,
                                       double epsilon)
   {
      assertRGBAEquals(expectedRed,
                       expectedGreen,
                       expectedBlue,
                       expectedAlpha,
                       actualColor.getRed(),
                       actualColor.getGreen(),
                       actualColor.getBlue(),
                       actualColor.getAlpha(),
                       epsilon);
   }

   public static void assertRGBAEquals(double expectedRed,
                                       double expectedGreen,
                                       double expectedBlue,
                                       double expectedAlpha,
                                       double actualRed,
                                       double actualGreen,
                                       double actualBlue,
                                       double actualAlpha,
                                       double epsilon)
         throws AssertionFailedError
   {
      boolean equals = EuclidCoreTools.epsilonEquals(expectedRed, actualRed, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedGreen, actualGreen, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedBlue, actualBlue, epsilon);
      equals &= EuclidCoreTools.epsilonEquals(expectedAlpha, actualAlpha, epsilon);

      if (!equals)
      {
         String message = String.format("expected:\n\t<%s>\nwas:\n\t<%s>\ndifference:\n\t<%s>",
                                        rgbaToString(expectedRed, expectedGreen, expectedBlue, expectedAlpha),
                                        rgbaToString(actualRed, actualGreen, actualBlue, actualAlpha),
                                        rgbaToString(Math.abs(expectedRed - actualRed),
                                                     Math.abs(expectedGreen - actualGreen),
                                                     Math.abs(expectedBlue - actualBlue),
                                                     Math.abs(expectedAlpha - actualAlpha)));

         throw new AssertionFailedError(message);
      }
   }

   public static void assertRGBEquals(int expectedRed, int expectedGreen, int expectedBlue, int[] actualRGB)
   {
      assertRGBEquals(expectedRed, expectedGreen, expectedBlue, actualRGB[0], actualRGB[1], actualRGB[2]);
   }

   public static void assertRGBEquals(int expectedRed, int expectedGreen, int expectedBlue, ColorDefinition actualColor)
   {
      assertRGBEquals(expectedRed, expectedGreen, expectedBlue, actualColor.getRedAsInteger(), actualColor.getGreenAsInteger(), actualColor.getBlueAsInteger());
   }

   public static void assertRGBEquals(int expectedRed, int expectedGreen, int expectedBlue, int actualRed, int actualGreen, int actualBlue)
         throws AssertionFailedError
   {
      boolean equals = expectedRed == actualRed;
      equals &= expectedGreen == actualGreen;
      equals &= expectedBlue == actualBlue;

      if (!equals)
      {
         String message = String.format("expected:\n\t<%s>\nwas:\n\t<%s>\ndifference:\n\t<%s>",
                                        rgbToString(expectedRed, expectedGreen, expectedBlue),
                                        rgbToString(actualRed, actualGreen, actualBlue),
                                        rgbToString(Math.abs(expectedRed - actualRed),
                                                    Math.abs(expectedGreen - actualGreen),
                                                    Math.abs(expectedBlue - actualBlue)));

         throw new AssertionFailedError(message);
      }
   }

   public static void assertRGBAEquals(int expectedRed, int expectedGreen, int expectedBlue, int expectedAlpha, int[] actualRGBA)
   {
      assertRGBAEquals(expectedRed, expectedGreen, expectedBlue, expectedAlpha, actualRGBA[0], actualRGBA[1], actualRGBA[2], actualRGBA[3]);
   }

   public static void assertRGBAEquals(int expectedRed, int expectedGreen, int expectedBlue, int expectedAlpha, ColorDefinition actualColor)
   {
      assertRGBAEquals(expectedRed,
                       expectedGreen,
                       expectedBlue,
                       expectedAlpha,
                       actualColor.getRedAsInteger(),
                       actualColor.getGreenAsInteger(),
                       actualColor.getBlueAsInteger(),
                       actualColor.getAlphaAsInteger());
   }

   public static void assertRGBAEquals(int expectedRed,
                                       int expectedGreen,
                                       int expectedBlue,
                                       int expectedAlpha,
                                       int actualRed,
                                       int actualGreen,
                                       int actualBlue,
                                       int actualAlpha)
         throws AssertionFailedError
   {
      boolean equals = expectedRed == actualRed;
      equals &= expectedGreen == actualGreen;
      equals &= expectedBlue == actualBlue;
      equals &= expectedAlpha == actualAlpha;

      if (!equals)
      {
         String message = String.format("expected:\n\t<%s>\nwas:\n\t<%s>\ndifference:\n\t<%s>",
                                        rgbaToString(expectedRed, expectedGreen, expectedBlue, expectedAlpha),
                                        rgbaToString(actualRed, actualGreen, actualBlue, actualAlpha),
                                        rgbaToString(Math.abs(expectedRed - actualRed),
                                                     Math.abs(expectedGreen - actualGreen),
                                                     Math.abs(expectedBlue - actualBlue),
                                                     Math.abs(expectedAlpha - actualAlpha)));

         throw new AssertionFailedError(message);
      }
   }

   private static String hsbToString(double hue, double saturation, double brightness)
   {
      return "H=" + hue + ", S=" + saturation + ", B=" + brightness;
   }

   private static String hsbaToString(double hue, double saturation, double brightness, double alpha)
   {
      return hsbToString(hue, saturation, brightness) + ", A=" + alpha;
   }

   private static String hslToString(double hue, double saturation, double lightness)
   {
      return "H=" + hue + ", S=" + saturation + ", L=" + lightness;
   }

   private static String hslaToString(double hue, double saturation, double lightness, double alpha)
   {
      return hslToString(hue, saturation, lightness) + ", A=" + alpha;
   }

   private static String rgbToString(double red, double green, double blue)
   {
      return "R=" + red + ", G=" + green + ", B=" + blue;
   }

   private static String rgbaToString(double red, double green, double blue, double alpha)
   {
      return rgbToString(red, green, blue) + ", A=" + alpha;
   }
}
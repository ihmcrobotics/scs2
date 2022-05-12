package us.ihmc.scs2.definition.visual;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D32;

/**
 * Provides a HSB-based color palette. As an image is 2D, one of the three components, i.e.
 * hue/saturation/brightness, has to be constant.
 * <p>
 * Note: This implementation of {@link TextureDefinitionColorPalette} is affected by a bug in
 * JavaFX:
 * <p>
 * <b> When all the texture coordinates of a mesh have the same y-coordinate, the latter gets
 * ignored and replaced with the value {@code 0.5f} (tested with JDK 1.8.0_112). </b>
 * </p>
 * When affected, the resulting colors of a mesh won't be as accurate as usual.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class TextureDefinitionColorPalette2D implements TextureDefinitionColorPalette
{
   /**
    * Debug variable. When set to true, the {@link BufferedImage} used in this color palette is printed
    * as a png file.
    */
   private static final boolean PRINT_PALETTE = false;
   private static final int DEFAULT_RESOLUTION = 256;

   private int hueResolution = -1;
   private int saturationResolution = -1;
   private int brightnessResolution = -1;

   private double hueConstant = Double.NaN;
   private double saturationConstant = Double.NaN;
   private double brightnessConstant = Double.NaN;

   private BufferedImage colorPalette;

   /**
    * Creates a color palette with the brightness constant and set to {@code 1.0}. The two other
    * components have a resolution of {@value #DEFAULT_RESOLUTION}.
    */
   public TextureDefinitionColorPalette2D()
   {
      setHueSaturationBased(1.0);
   }

   /**
    * Changes this color palette to allow variation in hue and saturation component.
    *
    * @param brightnessConstant the new constant value for the brightness component.
    */
   public void setHueSaturationBased(double brightnessConstant)
   {
      setHueSaturationBased(DEFAULT_RESOLUTION, DEFAULT_RESOLUTION, brightnessConstant);
   }

   /**
    * Changes this color palette to allow variation in hue and saturation.
    *
    * @param hueResolution        the new resolution to use for the hue component.
    * @param saturationResolution the new resolution to use for the saturation component.
    * @param brightnessConstant   the new constant value for the brightness component.
    */
   public void setHueSaturationBased(int hueResolution, int saturationResolution, double brightnessConstant)
   {
      MathTools.checkGreaterThanOrEquals(hueResolution, 1);
      MathTools.checkGreaterThanOrEquals(saturationResolution, 1);
      MathTools.checkIntervalContains(brightnessConstant, 0.0, 1.0);

      this.hueResolution = hueResolution;
      this.saturationResolution = saturationResolution;
      brightnessResolution = -1;

      hueConstant = Double.NaN;
      saturationConstant = Double.NaN;
      this.brightnessConstant = brightnessConstant;

      updateColorPalette();
   }

   /**
    * Changes this color palette to allow variation in hue and brightness. The two other components
    * have a resolution of {@value #DEFAULT_RESOLUTION}.
    *
    * @param saturationConstant the new constant value for the saturation component.
    */
   public void setHueBrightnessBased(double saturationConstant)
   {
      setHueBrightnessBased(DEFAULT_RESOLUTION, DEFAULT_RESOLUTION, saturationConstant);
   }

   /**
    * Changes this color palette to allow variation in hue and saturation.
    *
    * @param hueResolution        the new resolution to use for the hue component.
    * @param brightnessResolution the new resolution to use for the brightness component.
    * @param saturationConstant   the new constant value for the saturation component.
    */
   public void setHueBrightnessBased(int hueResolution, int brightnessResolution, double saturationConstant)
   {
      MathTools.checkGreaterThanOrEquals(hueResolution, 1);
      MathTools.checkGreaterThanOrEquals(brightnessResolution, 1);
      MathTools.checkIntervalContains(saturationConstant, 0.0, 1.0);

      this.hueResolution = hueResolution;
      saturationResolution = -1;
      this.brightnessResolution = brightnessResolution;

      hueConstant = Double.NaN;
      this.saturationConstant = saturationConstant;
      brightnessConstant = Double.NaN;

      updateColorPalette();
   }

   /**
    * Changes this color palette to allow variation in saturation and brightness. The two other
    * components have a resolution of {@value #DEFAULT_RESOLUTION}.
    *
    * @param hueConstant the new constant value for the hue component.
    */
   public void setSaturationBrightnessBased(double hueConstant)
   {
      setSaturationBrightnessBased(DEFAULT_RESOLUTION, DEFAULT_RESOLUTION, hueConstant);
   }

   /**
    * Changes this color palette to allow variation in saturation and brightness.
    *
    * @param saturationResolution the new resolution to use for the saturation component.
    * @param brightnessResolution the new resolution to use for the brightness component.
    * @param hueConstant          the new constant value for the hue component.
    */
   public void setSaturationBrightnessBased(int saturationResolution, int brightnessResolution, double hueConstant)
   {
      MathTools.checkGreaterThanOrEquals(saturationResolution, 1);
      MathTools.checkGreaterThanOrEquals(brightnessResolution, 1);
      MathTools.checkIntervalContains(hueConstant, 0.0, 1.0);

      hueResolution = -1;
      this.saturationResolution = saturationResolution;
      this.brightnessResolution = brightnessResolution;

      this.hueConstant = hueConstant;
      saturationConstant = Double.NaN;
      brightnessConstant = Double.NaN;

      updateColorPalette();
   }

   private void updateColorPalette()
   {
      int width = hueResolution != -1 ? hueResolution : saturationResolution;
      int height = brightnessResolution != -1 ? brightnessResolution : saturationResolution;

      BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);

      for (int x = 0; x < width; x++)
      {
         for (int y = 0; y < height; y++)
         {
            image.setRGB(x, y, getColorAtLocation(x, y));
         }
      }

      if (PRINT_PALETTE)
      {
         // save for testing purposes
         try
         {
            ImageIO.write(image, "png", new File("palette.png"));
         }
         catch (IOException ex)
         {
         }
      }

      colorPalette = image;
   }

   private int getColorAtLocation(int x, int y)
   {
      double hue;
      if (hueResolution != -1)
         hue = 360.0 * x / hueResolution;
      else
         hue = 360.0 * hueConstant;

      double saturation;
      if (saturationResolution != -1)
      {
         int index = hueResolution == -1 ? x : y;
         saturation = (double) index / (double) saturationResolution;
      }
      else
         saturation = saturationConstant;

      double brightness;
      if (brightnessResolution != -1)
         brightness = (double) y / (double) brightnessResolution;
      else
         brightness = brightnessConstant;

      return ColorDefinitions.hsb(hue, saturation, brightness).toRGB();
   }

   @Override
   public Point2D32 getTextureLocation(ColorDefinition color)
   {
      float x = (float) (hueResolution != -1 ? color.getHue() / 360.0 : color.getSaturation());
      float y = (float) (brightnessResolution != -1 ? color.getBrightness() : color.getSaturation());

      return new Point2D32(x, y);
   }

   @Override
   public TextureDefinition getTextureDefinition()
   {
      return new TextureDefinition(colorPalette);
   }
}

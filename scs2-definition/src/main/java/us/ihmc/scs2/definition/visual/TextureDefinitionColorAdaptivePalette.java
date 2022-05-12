package us.ihmc.scs2.definition.visual;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;

import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.log.LogTools;

/**
 * Provides an color palette that expands as new colors are being accessed instead of being
 * pre-generated as in {@link TextureDefinitionColorPalette2D}. The main advantage over the other
 * color palettes is the guarantee of an exact restitution of the colors. However, the approximate
 * number of colors that will be used needs to be known in advance as the color palette has a fixed
 * size and only a certain number of colors can be registered. By default, the image used by this
 * color palette is automatically cleared every time {@link #getTextureDefinition()} is called. This
 * behavior can cause a noticeable slow down of the rendering thread when creating numerous
 * textures. In such scenario, it is preferred to initialize {@link #autoClear} to false and to
 * reuse as much as possible the same palette.
 * 
 * @author Sylvain Bertrand
 */
public class TextureDefinitionColorAdaptivePalette implements TextureDefinitionColorPalette
{
   /**
    * Debug variable. When set to true, the {@link BufferedImage} used in this color palette is printed
    * as a png file.
    */
   private static final boolean PRINT_PALETTE = false;
   private static final int DEFAULT_PALETTE_SIZE = 1024;

   private BufferedImage colorPalette;
   private int paletteSize;
   private int pixelSize;

   private final List<ColorDefinition> colorList = new ArrayList<>();
   private final Map<ColorDefinition, Integer> registeredColors = new HashMap<>();
   private final boolean autoClear;

   /**
    * Creates a default color palette of size {@value #DEFAULT_PALETTE_SIZE} *
    * {@value #DEFAULT_PALETTE_SIZE}.
    */
   public TextureDefinitionColorAdaptivePalette()
   {
      this(DEFAULT_PALETTE_SIZE, 1, true);
   }

   /**
    * Creates a color palette of size {@value #DEFAULT_PALETTE_SIZE} * {@value #DEFAULT_PALETTE_SIZE}.
    * 
    * @param autoClear defines how this palette should behave:
    *                  <li>{@code true}: the {@link BufferedImage} used by this palette is
    *                  automatically cleared when {@link #getTextureDefinition()} is called (this is
    *                  preferred option in most situations),
    *                  <li>{@code false}: the {@link BufferedImage} has to be cleared manually (this is
    *                  preferred option when numerous images have to be created).
    */
   public TextureDefinitionColorAdaptivePalette(boolean autoClear)
   {
      this(DEFAULT_PALETTE_SIZE, 1, autoClear);
   }

   /**
    * Creates a color palette with the given size.
    * 
    * @param paletteSize size of the palette, the number of different colors will be:
    *                    {@code paletteSize * paletteSize}.
    */
   public TextureDefinitionColorAdaptivePalette(int paletteSize)
   {
      this(paletteSize, 1, true);
   }

   /**
    * Creates a color palette.
    * 
    * @param paletteSize size of the palette, the number of different colors will be:
    *                    {@code paletteSize * paletteSize}.
    * @param autoClear   defines how this palette should behave:
    *                    <li>{@code true}: the {@link BufferedImage} used by this palette is
    *                    automatically cleared when {@link #getTextureDefinition()} is called (this is
    *                    preferred option in most situations),
    *                    <li>{@code false}: the {@link BufferedImage} has to be cleared manually (this
    *                    is preferred option when numerous images have to be created).
    */
   public TextureDefinitionColorAdaptivePalette(int paletteSize, boolean autoClear)
   {
      this(paletteSize, 1, autoClear);
   }

   /**
    * Creates a color palette.
    * 
    * @param paletteSize    size of the palette, the number of different colors will be:
    *                       {@code paletteSize * paletteSize}.
    * @param colorPixelSize size of the color pixels in the image to be used. A size of {@code 1} seems
    *                       to be working perfectly fine.
    * @param autoClear      defines how this palette should behave:
    *                       <li>{@code true}: the {@link BufferedImage} used by this palette is
    *                       automatically cleared when {@link #getTextureDefinition()} is called (this
    *                       is preferred option in most situations),
    *                       <li>{@code false}: the {@link BufferedImage} has to be cleared manually
    *                       (this is preferred option when numerous images have to be created).
    */
   public TextureDefinitionColorAdaptivePalette(int paletteSize, int colorPixelSize, boolean autoClear)
   {
      if (colorPixelSize >= paletteSize)
         throw new RuntimeException("The size of pixels has to be smaller than the size of the palette.");

      this.autoClear = autoClear;
      this.paletteSize = paletteSize;
      pixelSize = colorPixelSize;
   }

   /**
    * Clears the image currently in use by this color palette. Allows to empty the buffer in order to
    * register new colors.
    * <p>
    * Note that when {@link #autoClear} is set to true, the palette will automatically be cleared.
    * </p>
    */
   public void clearPalette()
   {
      colorPalette = null;
      colorList.clear();
      registeredColors.clear();
   }

   private void initializePalette()
   {
      colorPalette = new BufferedImage(paletteSize, paletteSize, BufferedImage.TYPE_INT_ARGB);
      colorList.clear();
      registeredColors.clear();
   }

   @Override
   public Point2D32 getTextureLocation(ColorDefinition color)
   {
      if (colorPalette == null)
         initializePalette();

      Integer pixelIndex = registeredColors.get(color);
      if (pixelIndex != null)
      {
         return getTextureLocation(pixelIndex);
      }
      else
      {
         int newPixelIndex = colorList.size();
         if (newPixelIndex <= paletteSize * paletteSize)
         {
            colorList.add(color);
            registeredColors.put(color, newPixelIndex);
            writePixel(newPixelIndex, color);
            return getTextureLocation(newPixelIndex);
         }
         else
         {
            LogTools.error("Reached maximum capacity of the palette. Next colors will be wrong.");
            return new Point2D32();
         }
      }
   }

   private Point2D32 getTextureLocation(int pixelIndex)
   {
      int scaledWidth = (int) colorPalette.getWidth() / pixelSize;
      int scaledHeight = (int) colorPalette.getHeight() / pixelSize;
      float x = (float) (Math.floorMod(pixelIndex, scaledWidth)) / (float) scaledWidth;
      x += 0.5f * pixelSize / (float) colorPalette.getWidth();
      float y = (float) (Math.floorDiv(pixelIndex, scaledWidth)) / (float) scaledHeight;
      y += 0.5f * pixelSize / (float) colorPalette.getHeight();
      return new Point2D32(x, y);
   }

   private void writePixel(int pixelIndex, ColorDefinition color)
   {
      int width = colorPalette.getWidth();
      int xStart = Math.floorMod(pixelIndex * pixelSize, width);
      int yStart = Math.floorDiv(pixelIndex * pixelSize, width) * pixelSize;
      for (int x = xStart; x < xStart + pixelSize; x++)
      {
         for (int y = yStart; y < yStart + pixelSize; y++)
         {
            colorPalette.setRGB(x, y, color.toRGB());
         }
      }
   }

   /**
    * Note that calling this method will clear the palette when {@link #autoClear} is equal to true.
    */
   @Override
   public TextureDefinition getTextureDefinition()
   {
      BufferedImage ret = colorPalette;
      if (PRINT_PALETTE)
      {
         // save for testing purposes
         try
         {
            ImageIO.write(colorPalette, "png", new File("palette.png"));
         }
         catch (IOException ex)
         {
            ex.printStackTrace();
         }
      }
      if (autoClear)
         clearPalette();
      return new TextureDefinition(ret);
   }
}

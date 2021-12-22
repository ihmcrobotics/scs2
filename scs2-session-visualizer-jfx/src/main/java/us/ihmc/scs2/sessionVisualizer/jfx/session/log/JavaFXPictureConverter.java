package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.nio.ByteBuffer;

import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.loader.NativeLibraryLoader;
import us.ihmc.codecs.util.ByteBufferProvider;

public class JavaFXPictureConverter
{
   static
   {
      NativeLibraryLoader.loadIHMCVideoCodecsLibrary();
   }

   private ByteBufferProvider byteBufferProvider = new ByteBufferProvider();

   /**
    * Convert YUVPicture to BufferedImage, minimizing object allocation
    * 
    * @param picture YUVPicture to convert
    * @return new BufferedImage.
    */
   public WritableImage toFXImage(YUVPicture picture)
   {
      return toFXImage(picture, null);
   }

   /**
    * Convert YUVPicture to BufferedImage, minimizing object allocation
    * 
    * @param picture     YUVPicture to convert
    * @param imageToPack Image to output to. If picture.size() != imageToPack.size() then a new
    *                    BufferedImage is allocated
    * @return imageToPack if sizes match, new BufferedImage otherwise.
    */
   public WritableImage toFXImage(YUVPicture picture, WritableImage imageToPack)
   {
      RGBPicture rgb = picture.toRGB();
      WritableImage img = toFXImage(rgb, imageToPack);
      rgb.delete();
      return img;
   }

   /**
    * Convert RGBPicture to BufferedImage, minimizing object allocation
    * 
    * @param picture     RGBPicture to convert
    * @param imageToPack Image to output to. If picture.size() != imageToPack.size() then a new
    *                    BufferedImage is allocated
    * @return imageToPack if sizes match, new BufferedImage otherwise.
    */
   public WritableImage toFXImage(RGBPicture picture, WritableImage imageToPack)
   {
      WritableImage target = imageToPack;
      int w = picture.getWidth();
      int h = picture.getHeight();
      if (target == null || target.getWidth() != w || target.getHeight() != h)
      {
         target = new WritableImage(w, h);
      }
      PixelWriter pixelWriter = target.getPixelWriter();

      ByteBuffer dstBuffer = byteBufferProvider.getOrCreateBuffer(w * h * 3);
      picture.get(dstBuffer);

      int x = 0;
      int y = 0;

      while (dstBuffer.position() < dstBuffer.limit())
      {
         int b = dstBuffer.get() & 0xff;
         int g = dstBuffer.get() & 0xff;
         int r = dstBuffer.get() & 0xff;
         int argb = (0xff << 24) | (r << 16) | (g << 8) | b;
         pixelWriter.setArgb(x, y, argb);
         x++;
         if (x >= w)
         {
            x = 0;
            y++;
         }
      }
      return target;
   }
}

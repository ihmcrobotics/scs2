package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.scene.image.PixelFormat;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import org.bytedeco.javacv.Frame;

import java.nio.ByteBuffer;

/**
 * Converts a 3-channel (BGR) {@link Frame}, as produced by javacv's FFmpeg-backed demuxers, into a
 * JavaFX {@link WritableImage}, shared by every video source that decodes through javacv (Magewell,
 * BlackMagic).
 * <p>
 * This reads directly from {@code frameToConvert.image[0]} instead of going through
 * {@code org.bytedeco.javacv.JavaFXFrameConverter}: that converter does its own hidden
 * {@code new WritableImage(...)} allocation and populates it one pixel (four separate byte
 * {@code put()} calls) at a time. Reading the frame's raw bytes directly, in bulk, into a reusable
 * heap buffer instead - profiling showed {@code DirectByteBuffer#get(int)} calls one at a time were
 * themselves a hot spot - avoids that allocation and that per-pixel/per-byte copy entirely.
 */
final class FrameImageConverter
{
   /**
    * Per-thread scratch buffers, reused across frames instead of reallocated every call. Thread-local
    * because each video reader's {@code readVideoFrame(long)} runs on whichever
    * {@code BackgroundExecutorManager} pool thread is servicing it, and a lock would defeat the
    * purpose.
    */
   private static final ThreadLocal<byte[]> RAW_ROW_BYTES = ThreadLocal.withInitial(() -> new byte[0]);
   private static final ThreadLocal<int[]> ARGB_PIXELS = ThreadLocal.withInitial(() -> new int[0]);

   private FrameImageConverter()
   {
   }

   static boolean hasImageData(Frame frame)
   {
      return frame.image != null && frame.imageWidth > 0 && frame.imageHeight > 0;
   }

   /**
    * @param frameToConvert is the next frame we want to visualize so we convert it to be compatible with JavaFX
    * @param imageToPack    image to write into if its size already matches; a new one is allocated otherwise (or if
    *                       {@code null})
    * @return {@code imageToPack} if its size matched, a new {@link WritableImage} otherwise
    */
   static WritableImage convertFrameToWritableImage(Frame frameToConvert, WritableImage imageToPack)
   {
      if (frameToConvert == null || !hasImageData(frameToConvert))
      {
         return null;
      }

      if (frameToConvert.imageChannels != 3)
         throw new UnsupportedOperationException("Only 3-channel (BGR) frames are supported, got " + frameToConvert.imageChannels + " channels");

      int width = frameToConvert.imageWidth;
      int height = frameToConvert.imageHeight;
      int stride = frameToConvert.imageStride;
      ByteBuffer sourceBuffer = (ByteBuffer) frameToConvert.image[0];

      WritableImage writableImage = imageToPack;
      if (writableImage == null || (int) writableImage.getWidth() != width || (int) writableImage.getHeight() != height)
         writableImage = new WritableImage(width, height);

      // One bulk native-memory read instead of 3 DirectByteBuffer#get(int) calls per pixel (measured
      // to be a hot spot): reading into a plain heap byte[] first means the conversion loop below only
      // ever touches cheap, bounds-checkable heap array accesses.
      int rawByteCount = stride * (height - 1) + width * 3;
      byte[] rawBytes = RAW_ROW_BYTES.get();
      if (rawBytes.length < rawByteCount)
      {
         rawBytes = new byte[rawByteCount];
         RAW_ROW_BYTES.set(rawBytes);
      }
      sourceBuffer.get(0, rawBytes, 0, rawByteCount);

      int pixelCount = width * height;
      int[] pixels = ARGB_PIXELS.get();
      if (pixels.length < pixelCount)
      {
         pixels = new int[pixelCount];
         ARGB_PIXELS.set(pixels);
      }

      for (int y = 0; y < height; y++)
      {
         int rowStart = stride * y;
         int rowOffset = y * width;

         for (int x = 0; x < width; x++)
         {
            int base = rowStart + 3 * x;
            int blue = rawBytes[base] & 0xFF;
            int green = rawBytes[base + 1] & 0xFF;
            int red = rawBytes[base + 2] & 0xFF;
            pixels[rowOffset + x] = 0xFF000000 | (red << 16) | (green << 8) | blue;
         }
      }

      PixelWriter pixelWriter = writableImage.getPixelWriter();
      pixelWriter.setPixels(0, 0, width, height, PixelFormat.getIntArgbInstance(), pixels, 0, width);

      return writableImage;
   }
}

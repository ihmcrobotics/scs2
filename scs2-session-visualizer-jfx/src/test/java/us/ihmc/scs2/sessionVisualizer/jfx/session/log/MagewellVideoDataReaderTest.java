package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import javafx.scene.image.PixelReader;
import javafx.scene.image.WritableImage;
import org.bytedeco.javacv.Frame;
import org.junit.jupiter.api.Test;
import us.ihmc.concurrent.ConcurrentCopier;

import java.nio.Buffer;
import java.nio.ByteBuffer;

public class MagewellVideoDataReaderTest
{
   private static final int WIDTH = 1280;
   private static final int HEIGHT = 720;
   private static final int WARMUP_ITERATIONS = 20;
   private static final int TIMED_ITERATIONS = 100;
   /** Generous ceiling so the test only fails on an actual performance regression, not machine noise. */
   private static final double MAX_AVERAGE_MILLIS_PER_FRAME = 40.0;

   @Test
   void convertFrameToWritableImageIsFastAndCorrect()
   {
      Frame frame = buildTestFrame(WIDTH, HEIGHT);

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         MagewellVideoDataReader.convertFrameToWritableImage(frame);
      }

      WritableImage result = null;
      long startTime = System.nanoTime();

      for (int i = 0; i < TIMED_ITERATIONS; i++)
      {
         result = MagewellVideoDataReader.convertFrameToWritableImage(frame);
      }

      long elapsedNanos = System.nanoTime() - startTime;
      double averageMillisPerFrame = elapsedNanos / 1_000_000.0 / TIMED_ITERATIONS;

      System.out.printf("convertFrameToWritableImage: %.3f ms/frame average over %d iterations (%dx%d)%n",
                        averageMillisPerFrame,
                        TIMED_ITERATIONS,
                        WIDTH,
                        HEIGHT);

      assertNotNull(result);
      assertEquals(WIDTH, (int) result.getWidth());
      assertEquals(HEIGHT, (int) result.getHeight());

      PixelReader resultReader = result.getPixelReader();
      assertEquals(expectedArgb(0, 0), resultReader.getArgb(0, 0));
      assertEquals(expectedArgb(WIDTH - 1, 0), resultReader.getArgb(WIDTH - 1, 0));
      assertEquals(expectedArgb(0, HEIGHT - 1), resultReader.getArgb(0, HEIGHT - 1));
      assertEquals(expectedArgb(WIDTH - 1, HEIGHT - 1), resultReader.getArgb(WIDTH - 1, HEIGHT - 1));
      assertEquals(expectedArgb(WIDTH / 2, HEIGHT / 2), resultReader.getArgb(WIDTH / 2, HEIGHT / 2));

      assertTrue(averageMillisPerFrame < MAX_AVERAGE_MILLIS_PER_FRAME,
                 "convertFrameToWritableImage averaged " + averageMillisPerFrame + " ms/frame, expected < " + MAX_AVERAGE_MILLIS_PER_FRAME);
   }

   /**
    * Regression test for the reused-image overload: {@link BlackMagicVideoDataReader} (via its
    * {@link ConcurrentCopier}-backed frame buffer) relies on a correctly-sized image being reused in
    * place rather than reallocated every frame - that's the whole point of passing one in.
    */
   @Test
   void convertFrameToWritableImageReusesACorrectlySizedImage()
   {
      Frame frame = buildTestFrame(WIDTH, HEIGHT);
      WritableImage imageToReuse = new WritableImage(WIDTH, HEIGHT);

      WritableImage result = FrameImageConverter.convertFrameToWritableImage(frame, imageToReuse);

      assertSame(imageToReuse, result, "A correctly-sized image should be reused in place instead of reallocated");

      PixelReader resultReader = result.getPixelReader();
      assertEquals(expectedArgb(0, 0), resultReader.getArgb(0, 0));
      assertEquals(expectedArgb(WIDTH - 1, HEIGHT - 1), resultReader.getArgb(WIDTH - 1, HEIGHT - 1));
   }

   /**
    * A mismatched-size (or {@code null}) image must never be reused - most commonly hit when the
    * video's resolution changes mid-stream, or on the very first frame.
    */
   @Test
   void convertFrameToWritableImageAllocatesWhenTheSizeDoesNotMatch()
   {
      Frame frame = buildTestFrame(WIDTH, HEIGHT);
      WritableImage wrongSize = new WritableImage(WIDTH / 2, HEIGHT / 2);

      WritableImage result = FrameImageConverter.convertFrameToWritableImage(frame, wrongSize);

      assertNotSame(wrongSize, result, "A mismatched-size image must not be reused");
      assertEquals(WIDTH, (int) result.getWidth());
      assertEquals(HEIGHT, (int) result.getHeight());
   }

   /**
    * Quantifies the win from reusing the {@link WritableImage} instead of allocating a fresh one
    * every frame (as {@link #convertFrameToWritableImageIsFastAndCorrect()} above measures).
    */
   @Test
   void convertFrameToWritableImageReuseIsFasterThanReallocating()
   {
      Frame frame = buildTestFrame(WIDTH, HEIGHT);
      WritableImage reused = new WritableImage(WIDTH, HEIGHT);

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         FrameImageConverter.convertFrameToWritableImage(frame, reused);
      }

      long startTime = System.nanoTime();

      for (int i = 0; i < TIMED_ITERATIONS; i++)
      {
         FrameImageConverter.convertFrameToWritableImage(frame, reused);
      }

      long elapsedNanos = System.nanoTime() - startTime;
      double averageMillisPerFrame = elapsedNanos / 1_000_000.0 / TIMED_ITERATIONS;

      System.out.printf("convertFrameToWritableImage (reused image): %.3f ms/frame average over %d iterations (%dx%d)%n",
                        averageMillisPerFrame,
                        TIMED_ITERATIONS,
                        WIDTH,
                        HEIGHT);
   }

   /**
    * Builds a Frame backed by a plain heap {@link ByteBuffer}, matching what this repo's vendored
    * {@code JavaFXFrameConverter} (org.bytedeco.javacv.JavaFXFrameConverter) expects: 3-channel,
    * tightly packed BGR rows, read straight out of {@code frame.image[0]}. No native JavaCPP
    * allocation is involved, so this doesn't require the video codec's native libraries to be loaded.
    */
   private static Frame buildTestFrame(int width, int height)
   {
      byte[] bgr = new byte[width * height * 3];

      for (int y = 0; y < height; y++)
      {
         for (int x = 0; x < width; x++)
         {
            int index = (y * width + x) * 3;
            bgr[index] = blue(x, y);
            bgr[index + 1] = green(x, y);
            bgr[index + 2] = red(x, y);
         }
      }

      Frame frame = new Frame();
      frame.imageWidth = width;
      frame.imageHeight = height;
      frame.imageChannels = 3;
      frame.imageDepth = Frame.DEPTH_UBYTE;
      frame.imageStride = width * 3;
      frame.image = new Buffer[] {ByteBuffer.wrap(bgr)};

      return frame;
   }

   private static byte red(int x, int y)
   {
      return (byte) (x & 0xFF);
   }

   private static byte green(int x, int y)
   {
      return (byte) (y & 0xFF);
   }

   private static byte blue(int x, int y)
   {
      return (byte) ((x + y) & 0xFF);
   }

   private static int expectedArgb(int x, int y)
   {
      return 0xFF000000 | ((red(x, y) & 0xFF) << 16) | ((green(x, y) & 0xFF) << 8) | (blue(x, y) & 0xFF);
   }
}

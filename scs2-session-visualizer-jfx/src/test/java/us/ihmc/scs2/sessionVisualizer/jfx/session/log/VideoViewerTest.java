package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.testfx.api.FxToolkit;

import java.util.concurrent.Callable;

/**
 * Regression test for the thumbnail downscaling in {@link VideoViewer}: the thumbnail used to be
 * handed the full-resolution decoded frame directly, which - regardless of how small the
 * {@code ImageView} draws it - forces JavaFX to upload the full-resolution image as a GPU texture
 * every pulse, for every camera. {@link VideoViewer#createThumbnailImage(WritableImage, int)}
 * downscales on the CPU first so only a small image ever reaches the GPU.
 */
public class VideoViewerTest
{
   private static final int SOURCE_WIDTH = 1280;
   private static final int SOURCE_HEIGHT = 720;
   /** Matches LogSessionManagerController.THUMBNAIL_WIDTH, so the benchmark reflects real usage. */
   private static final int THUMBNAIL_WIDTH = 200;
   private static final int WARMUP_ITERATIONS = 20;
   private static final int TIMED_ITERATIONS = 100;
   /** Generous ceiling so the test only fails on an actual performance regression, not machine noise. */
   private static final double MAX_AVERAGE_MILLIS_PER_FRAME = 5.0;

   @Tag("javafx-headless")
   @Test
   public void createThumbnailImageIsFast() throws Exception
   {
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();

      Callable<Void> test = () ->
      {
         WritableImage source = new WritableImage(SOURCE_WIDTH, SOURCE_HEIGHT);
         PixelWriter writer = source.getPixelWriter();
         for (int y = 0; y < SOURCE_HEIGHT; y++)
            for (int x = 0; x < SOURCE_WIDTH; x++)
               writer.setArgb(x, y, 0xFF000000 | (x << 16) | (y << 8) | ((x + y) & 0xFF));

         for (int i = 0; i < WARMUP_ITERATIONS; i++)
            VideoViewer.createThumbnailImage(source, THUMBNAIL_WIDTH);

         long startTime = System.nanoTime();

         for (int i = 0; i < TIMED_ITERATIONS; i++)
            VideoViewer.createThumbnailImage(source, THUMBNAIL_WIDTH);

         long elapsedNanos = System.nanoTime() - startTime;
         double averageMillisPerFrame = elapsedNanos / 1_000_000.0 / TIMED_ITERATIONS;

         System.out.printf("createThumbnailImage: %.3f ms/frame average over %d iterations (%dx%d -> %dpx wide)%n",
                           averageMillisPerFrame,
                           TIMED_ITERATIONS,
                           SOURCE_WIDTH,
                           SOURCE_HEIGHT,
                           THUMBNAIL_WIDTH);

         assertTrue(averageMillisPerFrame < MAX_AVERAGE_MILLIS_PER_FRAME,
                    "createThumbnailImage averaged " + averageMillisPerFrame + " ms/frame, expected < " + MAX_AVERAGE_MILLIS_PER_FRAME);
         return null;
      };
      FxToolkit.setupFixture(test);
   }

   @Tag("javafx-headless")
   @Test
   public void testCreateThumbnailImagePreservesAspectRatioAndSamplesCorrectly() throws Exception
   {
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();

      Callable<Void> test = () ->
      {
         // 4 quadrants of distinct solid colors, easy to reason about after downscaling.
         int sourceWidth = 1280;
         int sourceHeight = 720;
         WritableImage source = new WritableImage(sourceWidth, sourceHeight);
         PixelWriter writer = source.getPixelWriter();
         for (int y = 0; y < sourceHeight; y++)
         {
            for (int x = 0; x < sourceWidth; x++)
            {
               boolean left = x < sourceWidth / 2;
               boolean top = y < sourceHeight / 2;
               Color color = top ? (left ? Color.RED : Color.GREEN) : (left ? Color.BLUE : Color.YELLOW);
               writer.setColor(x, y, color);
            }
         }

         int targetWidth = 160;
         WritableImage thumbnail = VideoViewer.createThumbnailImage(source, targetWidth);

         assertNotSame(source, thumbnail, "Downscaling should never return the source image itself");
         assertEquals(targetWidth, (int) thumbnail.getWidth());
         assertEquals(Math.round((float) targetWidth * sourceHeight / sourceWidth), (int) thumbnail.getHeight(), "Aspect ratio should be preserved");

         int tw = (int) thumbnail.getWidth();
         int th = (int) thumbnail.getHeight();
         assertEquals(Color.RED, thumbnail.getPixelReader().getColor(tw / 4, th / 4), "Top-left quadrant");
         assertEquals(Color.GREEN, thumbnail.getPixelReader().getColor(3 * tw / 4, th / 4), "Top-right quadrant");
         assertEquals(Color.BLUE, thumbnail.getPixelReader().getColor(tw / 4, 3 * th / 4), "Bottom-left quadrant");
         assertEquals(Color.YELLOW, thumbnail.getPixelReader().getColor(3 * tw / 4, 3 * th / 4), "Bottom-right quadrant");
         return null;
      };
      FxToolkit.setupFixture(test);
   }

   @Tag("javafx-headless")
   @Test
   public void testCreateThumbnailImageAlwaysReturnsANewInstance() throws Exception
   {
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();

      Callable<Void> test = () ->
      {
         WritableImage source = new WritableImage(320, 240);

         WritableImage first = VideoViewer.createThumbnailImage(source, 80);
         WritableImage second = VideoViewer.createThumbnailImage(source, 80);

         // Must never reuse the same instance across calls: ImageView only re-renders on an actual
         // reference change (see VideoViewer.createThumbnailImage's javadoc for why), so handing back
         // the same object twice would make the displayed thumbnail appear frozen after the first frame.
         assertNotSame(first, second, "Each call should allocate a fresh thumbnail image, never reuse one in place");
         return null;
      };
      FxToolkit.setupFixture(test);
   }
}

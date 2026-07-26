package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.scene.image.Image;
import javafx.scene.image.PixelReader;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import logger_msgs.Camera;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.JavaFXFrameConverter;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.scs2.session.log.BlackMagicScrubber;
import us.ihmc.scs2.session.log.ProgressConsumer;

import java.io.File;
import java.io.IOException;

public class BlackMagicVideoDataReader implements VideoDataReader
{
   /** Cap on consecutive non-video packets to skip per seek (audio/timecode interleaved with video). */
   private static final int MAX_NON_VIDEO_FRAMES_TO_SKIP = 256;

   private final BlackMagicScrubber blackMagicScrubber;
   private final ConcurrentCopier<FrameData> imageBuffer = new ConcurrentCopier<>(FrameData::new);

   public BlackMagicVideoDataReader(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      blackMagicScrubber = new BlackMagicScrubber(camera, dataDirectory, hasTimeBase);
   }

   @Override
   public int getImageHeight()
   {
      return 0;
   }

   @Override
   public int getImageWidth()
   {
      return 0;
   }

   public void readVideoFrame(long queryRobotTimestamp)
   {
      try
      {
         Frame nextFrame = blackMagicScrubber.readVideoFrame(queryRobotTimestamp);

         // The underlying FFmpegFrameGrabber.grabFrame() returns the next packet from any stream,
         // so a multi-stream MP4 (video + audio + timecode) may yield non-image frames here.
         int skipped = 0;
         while (nextFrame != null && !hasImageData(nextFrame) && skipped < MAX_NON_VIDEO_FRAMES_TO_SKIP)
         {
            nextFrame = blackMagicScrubber.getDemuxer().getNextFrame();
            skipped++;
         }

         FrameData copyForWriting = imageBuffer.getCopyForWriting();
         copyForWriting.queryRobotTimestamp = queryRobotTimestamp;
         copyForWriting.currentRobotTimestamp = blackMagicScrubber.getCurrentRobotTimestamp();
         copyForWriting.currentVideoTimestamp = blackMagicScrubber.getVideoTimestamp();
         copyForWriting.currentDemuxerTimestamp = blackMagicScrubber.getDemuxer().getCurrentPTS();
         copyForWriting.frame = convertFrameToWritableImage(nextFrame);

         imageBuffer.commit();
      }
      catch (RuntimeException e)
      {
         e.printStackTrace();
      }
   }

   private static boolean hasImageData(Frame frame)
   {
      return frame.image != null && frame.imageWidth > 0 && frame.imageHeight > 0;
   }

   private static WritableImage convertFrameToWritableImage(Frame frameToConvert)
   {
      if (frameToConvert == null || !hasImageData(frameToConvert))
      {
         return null;
      }

      Image currentImage;

      try (JavaFXFrameConverter frameConverter = new JavaFXFrameConverter())
      {
         currentImage = frameConverter.convert(frameToConvert);
      }

      WritableImage writableImage = new WritableImage((int) currentImage.getWidth(), (int) currentImage.getHeight());
      PixelReader pixelReader = currentImage.getPixelReader();
      PixelWriter pixelWriter = writableImage.getPixelWriter();

      for (int y = 0; y < currentImage.getHeight(); y++)
      {
         for (int x = 0; x < currentImage.getWidth(); x++)
         {
            pixelWriter.setArgb(x, y, pixelReader.getArgb(x, y));
         }
      }

      return writableImage;
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor) throws IOException
   {
      blackMagicScrubber.cropVideo(outputFile, timestampFile, startTimestamp, endTimestamp, monitor);
   }

   public String getName()
   {
      return blackMagicScrubber.getName();
   }

   public Camera getCamera()
   {
      return blackMagicScrubber.getCamera();
   }

   public FrameData pollCurrentFrame()
   {
      return imageBuffer.getCopyForReading();
   }

   public int getCurrentIndex()
   {
      return blackMagicScrubber.getCurrentIndex();
   }

   public boolean replacedRobotTimestampsContainsIndex(int index)
   {
      return blackMagicScrubber.replacedRobotTimestampsContainsIndex(index);
   }
}

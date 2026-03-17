package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import logger_msgs.msg.dds.Camera;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.scs2.session.log.ProgressConsumer;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;

import java.io.File;
import java.io.IOException;

import static us.ihmc.zed.global.zed.*;

/**
 * Reads video data from one or more ZED SDK SVO2 files recorded during the log.
 */
public class ZEDSVOVideoDataReader implements VideoDataReader
{
   private final ZEDSVOScrubber zedScrubber;
   private final Camera camera = new Camera();

   private final FrameData frameData = new FrameData();

   public ZEDSVOVideoDataReader(File timestampsDatFile)
   {
      zedScrubber = new ZEDSVOScrubber(timestampsDatFile);

      camera.setVideoFile("%s.svo2".formatted(zedScrubber.getName())); // We just have to set this to not crash
      camera.setTimestampFile(timestampsDatFile.toPath().getFileName().toString());
   }

   @Override
   public void readVideoFrame(long timestamp)
   {
      zedScrubber.scrub(timestamp);
      int imageHeight = zedScrubber.getImageHeight();
      int imageWidth = zedScrubber.getImageWidth();

      Pointer leftColorImageSlMatPointer = zedScrubber.getLeftColorImageSlMatPointer();

      Mat imageMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                             sl_mat_get_ptr(leftColorImageSlMatPointer, SL_MEM_CPU), sl_mat_get_step_bytes(leftColorImageSlMatPointer, SL_MEM_CPU));

      WritableImage writableImage = new WritableImage(imageWidth, imageHeight);

      PixelWriter pixelWriter = writableImage.getPixelWriter();

      byte[] buffer = new byte[imageWidth * imageHeight * 4]; // BGRA buffer
      imageMat.ptr(0, 0).get(buffer);

      for (int y = 0; y < imageHeight; y++)
      {
         for (int x = 0; x < imageWidth; x++)
         {
            int bufferIndex = (y * imageWidth + x) * 4;
            int b = buffer[bufferIndex] & 0xFF;
            int g = buffer[bufferIndex + 1] & 0xFF;
            int r = buffer[bufferIndex + 2] & 0xFF;
            int a = buffer[bufferIndex + 3] & 0xFF;
            int argb = (a << 24) | (r << 16) | (g << 8) | b;
            pixelWriter.setArgb(x, y, argb);
         }
      }

      frameData.frame = writableImage;

      // may not be necessary, it's for debugging supposedly
      frameData.queryRobotTimestamp = timestamp;
      frameData.currentRobotTimestamp = zedScrubber.getTimestampScrubber().getCurrentRobotTimestamp();
      frameData.currentVideoTimestamp = zedScrubber.getTimestampScrubber().getCurrentVideoTimestamp();
      frameData.currentDemuxerTimestamp = zedScrubber.getCurrentTimestamp();
   }

   @Override
   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor) throws IOException
   {
      zedScrubber.cropVideo(outputFile, timestampFile, startTimestamp, endTimestamp, monitor);
   }

   @Override
   public Camera getCamera()
   {
      return camera; // Only used for cropping
   }

   public void close()
   {
      zedScrubber.close();
   }

   @Override
   public String getName()
   {
      return zedScrubber.getName();
   }

   @Override
   public FrameData pollCurrentFrame()
   {
      return frameData;
   }

   @Override
   public int getCurrentIndex()
   {
      return zedScrubber.getTimestampScrubber().getCurrentIndex();
   }

   @Override
   public boolean replacedRobotTimestampsContainsIndex(int index)
   {
      return zedScrubber.getTimestampScrubber().getReplacedRobotTimestampIndex(index);
   }

   @Override
   public int getImageHeight()
   {
      return zedScrubber.getImageHeight(); // Unused
   }

   @Override
   public int getImageWidth()
   {
      return zedScrubber.getImageWidth(); // Unused
   }
}

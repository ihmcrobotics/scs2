package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.scs2.session.log.ProgressConsumer;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

public class BlackMagicVideoDataReader implements VideoDataReader
{
   private final TimestampScrubber timestampScrubber;
   private final String name;

   private final MP4VideoDemuxer demuxer;
   private final JavaFXPictureConverter converter = new JavaFXPictureConverter();

   private final File videoFile;
   private final Camera camera;
   private final ConcurrentCopier<FrameData> imageBuffer = new ConcurrentCopier<>(FrameData::new);

   public BlackMagicVideoDataReader(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      this.camera = camera;
      name = camera.getNameAsString();
      boolean interlaced = camera.getInterlaced();

      if (!hasTimeBase)
      {
         System.err.println("Video data is using timestamps instead of frame numbers. Falling back to seeking based on timestamp.");
      }

      videoFile = new File(dataDirectory, camera.getVideoFileAsString());

      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      demuxer = new MP4VideoDemuxer(videoFile);

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());
      this.timestampScrubber = new TimestampScrubber(timestampFile, hasTimeBase, interlaced);
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
      long videoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(queryRobotTimestamp);
      long currentRobotTimestamp = timestampScrubber.getCurrentRobotTimestamp();

      try
      {
         demuxer.seekToPTS(videoTimestamp);
         FrameData copyForWriting = imageBuffer.getCopyForWriting();
         copyForWriting.queryRobotTimestamp = queryRobotTimestamp;
         copyForWriting.currentRobotTimestamp = currentRobotTimestamp;
         copyForWriting.currentVideoTimestamp = videoTimestamp;
         copyForWriting.currentDemuxerTimestamp = demuxer.getCurrentPTS();
         YUVPicture nextFrame = demuxer.getNextFrame(); // Increment frame index after getting frame.
         copyForWriting.frame = converter.toFXImage(nextFrame, copyForWriting.frame);

         imageBuffer.commit();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor) throws IOException
   {
      long startVideoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(endTimestamp);

      int framerate = VideoConverter.cropBlackMagicVideo(videoFile, outputFile, startVideoTimestamp, endVideoTimestamp, monitor);

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(1);
      timestampWriter.println(framerate);

      long pts = 0;
      /*
       * PTS gets reordered to be monotonically increasing starting from 0
       */
      for (int i = 0; i < timestampScrubber.getRobotTimestampsLength(); i++)
      {
         long robotTimestamp = timestampScrubber.getRobotTimestampAtIndex(i);

         if (robotTimestamp >= startTimestamp && robotTimestamp <= endTimestamp)
         {
            timestampWriter.print(robotTimestamp);
            timestampWriter.print(" ");
            timestampWriter.println(pts);
            pts++;
         }
         else if (robotTimestamp > endTimestamp)
         {
            break;
         }
      }

      timestampWriter.close();
   }

   public String getName()
   {
      return name;
   }

   public Camera getCamera()
   {
      return camera;
   }

   public FrameData pollCurrentFrame()
   {
      return imageBuffer.getCopyForReading();
   }

   public int getCurrentIndex()
   {
      return timestampScrubber.getCurrentIndex();
   }

   public boolean replacedRobotTimestampsContainsIndex(int index)
   {
      return timestampScrubber.getReplacedRobotTimestampIndex(index);
   }
}

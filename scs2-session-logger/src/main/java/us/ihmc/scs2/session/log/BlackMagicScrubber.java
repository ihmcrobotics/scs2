package us.ihmc.scs2.session.log;

import logger_msgs.msg.dds.Camera;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Provides support for scrubbing images from .mov files recorded from the Decklink cards.
 */
public class BlackMagicScrubber
{
   private final TimestampScrubber timestampScrubber;
   private final String name;

   private final MP4VideoDemuxer demuxer;

   private final File videoFile;
   private final Camera camera;
   private long videoTimestamp;
   private long currentRobotTimestamp;

   public BlackMagicScrubber(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
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

   public YUVPicture readVideoFrame(long queryRobotTimestamp) throws IOException
   {
      videoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(queryRobotTimestamp);
      currentRobotTimestamp = timestampScrubber.getCurrentRobotTimestamp();

      demuxer.seekToPTS(videoTimestamp);

      return demuxer.getNextFrame(); // Increment frame index after getting frame.
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

   public MP4VideoDemuxer getDemuxer()
   {
      return demuxer;
   }

   public long getVideoTimestamp()
   {
      return videoTimestamp;
   }

   public long getCurrentRobotTimestamp()
   {
      return currentRobotTimestamp;
   }

   public String getName()
   {
      return name;
   }

   public Camera getCamera()
   {
      return camera;
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

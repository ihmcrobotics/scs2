package us.ihmc.scs2.session.log;

import logger_msgs.Camera;
import org.bytedeco.javacv.Frame;
import us.ihmc.robotDataLogger.logger.MagewellDemuxer;
import us.ihmc.robotDataLogger.logger.MagewellMuxer;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Provides support for scrubbing images from .mov files recorded with the Magewell logger.
 */
public class MagewellScrubber
{
   private final TimestampScrubber timestampScrubber;
   private final String name;

   private final MagewellDemuxer magewellDemuxer;

   private final Camera camera;
   private long currentVideoTimestamp;
   private long currentRobotTimestamp;

   public MagewellScrubber(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      this.camera = camera;
      name = camera.getNameAsString();
      boolean interlaced = camera.getInterlaced();

      if (!hasTimeBase)
      {
         System.err.println("Video data is using timestamps instead of frame numbers. Falling back to seeking based on timestamp.");
      }

      File videoFile = new File(dataDirectory, camera.getVideoFileAsString());

      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      magewellDemuxer = new MagewellDemuxer(videoFile);

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());
      this.timestampScrubber = new TimestampScrubber(timestampFile, hasTimeBase, interlaced);
   }

   public int getImageHeight()
   {
      return magewellDemuxer.getImageHeight();
   }

   public int getImageWidth()
   {
      return magewellDemuxer.getImageWidth();
   }

   public Frame readVideoFrame(long queryRobotTimestamp)
   {
      currentVideoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(queryRobotTimestamp);
      currentRobotTimestamp = timestampScrubber.getCurrentRobotTimestamp();

      magewellDemuxer.seekToPTS(currentVideoTimestamp);

      return magewellDemuxer.getNextFrame();
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer progressConsumer) throws IOException
   {
      long startVideoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(endTimestamp);

      long[] robotTimestampsForCroppedLog = timestampScrubber.getCroppedRobotTimestamps(startTimestamp, endTimestamp);
      long[] videoTimestampsForCroppedLog = new long[robotTimestampsForCroppedLog.length];
      int i = 0;

      // This stuff is used to print to SCS2 so the user knows how the cropped log is going, progress wise
      long startFrame = getFrameAtTimestamp(startVideoTimestamp, magewellDemuxer); // This also moves the stream to the startFrame
      long endFrame = getFrameAtTimestamp(endVideoTimestamp, magewellDemuxer);
      long numberOfFrames = endFrame - startFrame;
      int frameRate = (int) magewellDemuxer.getFrameRate();

      magewellDemuxer.seekToPTS(startVideoTimestamp);

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(1 + "\n" + frameRate);

      MagewellMuxer magewellMuxer = new MagewellMuxer(outputFile, magewellDemuxer.getImageWidth(), magewellDemuxer.getImageHeight());
      magewellMuxer.start();

      Frame frame;
      while (i < videoTimestampsForCroppedLog.length && (frame = magewellDemuxer.getNextFrame()) != null && magewellDemuxer.getFrameNumber() <= endFrame)
      {
         // Skip non-video packets (audio, timecode) that grabFrame() returns from multi-stream MP4s.
         if (frame.image == null || frame.imageWidth <= 0 || frame.imageHeight <= 0)
            continue;

         // Use the frame's original PTS (relative to the crop start) so playback speed matches the source
         // recording, regardless of how fast this machine happens to decode/encode during cropping.
         long videoTimestamp = magewellDemuxer.getCurrentPTS() - startVideoTimestamp;
         magewellMuxer.recordFrame(frame, videoTimestamp);
         videoTimestampsForCroppedLog[i] = magewellMuxer.getTimeStamp();
         i++;

         if (progressConsumer != null)
         {
            progressConsumer.info("frame %d/%d".formatted(magewellDemuxer.getFrameNumber() - startFrame, numberOfFrames));
            progressConsumer.progress((double) (magewellDemuxer.getFrameNumber() - startFrame) / (double) numberOfFrames);
         }
      }

      // i may be less than videoTimestampsForCroppedLog.length if the demuxer ran out of frames before reaching
      // endFrame (e.g. seeking landed short on an old, keyframe-less recording); only pair up what was actually written.
      int framesWritten = i;
      for (i = 0; i < framesWritten; i++)
      {
         timestampWriter.print(robotTimestampsForCroppedLog[i]);
         timestampWriter.print(" ");
         timestampWriter.println(videoTimestampsForCroppedLog[i]);
      }

      magewellMuxer.close();
      timestampWriter.close();
   }

   private static long getFrameAtTimestamp(long endCameraTimestamp, MagewellDemuxer magewellDemuxer)
   {
      magewellDemuxer.seekToPTS(endCameraTimestamp);
      return magewellDemuxer.getFrameNumber();
   }

   public long getCurrentRobotTimestamp()
   {
      return currentRobotTimestamp;
   }

   public long getCurrentVideoTimestamp()
   {
      return currentVideoTimestamp;
   }

   public TimestampScrubber getTimestampScrubber()
   {
      return timestampScrubber;
   }

   public MagewellDemuxer getMagewellDemuxer()
   {
      return magewellDemuxer;
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

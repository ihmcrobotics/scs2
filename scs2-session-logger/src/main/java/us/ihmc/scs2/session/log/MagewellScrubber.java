package us.ihmc.scs2.session.log;

import logger_msgs.Camera;
import org.bytedeco.javacv.Frame;
import us.ihmc.robotDataLogger.logger.FFmpegDemuxer;
import us.ihmc.robotDataLogger.logger.FFmpegMuxer;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Provides support for scrubbing images from .mov files recorded with the Magewell logger.
 */
public class MagewellScrubber
{
   public static final int DEFAULT_FRAME_DELAY = 4;

   private final TimestampScrubber timestampScrubber;
   private final String name;

   private final FFmpegDemuxer ffmpegDemuxer;
   private final long nanosPerFrame;

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

      ffmpegDemuxer = new FFmpegDemuxer(videoFile);
      // Duration of one video frame in nanoseconds, matching the units of the robot timestamps, so a frame count
      // (e.g. the user-facing frame delay) can be converted to a delay in TimestampScrubber's search key.
      nanosPerFrame = Math.round(1.0e9 / ffmpegDemuxer.getFrameRate());

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());
      this.timestampScrubber = new TimestampScrubber(timestampFile, hasTimeBase, interlaced);
      setFrameDelay(DEFAULT_FRAME_DELAY);
   }

   public void setFrameDelay(int frames)
   {
      timestampScrubber.setDelay(frames * nanosPerFrame);
   }

   public int getFrameDelay()
   {
      return nanosPerFrame == 0 ? 0 : Math.round((float) timestampScrubber.getDelay() / nanosPerFrame);
   }

   public int getImageHeight()
   {
      return ffmpegDemuxer.getImageHeight();
   }

   public int getImageWidth()
   {
      return ffmpegDemuxer.getImageWidth();
   }

   public Frame readVideoFrame(long queryRobotTimestamp)
   {
      currentVideoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(queryRobotTimestamp);
      currentRobotTimestamp = timestampScrubber.getCurrentRobotTimestamp();

      ffmpegDemuxer.seekToPTS(currentVideoTimestamp);

      return ffmpegDemuxer.getNextFrame();
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer progressConsumer) throws IOException
   {
      long startVideoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getVideoTimestampFromRobotTimestamp(endTimestamp);

      long[] robotTimestampsForCroppedLog = timestampScrubber.getCroppedRobotTimestamps(startTimestamp, endTimestamp);
      long[] videoTimestampsForCroppedLog = new long[robotTimestampsForCroppedLog.length];
      int i = 0;

      // This stuff is used to print to SCS2 so the user knows how the cropped log is going, progress wise
      long startFrame = getFrameAtTimestamp(startVideoTimestamp, ffmpegDemuxer); // This also moves the stream to the startFrame
      long endFrame = getFrameAtTimestamp(endVideoTimestamp, ffmpegDemuxer);
      long numberOfFrames = endFrame - startFrame;
      int frameRate = (int) ffmpegDemuxer.getFrameRate();

      ffmpegDemuxer.seekToPTS(startVideoTimestamp);

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(1 + "\n" + frameRate);

      FFmpegMuxer magewellMuxer = new FFmpegMuxer(outputFile, ffmpegDemuxer.getImageWidth(), ffmpegDemuxer.getImageHeight());
      magewellMuxer.start();

      Frame frame;
      while (i < videoTimestampsForCroppedLog.length && (frame = ffmpegDemuxer.getNextFrame()) != null && ffmpegDemuxer.getFrameNumber() <= endFrame)
      {
         // Skip non-video packets (audio, timecode) that grabFrame() returns from multi-stream MP4s.
         if (frame.image == null || frame.imageWidth <= 0 || frame.imageHeight <= 0)
            continue;

         // Use the frame's original PTS (relative to the crop start) so playback speed matches the source
         // recording, regardless of how fast this machine happens to decode/encode during cropping.
         long videoTimestamp = ffmpegDemuxer.getCurrentPTS() - startVideoTimestamp;
         magewellMuxer.recordFrame(frame, videoTimestamp);
         videoTimestampsForCroppedLog[i] = magewellMuxer.getTimeStamp();
         i++;

         if (progressConsumer != null)
         {
            progressConsumer.info("frame %d/%d".formatted(ffmpegDemuxer.getFrameNumber() - startFrame, numberOfFrames));
            progressConsumer.progress((double) (ffmpegDemuxer.getFrameNumber() - startFrame) / (double) numberOfFrames);
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

      magewellMuxer.stopRecording();
      timestampWriter.close();
   }

   private static long getFrameAtTimestamp(long endCameraTimestamp, FFmpegDemuxer magewellDemuxer)
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

   public FFmpegDemuxer getFfmpegDemuxer()
   {
      return ffmpegDemuxer;
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

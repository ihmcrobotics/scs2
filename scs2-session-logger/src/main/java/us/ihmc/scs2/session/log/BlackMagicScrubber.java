package us.ihmc.scs2.session.log;

import logger_msgs.Camera;
import org.bytedeco.javacv.Frame;
import us.ihmc.robotDataLogger.logger.FFmpegDemuxer;
import us.ihmc.robotDataLogger.logger.FFmpegMuxer;

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

   private final FFmpegDemuxer demuxer;
   private final boolean hasTimeBase;

   private final Camera camera;
   private long videoTimestamp;
   private long currentRobotTimestamp;

   public BlackMagicScrubber(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      this.camera = camera;
      this.hasTimeBase = hasTimeBase;
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

      demuxer = new FFmpegDemuxer(videoFile);

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());
      this.timestampScrubber = new TimestampScrubber(timestampFile, hasTimeBase, interlaced);
   }

   public Frame readVideoFrame(long queryRobotTimestamp)
   {
      long scrubbedVideoValue = timestampScrubber.getVideoTimestampFromRobotTimestamp(queryRobotTimestamp);
      currentRobotTimestamp = timestampScrubber.getCurrentRobotTimestamp();

      videoTimestamp = toVideoPTS(scrubbedVideoValue);
      demuxer.seekToPTS(videoTimestamp);

      return demuxer.getNextFrame(); // Increment frame index after getting frame.
   }

   /**
    * When {@code hasTimeBase} is true, the Timestamps.dat file's second column is a video frame
    * number (0, 1, 2, ...), not a timestamp - it has to be converted to the microsecond PTS
    * {@link FFmpegDemuxer#seekToPTS} actually expects using the video's own frame rate, or every
    * seek lands inside frame 0's PTS window and playback never advances past the first frame.
    */
   private long toVideoPTS(long scrubbedVideoValue)
   {
      if (!hasTimeBase)
         return scrubbedVideoValue;

      return Math.round(1.0e6 * scrubbedVideoValue / demuxer.getFrameRate());
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer progressConsumer) throws IOException
   {
      long startVideoTimestamp = toVideoPTS(timestampScrubber.getVideoTimestampFromRobotTimestamp(startTimestamp));
      long endVideoTimestamp = toVideoPTS(timestampScrubber.getVideoTimestampFromRobotTimestamp(endTimestamp));

      long[] robotTimestampsForCroppedLog = timestampScrubber.getCroppedRobotTimestamps(startTimestamp, endTimestamp);
      long[] videoTimestampsForCroppedLog = new long[robotTimestampsForCroppedLog.length];
      int i = 0;

      // This stuff is used to print to SCS2 so the user knows how the cropped log is going, progress wise
      long startFrame = getFrameAtTimestamp(startVideoTimestamp, demuxer); // This also moves the stream to the startFrame
      long endFrame = getFrameAtTimestamp(endVideoTimestamp, demuxer);
      long numberOfFrames = endFrame - startFrame;
      int frameRate = (int) demuxer.getFrameRate();

      demuxer.seekToPTS(startVideoTimestamp);

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(1 + "\n" + frameRate);

      FFmpegMuxer muxer = new FFmpegMuxer(outputFile, demuxer.getImageWidth(), demuxer.getImageHeight());
      muxer.start();

      Frame frame;
      while (i < videoTimestampsForCroppedLog.length && (frame = demuxer.getNextFrame()) != null && demuxer.getFrameNumber() <= endFrame)
      {
         // Skip non-video packets (audio, timecode) that grabFrame() returns from multi-stream MP4s.
         if (frame.image == null || frame.imageWidth <= 0 || frame.imageHeight <= 0)
            continue;

         // Use the frame's original PTS (relative to the crop start) so playback speed matches the source
         // recording, regardless of how fast this machine happens to decode/encode during cropping.
         long frameVideoTimestamp = demuxer.getCurrentPTS() - startVideoTimestamp;
         muxer.recordFrame(frame, frameVideoTimestamp);
         videoTimestampsForCroppedLog[i] = muxer.getTimeStamp();
         i++;

         if (progressConsumer != null)
         {
            progressConsumer.info("frame %d/%d".formatted(demuxer.getFrameNumber() - startFrame, numberOfFrames));
            progressConsumer.progress((double) (demuxer.getFrameNumber() - startFrame) / (double) numberOfFrames);
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

      muxer.stopRecording();
      timestampWriter.close();
   }

   private static long getFrameAtTimestamp(long endCameraTimestamp, FFmpegDemuxer demuxer)
   {
      demuxer.seekToPTS(endCameraTimestamp);
      return demuxer.getFrameNumber();
   }

   public FFmpegDemuxer getDemuxer()
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

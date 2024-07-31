package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.FrameGrabber;

import java.io.File;
import java.util.concurrent.atomic.AtomicLong;

public class FFMPEGVideoDataReader
{
   private final File videoFile;
   private final FFmpegFrameGrabber frameGrabber;
   private Frame currentFrame = null;
   /**
    * Stores the max video duration in nanoseconds
    */
   private final long maxVideoTimestamp;
   /**
    * Stores the current nanosecond playback offset from start of video
    */
   private final AtomicLong playbackOffset = new AtomicLong(0);
   /**
    * Stores the current nanosecond timestamp for video playback. Note that this stores the timestamp independent of the playback offset.
    */
   private final AtomicLong currentTimestamp = new AtomicLong(0);

   public FFMPEGVideoDataReader(File file)
   {
      videoFile = file;
      frameGrabber = new FFmpegFrameGrabber(file);
      try
      {
         frameGrabber.start();
         currentFrame = frameGrabber.grabFrame();
      }
      catch (FrameGrabber.Exception e)
      {
         throw new RuntimeException(e);
      }

      maxVideoTimestamp = frameGrabber.getLengthInTime() * 1000;
   }

   public Frame getCurrentFrame()
   {
      return currentFrame;
   }

   /**
    * reads the frame in the video at timestamp + playbackOffset
    *
    * @param timestamp nanosecond timestamp to specify time from start of video
    */
   public void readFrameAtTimestamp(long timestamp)
   {
      if (currentTimestamp.get() != timestamp)
      {
         // clamp video timestamp to be between 0 and maxVideoTimestamp
         currentTimestamp.set(Math.min(maxVideoTimestamp, Math.max(0, (timestamp))));
      }

      // clamp video timestamp + offset to be within bounds
      long clampedTime = Math.min(maxVideoTimestamp, Math.max(0, currentTimestamp.get() + playbackOffset.get()));

      // NOTE: timestamp passed in is in nanoseconds
      if (clampedTime != frameGrabber.getTimestamp())
      {
         try
         {
            frameGrabber.setVideoTimestamp(convertNanosecondToVideoTimestamp(clampedTime));
            currentFrame = frameGrabber.grabFrame();
         }
         catch (FrameGrabber.Exception e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   public void readCurrentFrame()
   {
      readFrameAtTimestamp(this.currentTimestamp.get());
   }

   public long getVideoLengthInSeconds()
   {
      return maxVideoTimestamp / 1000000000L;
   }

   /**
    * Sets the video playback offset referenced from start of the video.
    *
    * @param offset Nanosecond offset for starting video playback
    */
   public void setPlaybackOffset(long offset)
   {
      this.playbackOffset.set(offset);
   }

   public long getPlaybackOffset()
   {
      return this.playbackOffset.get();
   }

   public long getCurrentTimestamp()
   {
      return currentTimestamp.get();
   }

   public void shutdown()
   {
      try
      {
         frameGrabber.stop();
         frameGrabber.release();
      }
      catch (FFmpegFrameGrabber.Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   private long convertNanosecondToVideoTimestamp(long nanosecondTimestamp)
   {
      return nanosecondTimestamp / 1000L;
   }
}

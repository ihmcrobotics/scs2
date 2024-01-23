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
   private final long maxVideoTimestamp;

   private final AtomicLong currentTimestamp = new AtomicLong(-1);

   public FFMPEGVideoDataReader(File file)
   {
      videoFile = file;
      frameGrabber = new FFmpegFrameGrabber(file);
      try
      {
         frameGrabber.start();
      }
      catch (FFmpegFrameGrabber.Exception e)
      {
         throw new RuntimeException(e);
      }

      maxVideoTimestamp = frameGrabber.getLengthInTime();
   }

   public Frame getCurrentFrame()
   {
      return currentFrame;
   }

   public long readFrameAtTimestamp(long timestamp)
   {
      // NOTE: timestamp passed in is in nanoseconds
      if (timestamp != currentTimestamp.get())
      {
         // timestamp / 1000L converts a nanosecond timestamp to the video timestamp in time_base units
         long clampedTime = Math.min(maxVideoTimestamp, Math.max(0, timestamp / 1000L));
         currentTimestamp.set(clampedTime);
         try
         {
            frameGrabber.setVideoTimestamp(clampedTime);
            currentFrame = frameGrabber.grabFrame();
         }
         catch (FrameGrabber.Exception e)
         {
            throw new RuntimeException(e);
         }
      }
      return currentTimestamp.get();
   }

   public long getVideoLengthInSeconds()
   {
      return maxVideoTimestamp / 1000000;
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
}

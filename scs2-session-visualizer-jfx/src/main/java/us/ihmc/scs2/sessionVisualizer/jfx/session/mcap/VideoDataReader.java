package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.FrameGrabber;

import java.io.File;
import java.util.concurrent.atomic.AtomicLong;

public class VideoDataReader
{
   private final File videoFile;
   private final FFmpegFrameGrabber frameGrabber;
   private Frame currentFrame = null;
   private final long maxLengthTimestamp;

   private final AtomicLong currentTimestamp = new AtomicLong(-1);

   public VideoDataReader(File file)
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

      maxLengthTimestamp = frameGrabber.getLengthInTime();
   }

   public Frame getCurrentFrame()
   {
      return currentFrame;
   }

   public long readFrameAtTimestamp(long timestamp)
   {
      if (timestamp != currentTimestamp.get())
      {
         long clampedTime = Math.min(maxLengthTimestamp, Math.max(0, timestamp));
         currentTimestamp.set(clampedTime);
         try
         {
            frameGrabber.setVideoTimestamp(timestamp);
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
      return maxLengthTimestamp / 1000000;
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

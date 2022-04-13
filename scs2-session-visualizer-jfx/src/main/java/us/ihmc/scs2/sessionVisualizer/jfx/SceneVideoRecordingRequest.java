package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;

public class SceneVideoRecordingRequest
{
   private File file;
   private double frameRate = 60.0;
   private double realTimeRate = 1.0;
   private int width = 1920;
   private int height = 1080;

   private Runnable recordingStartedCallback = null;
   private Runnable recordingEndedCallback = null;

   public SceneVideoRecordingRequest()
   {
   }

   public File getFile()
   {
      return file;
   }

   public double getFrameRate()
   {
      return frameRate;
   }

   public double getRealTimeRate()
   {
      return realTimeRate;
   }

   public int getWidth()
   {
      return width;
   }

   public int getHeight()
   {
      return height;
   }

   public Runnable getRecordingStartedCallback()
   {
      return recordingStartedCallback;
   }

   public Runnable getRecordingEndedCallback()
   {
      return recordingEndedCallback;
   }

   public void setFile(File file)
   {
      this.file = file;
   }

   public void setFrameRate(double frameRate)
   {
      this.frameRate = frameRate;
   }

   public void setRealTimeRate(double realTimeRate)
   {
      this.realTimeRate = realTimeRate;
   }

   public void setWidth(int width)
   {
      this.width = width;
   }

   public void setHeight(int height)
   {
      this.height = height;
   }

   public void setRecordingStartedCallback(Runnable recordingStartedCallback)
   {
      this.recordingStartedCallback = recordingStartedCallback;
   }

   public void setRecordingEndedCallback(Runnable recordingEndedCallback)
   {
      this.recordingEndedCallback = recordingEndedCallback;
   }

   @Override
   public String toString()
   {
      return "[file=" + file + ", frameRate=" + frameRate + ", realTimeRate=" + realTimeRate + ", width=" + width + ", height=" + height + "]";
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;

public class SceneVideoRecordingRequest
{
   private File file;
   private int bufferStart = -1;
   private int bufferEnd = -1;
   private double frameRate = 60.0;
   private double realTimeRate = 1.0;
   private int width = 1920;
   private int height = 1080;

   public SceneVideoRecordingRequest()
   {
   }

   public File getFile()
   {
      return file;
   }

   public int getBufferStart()
   {
      return bufferStart;
   }

   public int getBufferEnd()
   {
      return bufferEnd;
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

   public void setFile(File file)
   {
      this.file = file;
   }

   public void setBufferStart(int bufferStart)
   {
      this.bufferStart = bufferStart;
   }

   public void setBufferEnd(int bufferEnd)
   {
      this.bufferEnd = bufferEnd;
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

   @Override
   public String toString()
   {
      return "[file=" + file + ", bufferStart=" + bufferStart + ", bufferEnd=" + bufferEnd + ", frameRate=" + frameRate + ", realTimeRate=" + realTimeRate
            + ", width=" + width + ", height=" + height + "]";
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;

/**
 * Requests for capturing a video of the 3D scene from the playback data.
 * 
 * @author Sylvain Bertrand
 */
public class SceneVideoRecordingRequest
{
   /**
    * The target file where the video is to be written. It is required and the file extension should be
    * {@link SessionVisualizerIOTools#videoFileExtension}.
    */
   private File file;
   /** The desired number of frames per second. */
   private double frameRate = 60.0;
   /** The playback speed. */
   private double realTimeRate = 1.0;
   /** Video size: width in number of pixels. */
   private int width = 1920;
   /** Video size: height in number of pixels. */
   private int height = 1080;

   /** Optional callback that will be invoked right before starting the video capture. */
   private Runnable recordingStartedCallback = null;
   /** Optional callback that will be invoked when the video capture just finished. */
   private Runnable recordingEndedCallback = null;

   /** Creates a new request, the target file has to be set before submitting this request. */
   public SceneVideoRecordingRequest()
   {
   }

   /**
    * Creates a new request ready to be submitted.
    * <p>
    * The file extension should be {@value SessionVisualizerIOTools#videoFileExtension}.
    * </p>
    * 
    * @param file the target file where the video is to be written.
    */
   public SceneVideoRecordingRequest(File file)
   {
      setFile(file);
   }

   /**
    * Gets the target file where the video is to be written.
    * 
    * @return the target file where the video is to be written.
    */
   public File getFile()
   {
      return file;
   }

   /**
    * Get the video desired frames per second.
    * 
    * @return the desired frames per second.
    */
   public double getFrameRate()
   {
      return frameRate;
   }

   /**
    * Gets the desired playback speed.
    * 
    * @return the desired playback speed.
    */
   public double getRealTimeRate()
   {
      return realTimeRate;
   }

   /**
    * Gets the desired video width in pixels.
    * 
    * @return the desired video width in pixels.
    */
   public int getWidth()
   {
      return width;
   }

   /**
    * Gets the desired video height in pixels.
    * 
    * @return the desired video height in pixels.
    */
   public int getHeight()
   {
      return height;
   }

   /**
    * Gets the callback to be invoked before the video capture starts.
    * 
    * @return the callback to be invoked before the video capture starts.
    */
   public Runnable getRecordingStartedCallback()
   {
      return recordingStartedCallback;
   }

   /**
    * Gets the callback to be invoked after the video capture finishes.
    * 
    * @return the callback to be invoked after the video capture finishes.
    */
   public Runnable getRecordingEndedCallback()
   {
      return recordingEndedCallback;
   }

   /**
    * Sets the target file where the video is to be written. It is required and the file extension
    * should be {@link SessionVisualizerIOTools#videoFileExtension}.
    * 
    * @param file the target file where the video is to be written.
    */
   public void setFile(File file)
   {
      this.file = file;
   }

   /**
    * Sets the desired number of frames per second.
    * 
    * @param frameRate the desired number of frames per second.
    */
   public void setFrameRate(double frameRate)
   {
      this.frameRate = frameRate;
   }

   /**
    * Sets the desired playback speed.
    * 
    * @param realTimeRate the desired playback speed.
    */
   public void setRealTimeRate(double realTimeRate)
   {
      this.realTimeRate = realTimeRate;
   }

   /**
    * Sets the desired video width in pixels.
    * 
    * @param width the desired video width in pixels.
    */
   public void setWidth(int width)
   {
      this.width = width;
   }

   /**
    * Sets the desired video height in pixels.
    * 
    * @param height the desired video height in pixels.
    */
   public void setHeight(int height)
   {
      this.height = height;
   }

   /**
    * Sets the callback that will be invoked right before starting the video capture.
    * 
    * @param recordingStartedCallback the callback that will be invoked right before starting the video
    *                                 capture.
    */
   public void setRecordingStartedCallback(Runnable recordingStartedCallback)
   {
      this.recordingStartedCallback = recordingStartedCallback;
   }

   /**
    * Sets the callback that will be invoked when the video capture just finished.
    * 
    * @param recordingEndedCallback the callback that will be invoked when the video capture just
    *                               finished.
    */
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

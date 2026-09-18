package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import logger_msgs.Camera;
import logger_msgs.LogProperties;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.scs2.session.DaemonThreadFactory;
import us.ihmc.scs2.session.log.ProgressConsumer;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class MultiVideoDataReader
{
   private final List<VideoDataReader> readers = new ArrayList<>();
   /**
    * Dedicated single-thread executor for {@link #readVideoFrameInBackground(long)}, instead of
    * sharing the general-purpose {@code BackgroundExecutorManager} pool used everywhere else in the
    * app.
    * <p>
    * This work is inherently sequential - only one call is ever in flight, guarded by
    * {@code currentTask.isDone()} below - and it fires on every timestamp change during
    * playback/scrubbing, i.e. essentially continuously. Submitting it to a shared multi-thread pool
    * meant each call could land on a different worker thread (a plain {@code ExecutorService} has no
    * thread affinity), which cold-started caches on a different core every time and defeated the
    * per-thread scratch-buffer reuse in {@link FrameImageConverter} - for work that was never actually
    * running in parallel with itself to begin with. Profiling showed most of the wall-clock time here
    * was that thread-hopping overhead, not the decode work itself.
    */
   private final ExecutorService dedicatedReadExecutor = Executors.newSingleThreadExecutor(new DaemonThreadFactory("MultiVideoDataReader"));
   private Future<?> currentTask = null;

   public MultiVideoDataReader(File dataDirectory, LogProperties logProperties)
   {
      IDLObjectSequence<Camera> cameras = logProperties.getCameras();

      for (int i = 0; i < cameras.size(); i++)
      {
         Camera camera = cameras.get(i);
         try
         {
            VideoDataReader reader;
            if (isMagewellCamera(camera))
            {
               reader = new MagewellVideoDataReader(camera, dataDirectory, logProperties.getVideo().getHasTimebase());
            }
            else if (isBlackMagicCamera(camera))
            {
               reader = new BlackMagicVideoDataReader(camera, dataDirectory, logProperties.getVideo().getHasTimebase());
            }
            else
            {  // Older logs won't have the camera type set correctly, if there isn't a type set this as the only option
               reader = new BlackMagicVideoDataReader(camera, dataDirectory, logProperties.getVideo().getHasTimebase());
            }

            readers.add(reader);
         }
         catch (IOException e)
         {
            System.err.println(e.getMessage());
         }
      }

      try
      {
         for (File zedSensorDatFile : ZEDSVOScrubber.findZEDSensorDatFiles(dataDirectory))
         {
            VideoDataReader reader = new ZEDSVOVideoDataReader(zedSensorDatFile);
            readers.add(reader);
         }
      }
      catch (Throwable t)
      {
         // The ZED SDK is not available on all platforms (e.g. macOS). ZEDSVOScrubber already checks a
         // ZED_SDK_LOADED flag before returning any files, but a missing us.ihmc:zed native library can
         // still throw an Error (UnsatisfiedLinkError, NoClassDefFoundError, ExceptionInInitializerError)
         // out of that class's static initializer, which a plain "catch (Exception e)" would not catch.
         System.err.println("Skipping ZED video data, ZED SDK unavailable: " + t.getMessage());
      }
   }

   public void readVideoFrameNow(long queryRobotTimestamp)
   {
      readers.forEach(reader -> reader.readVideoFrame(queryRobotTimestamp));
   }

   public void readVideoFrameInBackground(long queryRobotTimestamp)
   {
      if (currentTask == null || currentTask.isDone())
         currentTask = dedicatedReadExecutor.submit(() ->
         {
            try
            {
               readVideoFrameNow(queryRobotTimestamp);
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         });
   }

   /**
    * Stops the dedicated background thread used by {@link #readVideoFrameInBackground(long)}. Safe to
    * skip if this reader is simply discarded - the thread is a daemon and does nothing once idle - but
    * call this when a session ends to release it promptly instead of leaving it parked.
    */
   public void shutdown()
   {
      dedicatedReadExecutor.shutdownNow();
   }

   public void crop(File selectedDirectory, long startTimestamp, long endTimestamp, ProgressConsumer progressConsumer) throws IOException
   {
      crop(selectedDirectory, readers, startTimestamp, endTimestamp, progressConsumer);
   }

   public static void crop(File selectedDirectory, List<VideoDataReader> videoDataReaders, long startTimestamp, long endTimestamp, ProgressConsumer progressConsumer) throws IOException
   {
      ProgressConsumer subProgressConsumer = null;

      for (int i = 0; i < videoDataReaders.size(); i++)
      {
         VideoDataReader reader = videoDataReaders.get(i);
         Camera camera = reader.getCamera();

         if (progressConsumer != null)
         {
            progressConsumer.info("Cropping video (%s)".formatted(camera.getVideoFileAsString()));
            double progressPercentage = (double) i / (double) videoDataReaders.size();
            progressConsumer.progress(progressPercentage);
            subProgressConsumer = progressConsumer.subProgress("Cropping video (%s): ".formatted(camera.getVideoFileAsString()),
                                                               progressPercentage,
                                                               (i + 1.0) / videoDataReaders.size());
         }

         File timestampFile = new File(selectedDirectory, camera.getTimestampFileAsString());
         File videoFile = new File(selectedDirectory, camera.getVideoFileAsString());
         reader.cropVideo(videoFile, timestampFile, startTimestamp, endTimestamp, subProgressConsumer);
      }
   }

   public int getNumberOfVideos()
   {
      return readers.size();
   }

   public List<VideoDataReader> getReaders()
   {
      return readers;
   }

   static boolean isMagewellCamera(Camera camera)
   {
      // logger_msgs Camera.type strings (legacy us.ihmc.robotDataLogger.CameraType values)
      String type = camera.getTypeAsString();
      return "CAPTURE_CARD_MAGEWELL".equals(type) || "Magewell".equals(type);
   }

   static boolean isBlackMagicCamera(Camera camera)
   {
      String type = camera.getTypeAsString();
      return "CAPTURE_CARD".equals(type) || "Capture Card".equals(type);
   }
}

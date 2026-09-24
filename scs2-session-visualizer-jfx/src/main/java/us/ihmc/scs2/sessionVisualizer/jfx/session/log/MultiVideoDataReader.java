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
   /**
    * Single thread shared by all the video readers, such that video frames are always read from the same thread instead of hopping between the threads of the
    * general purpose background pool. Each read is short, so sharing the thread between the readers of a main log and its added logs is not a concern.
    */
   private static final ExecutorService VIDEO_READ_EXECUTOR = Executors.newSingleThreadExecutor(new DaemonThreadFactory(MultiVideoDataReader.class.getSimpleName()));

   private final List<VideoDataReader> readers = new ArrayList<>();
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
      {
         currentTask = VIDEO_READ_EXECUTOR.submit(() ->
         { // A task submitted to an executor swallows its exceptions, so they need to be printed here.
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

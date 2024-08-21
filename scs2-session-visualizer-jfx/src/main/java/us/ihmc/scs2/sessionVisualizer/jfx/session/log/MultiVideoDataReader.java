package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Future;

import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataLogger.CameraType;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.scs2.session.log.ProgressConsumer;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;

public class MultiVideoDataReader
{
   private final List<VideoDataReader> readers = new ArrayList<>();
   private final BackgroundExecutorManager backgroundExecutorManager;
   private Future<?> currentTask = null;

   public MultiVideoDataReader(File dataDirectory, LogProperties logProperties, BackgroundExecutorManager backgroundExecutorManager)
   {
      this.backgroundExecutorManager = backgroundExecutorManager;
      List<Camera> cameras = logProperties.getCameras();

      for (int i = 0; i < cameras.size(); i++)
      {
         Camera camera = cameras.get(i);
         try
         {
            VideoDataReader reader;
            if (camera.getType().toString().equals(CameraType.CAPTURE_CARD_MAGEWELL.toString()))
            {
               reader = new MagewellVideoDataReader(camera, dataDirectory, logProperties.getVideo().getHasTimebase());
            }
            else if (camera.getType().toString().equals(CameraType.CAPTURE_CARD.toString()))
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
   }

   public void readVideoFrameNow(long queryRobotTimestamp)
   {
      readers.forEach(reader -> reader.readVideoFrame(queryRobotTimestamp));
   }

   public void readVideoFrameInBackground(long queryRobotTimestamp)
   {
      if (currentTask == null || currentTask.isDone())
         currentTask = backgroundExecutorManager.executeInBackground(() -> readVideoFrameNow(queryRobotTimestamp));
   }

   public void crop(File selectedDirectory, long startTimestamp, long endTimestamp, ProgressConsumer progressConsumer) throws IOException
   {
      ProgressConsumer subProgressConsumer = null;

      for (int i = 0; i < readers.size(); i++)
      {
         VideoDataReader reader = readers.get(i);
         Camera camera = reader.getCamera();

         if (progressConsumer != null)
         {
            progressConsumer.info("Cropping video (%s)".formatted(camera.getVideoFileAsString()));
            double progressPercentage = (double) i / (double) readers.size();
            progressConsumer.progress(progressPercentage);
            subProgressConsumer = progressConsumer.subProgress("Cropping video (%s): ".formatted(camera.getVideoFileAsString()),
                                                               progressPercentage,
                                                               (i + 1.0) / readers.size());
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
}

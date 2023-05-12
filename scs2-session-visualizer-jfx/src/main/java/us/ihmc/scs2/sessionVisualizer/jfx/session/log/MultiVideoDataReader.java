package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Future;

import us.ihmc.robotDataLogger.Camera;
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
            VideoDataReader reader = new VideoDataReader(camera, dataDirectory, logProperties.getVideo().getHasTimebase());
            readers.add(reader);
         }
         catch (IOException e)
         {
            System.err.println(e.getMessage());
         }
      }
   }

   public void readVideoFrameNow(long givenRobotTimestamp)
   {
      readers.forEach(reader -> reader.readVideoFrame(givenRobotTimestamp));
   }

   public void readVideoFrameInBackground(long givenRobotTimestamp)
   {
      if (currentTask == null || currentTask.isDone())
         currentTask = backgroundExecutorManager.executeInBackground(() -> readVideoFrameNow(givenRobotTimestamp));
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
            progressConsumer.info("Cropping video " + camera.getNameAsString());
            double progressPercentage = (double) i / (double) readers.size();
            progressConsumer.progress(progressPercentage);
            subProgressConsumer = progressConsumer.subProgress(progressPercentage, (i + 1.0) / readers.size());
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

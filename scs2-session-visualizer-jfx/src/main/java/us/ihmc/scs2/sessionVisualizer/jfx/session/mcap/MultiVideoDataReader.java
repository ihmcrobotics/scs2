package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.javacv.FFmpegFrameGrabber;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Future;

public class MultiVideoDataReader
{
   static
   {
      // TODO: (AM) hack to suppress warnings from https://github.com/bytedeco/javacv/issues/780
      avutil.av_log_set_level(avutil.AV_LOG_ERROR);
   }

   private final List<VideoDataReader> readers = new ArrayList<>();
   private final BackgroundExecutorManager backgroundExecutorManager;
   private Future<?> currentTask = null;

   public MultiVideoDataReader(File dataDirectory, BackgroundExecutorManager backgroundExecutorManager)
   {
      this.backgroundExecutorManager = backgroundExecutorManager;
      List<Path> videoFiles;
      LogTools.info("Searching for videos in {}", dataDirectory.getAbsolutePath());
      if (dataDirectory.isDirectory())
      {
         try
         {
            videoFiles = Files.walk(dataDirectory.toPath(), 1).filter(f -> f.toString().endsWith(".mp4") | f.toString().endsWith(".avi")).toList();
            LogTools.info("Found video file(s): {}", videoFiles.toString());
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
         for (int i = 0; i < videoFiles.size(); i++)
         {
            readers.add(new VideoDataReader(videoFiles.get(i).toFile()));
         }
      }
   }

   public void readVideoFrameNow(double time)
   {
      readers.forEach(reader ->
                      {
                         reader.readFrameAtTimestamp(Math.round(time * 1000000L));
//                         LogTools.info("Reading frame at {} out of {}\n", time, reader.getVideoLengthInSeconds());
                      });
   }

   public void readVideoFrameInBackground(double time)
   {
      if (currentTask == null || currentTask.isDone())
         currentTask = backgroundExecutorManager.executeInBackground(() -> readVideoFrameNow(time));
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

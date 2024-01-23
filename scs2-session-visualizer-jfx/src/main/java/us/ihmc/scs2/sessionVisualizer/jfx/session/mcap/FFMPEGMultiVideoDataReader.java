package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import org.bytedeco.ffmpeg.global.avutil;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Future;

public class FFMPEGMultiVideoDataReader
{
   static
   {
      // TODO: (AM) hack to suppress warnings from https://github.com/bytedeco/javacv/issues/780
      avutil.av_log_set_level(avutil.AV_LOG_ERROR);
   }

   private final List<FFMPEGVideoDataReader> readers = new ArrayList<>();
   private final BackgroundExecutorManager backgroundExecutorManager;
   private Future<?> currentTask = null;

   public FFMPEGMultiVideoDataReader(File dataDirectory, BackgroundExecutorManager backgroundExecutorManager)
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
            readers.add(new FFMPEGVideoDataReader(videoFiles.get(i).toFile()));
         }
      }
   }

   public void readVideoFrameNow(long timestamp)
   {
      readers.forEach(reader ->
                      {
                         reader.readFrameAtTimestamp(timestamp);
                         //                          LogTools.info("Reading frame at {} out of {}\n", timestamp / 1000000000.0, reader.getVideoLengthInSeconds());
                      });
   }

   public void readVideoFrameInBackground(long timestamp)
   {
      if (currentTask == null || currentTask.isDone())
         currentTask = backgroundExecutorManager.executeInBackground(() -> readVideoFrameNow(timestamp));
   }

   public int getNumberOfVideos()
   {
      return readers.size();
   }

   public List<FFMPEGVideoDataReader> getReaders()
   {
      return readers;
   }
}

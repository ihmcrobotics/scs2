package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import org.bytedeco.ffmpeg.global.avutil;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.DaemonThreadFactory;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class FFMPEGMultiVideoDataReader
{
   static
   {
      // TODO: (AM) hack to suppress warnings from https://github.com/bytedeco/javacv/issues/780
      avutil.av_log_set_level(avutil.AV_LOG_ERROR);
   }

   private final List<FFMPEGVideoDataReader> readers = new ArrayList<>();
   /**
    * Dedicated single-thread executor for {@link #readVideoFrameInBackground(long)} - see
    * {@code MultiVideoDataReader}'s field of the same name for why this isn't the shared
    * {@code BackgroundExecutorManager} pool.
    */
   private final ExecutorService dedicatedReadExecutor = Executors.newSingleThreadExecutor(new DaemonThreadFactory("FFMPEGMultiVideoDataReader"));
   private Future<?> currentTask = null;

   public FFMPEGMultiVideoDataReader(File dataDirectory)
   {
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
                      });
   }

   public void readVideoFrameInBackground(long timestamp)
   {
      if (currentTask == null || currentTask.isDone())
         currentTask = dedicatedReadExecutor.submit(() ->
         {
            try
            {
               readVideoFrameNow(timestamp);
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

   public int getNumberOfVideos()
   {
      return readers.size();
   }

   public List<FFMPEGVideoDataReader> getReaders()
   {
      return readers;
   }
}

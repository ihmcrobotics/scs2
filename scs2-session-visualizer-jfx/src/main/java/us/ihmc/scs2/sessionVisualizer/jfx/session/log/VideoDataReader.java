package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

import gnu.trove.list.array.TLongArrayList;
import javafx.scene.image.WritableImage;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.scs2.session.log.ProgressConsumer;

public class VideoDataReader
{
   private final TimestampScrubber timestampScrubber;

   private final String name;

   private final MP4VideoDemuxer demuxer;
   private final JavaFXPictureConverter converter = new JavaFXPictureConverter();

   private final File videoFile;
   private final Camera camera;
   private final ConcurrentCopier<FrameData> imageBuffer = new ConcurrentCopier<>(FrameData::new);

   public VideoDataReader(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {

      this.camera = camera;
      name = camera.getNameAsString();
      boolean interlaced = camera.getInterlaced();

      if (!hasTimeBase)
      {
         System.err.println("Video data is using timestamps instead of frame numbers. Falling back to seeking based on timestamp.");
      }

      videoFile = new File(dataDirectory, camera.getVideoFileAsString());

      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());

      demuxer = new MP4VideoDemuxer(videoFile);

      this.timestampScrubber = new TimestampScrubber(timestampFile, hasTimeBase, interlaced);
   }

   public void readVideoFrame(long givenRobotTimestamp)
   {
      long videoTimestamp = timestampScrubber.getVideoTimestamp(givenRobotTimestamp);
      long currentlyShowingRobotTimestamp = timestampScrubber.getCurrentlyShowingRobotTimestamp();

      try
      {
         demuxer.seekToPTS(videoTimestamp);
         YUVPicture nextFrame = demuxer.getNextFrame();
         FrameData copyForWriting = imageBuffer.getCopyForWriting();
         copyForWriting.frame = converter.toFXImage(nextFrame, copyForWriting.frame);
         copyForWriting.givenRobotTimestamp = givenRobotTimestamp;
         copyForWriting.cameraCurrentPTS = videoTimestamp;
         copyForWriting.demuxerCurrentPTS = demuxer.getCurrentPTS();
         copyForWriting.robotTimestamp = currentlyShowingRobotTimestamp;

         imageBuffer.commit();
      } catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor) throws IOException
   {
      long startVideoTimestamp = timestampScrubber.getVideoTimestampWithBinarySearch(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getVideoTimestampWithBinarySearch(endTimestamp);

      int framerate = VideoConverter.crop(videoFile, outputFile, startVideoTimestamp, endVideoTimestamp, monitor);

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(1);
      timestampWriter.println(framerate);

      long pts = 0;
      /*
       * PTS gets reordered to be monotonically increasing starting from 0
       */
      for (int i = 0; i < timestampScrubber.getRobotTimestampsLength(); i++)
      {
         long robotTimestamp = timestampScrubber.getRobotTimestampAtIndex(i);

         if (robotTimestamp >= startTimestamp && robotTimestamp <= endTimestamp)
         {

            timestampWriter.print(robotTimestamp);
            timestampWriter.print(" ");
            timestampWriter.println(pts);
            pts++;
         } else if (robotTimestamp > endTimestamp)
         {
            break;
         }
      }

      timestampWriter.close();
   }

   public void exportVideo(File selectedFile, long startTimestamp, long endTimestamp, ProgressConsumer progreesConsumer)
   {
      long startVideoTimestamp = timestampScrubber.getVideoTimestampWithBinarySearch(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getVideoTimestampWithBinarySearch(endTimestamp);

      try
      {
         VideoConverter.convert(videoFile, selectedFile, startVideoTimestamp, endVideoTimestamp, progreesConsumer);
      } catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public String getName()
   {
      return name;
   }

   public Camera getCamera()
   {
      return camera;
   }

   public FrameData pollCurrentFrame()
   {
      return imageBuffer.getCopyForReading();
   }

   public static class FrameData
   {
      public WritableImage frame;
      public long givenRobotTimestamp;
      public long robotTimestamp;
      public long cameraCurrentPTS;
      public long demuxerCurrentPTS;
   }

   public static class TimestampScrubber
   {
      private final boolean hasTimebase;
      private final boolean interlaced;
      private long[] robotTimestamps;
      private long[] videoTimestamps;

      private int currentIndex = 0;
      private long currentlyShowingRobotTimestamp = 0;
      private long upcomingRobotTimestamp = 0;

      private long videoTimestamp;

      public TimestampScrubber(File timestampFile, boolean hasTimebase, boolean interlaced) throws IOException
      {
         this.hasTimebase = hasTimebase;
         this.interlaced = interlaced;

         parseTimestampData(timestampFile);
      }

      private void parseTimestampData(File timestampFile) throws IOException
      {
         try (BufferedReader bufferedReader = new BufferedReader(new FileReader(timestampFile)))
         {

            String line;
            if (hasTimebase)
            {
               if (bufferedReader.readLine() == null)
               {
                  throw new IOException("Cannot read numerator");
               }

               if (bufferedReader.readLine() == null)
               {
                  throw new IOException("Cannot read denumerator");
               }
            }

            TLongArrayList robotTimestamps = new TLongArrayList();
            TLongArrayList videoTimestamps = new TLongArrayList();

            while ((line = bufferedReader.readLine()) != null)
            {
               String[] stamps = line.split("\\s");
               long robotStamp = Long.parseLong(stamps[0]);
               long videoStamp = Long.parseLong(stamps[1]);

               if (interlaced)
               {
                  videoStamp /= 2;
               }

               robotTimestamps.add(robotStamp);
               videoTimestamps.add(videoStamp);
            }

            this.robotTimestamps = robotTimestamps.toArray();
            this.videoTimestamps = videoTimestamps.toArray();

         } catch (FileNotFoundException e)
         {
            throw new RuntimeException(e);
         }
      }

      /**
       * Searchs the list of robotTimestamps for the value closest to givenRobotTimestamp and sets the currentIndex to that index. Then sets videoTimestamp to
       * that index in oder to display the right frame.
       * @param givenRobotTimestamp the value sent from the robot data in which we wnat to find the closest robotTimestamp in the timestamp file.
       * @return the videoTimestamp that matches the index of the closest robotTimestamp in our timestamp file.
       */

      public long getVideoTimestamp(long givenRobotTimestamp)
      {
         if (givenRobotTimestamp < currentlyShowingRobotTimestamp || givenRobotTimestamp >= upcomingRobotTimestamp)
         {
            videoTimestamp = getVideoTimestampWithBinarySearch(givenRobotTimestamp);

            if (currentIndex + 1 < robotTimestamps.length)
               upcomingRobotTimestamp = robotTimestamps[currentIndex + 1];
            else
               upcomingRobotTimestamp = currentlyShowingRobotTimestamp;
         }

         return videoTimestamp;
      }

      private long getVideoTimestampWithBinarySearch(long givenRobotTimestamp)
      {
         if (givenRobotTimestamp <= robotTimestamps[0])
         {
            currentIndex = 0;
            return videoTimestamps[currentIndex];
         }

         if (givenRobotTimestamp >= robotTimestamps[robotTimestamps.length-1])
         {
            currentIndex = robotTimestamps.length - 2;
            return videoTimestamps[currentIndex];
         }

         currentIndex = Arrays.binarySearch(robotTimestamps, givenRobotTimestamp);

         if (currentIndex < 0)
         {
            int nextIndex = -currentIndex - 1; // insertionPoint
            currentIndex = nextIndex;
         }

         currentlyShowingRobotTimestamp = robotTimestamps[currentIndex];
         return videoTimestamps[currentIndex];
      }

      public long getCurrentlyShowingRobotTimestamp()
      {
         return currentlyShowingRobotTimestamp;
      }

      public int getRobotTimestampsLength()
      {
         return robotTimestamps.length;
      }

      public long getRobotTimestampAtIndex(int i)
      {
         return robotTimestamps[i];
      }

      public long[] getRobotTimestampsFromFile()
      {
         return robotTimestamps;
      }

      public long[] getVideoTimestampsFromFile()
      {
         return videoTimestamps;
      }

      public long getCurrentVideoTimestamp()
      {
         return videoTimestamp;
      }
   }
}

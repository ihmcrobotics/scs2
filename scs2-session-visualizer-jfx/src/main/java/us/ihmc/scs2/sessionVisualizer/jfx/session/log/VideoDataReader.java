package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gnu.trove.list.array.TLongArrayList;
import javafx.scene.image.WritableImage;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.tools.EuclidCoreTools;
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

   public void readVideoFrame(long queryRobotTimestamp)
   {
      long videoTimestamp = timestampScrubber.getVideoTimestamp(queryRobotTimestamp);
      long currentRobotTimestamp = timestampScrubber.getCurrentRobotTimestamp();

      try
      {
         demuxer.seekToPTS(videoTimestamp);
         FrameData copyForWriting = imageBuffer.getCopyForWriting();
         copyForWriting.queryRobotTimestamp = queryRobotTimestamp;
         copyForWriting.robotTimestamp = currentRobotTimestamp;
         copyForWriting.cameraCurrentPTS = videoTimestamp;
         copyForWriting.demuxerCurrentPTS = demuxer.getCurrentPTS();
         YUVPicture nextFrame = demuxer.getNextFrame(); // Increment frame index after getting frame.
         copyForWriting.frame = converter.toFXImage(nextFrame, copyForWriting.frame);

         imageBuffer.commit();
      } catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor) throws IOException
   {
      long startVideoTimestamp = timestampScrubber.searchRobotTimestampIndex(startTimestamp);
      long endVideoTimestamp = timestampScrubber.searchRobotTimestampIndex(endTimestamp);

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
      long startVideoTimestamp = timestampScrubber.searchRobotTimestampIndex(startTimestamp);
      long endVideoTimestamp = timestampScrubber.searchRobotTimestampIndex(endTimestamp);

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

   public boolean replacedRobotTimestampsContainsIndex(int index)
   {
      return timestampScrubber.replacedRobotTimestampIndexes.contains(index);
   }

   public int getIndex()
   {
      return timestampScrubber.getIndex();
   }

   public static class FrameData
   {
      public WritableImage frame;
      public long queryRobotTimestamp;
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

      private long currentRobotTimestamp = 0;
      private long upcomingRobotTimestamp = 0;

      private long videoTimestamp;
      private int currentIndex = 0;

      public List<Integer> replacedRobotTimestampIndexes = new ArrayList<>();

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

         }
         catch (FileNotFoundException e)
         {
            throw new RuntimeException(e);
         }

         checkAndReplaceDuplicates();
      }

      public void checkAndReplaceDuplicates()
      {
         for (int index = 0; index < robotTimestamps.length - 1; index++)
         {
            if (robotTimestamps[index] == robotTimestamps[index + 1] && index + 2 < robotTimestamps.length)
            {
               int firstDuplicateIndex = index;
               int nextNonDuplicateIndex = getNextNonDuplicateIndex(index);
               int numberOfDuplicates = nextNonDuplicateIndex - firstDuplicateIndex;

               long firstAdjustedTimestamp = (long) EuclidCoreTools.interpolate(robotTimestamps[firstDuplicateIndex], robotTimestamps[nextNonDuplicateIndex], (double) 1 / numberOfDuplicates);
               long deltaValue = firstAdjustedTimestamp - robotTimestamps[firstDuplicateIndex];

               index = replaceDuplicateTimestampsAndIncrementIndex(firstDuplicateIndex, nextNonDuplicateIndex - 1, deltaValue);
            }
         }
      }

      public int getNextNonDuplicateIndex(int index)
      {
         while (robotTimestamps[index] == robotTimestamps[index + 1] && index + 2 < robotTimestamps.length)
            index++;

         return index + 1;
      }

      private int replaceDuplicateTimestampsAndIncrementIndex(int firstDuplicateIndex, int lastDuplicateIndex, long deltaValue)
      {
         // Don't replace duplicates at end of file since the controller has already stopped and this data is junk
         if (lastDuplicateIndex + 2 >= robotTimestamps.length)
            return robotTimestamps.length;

         int currentIndex = firstDuplicateIndex + 1;
         while (currentIndex <= lastDuplicateIndex)
         {
            int deltaIndex = currentIndex - firstDuplicateIndex;
            long additionalValue = deltaValue * deltaIndex;

            robotTimestamps[currentIndex] = robotTimestamps[currentIndex] + additionalValue;
            currentIndex++;
         }

         // Prevents looping through the currentIndex's we have already changed in this method, ( - 1) because the for loop increments once
         return currentIndex - 1;
      }

      /**
       * Searches the list of robotTimestamps for the value closest to queryRobotTimestamp and returns that index. Then sets videoTimestamp to
       * that index in oder to display the right frame.
       * @param queryRobotTimestamp the value sent from the robot data in which we want to find the closest robotTimestamp in the timestamp file.
       * @return the videoTimestamp that matches the index of the closest robotTimestamp in our timestamp file.
       */
      public long getVideoTimestamp(long queryRobotTimestamp)
      {
         if (queryRobotTimestamp < currentRobotTimestamp || queryRobotTimestamp >= upcomingRobotTimestamp)
         {
            currentIndex = searchRobotTimestampIndex(queryRobotTimestamp);

            videoTimestamp = videoTimestamps[currentIndex];
            currentRobotTimestamp = robotTimestamps[currentIndex];

            if (currentIndex + 1 < robotTimestamps.length)
               upcomingRobotTimestamp = robotTimestamps[currentIndex + 1];
            else
               upcomingRobotTimestamp = currentRobotTimestamp;
         }

         return videoTimestamp;
      }

      private int searchRobotTimestampIndex(long queryRobotTimestamp)
      {
         int index;

         if (queryRobotTimestamp <= robotTimestamps[0])
            return 0;

         if (queryRobotTimestamp >= robotTimestamps[robotTimestamps.length-1])
            return robotTimestamps.length - 2;

         index = Arrays.binarySearch(robotTimestamps, queryRobotTimestamp);

         if (index < 0)
         {
            int nextIndex = -index - 1; // insertionPoint
            index = nextIndex;
         }

         return index;
      }

      public int getIndex()
      {
         return currentIndex;
      }

      public long getCurrentRobotTimestamp()
      {
         return currentRobotTimestamp;
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

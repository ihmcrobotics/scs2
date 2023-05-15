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
         copyForWriting.givenRobotTimestamp = queryRobotTimestamp;
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
      long startVideoTimestamp = timestampScrubber.getIndexWithBinarySearch(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getIndexWithBinarySearch(endTimestamp);

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
      long startVideoTimestamp = timestampScrubber.getIndexWithBinarySearch(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getIndexWithBinarySearch(endTimestamp);

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

      private long currentRobotTimestamp = 0;
      private long upcomingRobotTimestamp = 0;
      private long videoTimestamp;

      public List<Integer> alteredRobotTimestampIndexes = new ArrayList<>();

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
       * Searches the list of robotTimestamps for the value closest to queryRobotTimestamp returns that index. Then sets videoTimestamp to
       * that index in oder to display the right frame.
       * @param queryRobotTimestamp the value sent from the robot data in which we want to find the closest robotTimestamp in the timestamp file.
       * @return the videoTimestamp that matches the index of the closest robotTimestamp in our timestamp file.
       */
      public long getVideoTimestamp(long queryRobotTimestamp)
      {
         int index;

         if (queryRobotTimestamp < currentRobotTimestamp || queryRobotTimestamp >= upcomingRobotTimestamp)
         {
            index = getIndexWithBinarySearch(queryRobotTimestamp);

            videoTimestamp = videoTimestamps[index];
            currentRobotTimestamp = robotTimestamps[index];

            if (index + 1 < robotTimestamps.length)
               upcomingRobotTimestamp = robotTimestamps[index + 1];
            else
               upcomingRobotTimestamp = currentRobotTimestamp;
         }

         return videoTimestamp;
      }

      private int getIndexWithBinarySearch(long queryRobotTimestamp)
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

      // TODO have the method only be called when there are duplicates, otherwise it will get called all the time and inside then check, but we should
      // be able to checkout outside and only call the method if there are duplicates. That would clean up the method a bit.
      public void replaceDuplicateTimestamps(int currentIndex)
      {
         if (currentIndex + 1 >= robotTimestamps.length || currentIndex - 1 < 0)
            return;

         // Checks if the same robotTimestamps exists before or after the currentIndex
         if (robotTimestamps[currentIndex] == robotTimestamps[currentIndex + 1] || robotTimestamps[currentIndex] == robotTimestamps[currentIndex - 1])
         {
            boolean endOfFileReached = false;

            // Moves the currentIndex position to the beginning of the duplicated robotTimestamps
            while (robotTimestamps[currentIndex] == robotTimestamps[currentIndex - 1])
               currentIndex--;

            int nextIndex = currentIndex;
            int numberOfDuplicateRobotTimestamps = 1;

            // While we have duplicate robotTimestamps, add those indexes to a list, so we know which frames have been altered
            while (robotTimestamps[currentIndex] == robotTimestamps[nextIndex] && nextIndex + 1 < robotTimestamps.length)
            {
               nextIndex++;
               numberOfDuplicateRobotTimestamps++;

               if (!alteredRobotTimestampIndexes.contains(nextIndex))  // Only add new elements to the list of indexes we have altered
                  alteredRobotTimestampIndexes.add(nextIndex);
            }

            // Calculate the delta that should be added to a duplicated robotTimestamp
            long nextNonDuplicateRobotTimestamp = robotTimestamps[nextIndex];
            long totalDelta = nextNonDuplicateRobotTimestamp - robotTimestamps[currentIndex];
            long deltaAddedToEachDuplicateRobotTimestamp = totalDelta / numberOfDuplicateRobotTimestamps + 1; // add 1 because dividing by 1 for 2 duplicate frames doesn't give you half

            // We don't care about updating the robotTimestamps that get generated after the controller stops. So don't change them because they will never be displayed.
            if (!endOfFileReached)
            {
               int duplicateRobotTimestampOffset = 1; // Multiplier for which duplicate we are on
               for (int j = currentIndex; j < nextIndex; j++)
               {
                  robotTimestamps[j] = robotTimestamps[j] + deltaAddedToEachDuplicateRobotTimestamp * duplicateRobotTimestampOffset;
                  duplicateRobotTimestampOffset++;
               }
            }
         }
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

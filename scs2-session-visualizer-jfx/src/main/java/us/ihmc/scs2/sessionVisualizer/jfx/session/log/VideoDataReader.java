package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import gnu.trove.list.array.TLongArrayList;
import javafx.scene.image.WritableImage;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.scs2.session.log.ProgressConsumer;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

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

      demuxer = new MP4VideoDemuxer(videoFile);

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());
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
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor) throws IOException
   {
      long startVideoTimestamp = timestampScrubber.getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getVideoTimestamp(endTimestamp);

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
         }
         else if (robotTimestamp > endTimestamp)
         {
            break;
         }
      }

      timestampWriter.close();
   }

   public void exportVideo(File selectedFile, long startTimestamp, long endTimestamp, ProgressConsumer progreesConsumer)
   {
      long startVideoTimestamp = timestampScrubber.getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = timestampScrubber.getVideoTimestamp(endTimestamp);

      try
      {
         VideoConverter.convert(videoFile, selectedFile, startVideoTimestamp, endVideoTimestamp, progreesConsumer);
      }
      catch (IOException e)
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

   public int getCurrentIndex()
   {
      return timestampScrubber.getCurrentIndex();
   }

   public boolean replacedRobotTimestampsContainsIndex(int index)
   {
      return timestampScrubber.replacedRobotTimestampIndex[index];
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

      private int currentIndex = 0;
      private long currentRobotTimestamp = 0;
      private long videoTimestamp;
      private boolean[] replacedRobotTimestampIndex;

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

      private void checkAndReplaceDuplicates()
      {
         replacedRobotTimestampIndex = new boolean[robotTimestamps.length];
         int duplicatesAtEndOfFile = getNumberOfDuplicatesAtEndOfFile();

         for (int currentIndex = 0; currentIndex < robotTimestamps.length - duplicatesAtEndOfFile; )
         {
            if (robotTimestamps[currentIndex] != robotTimestamps[currentIndex + 1])
            {
               currentIndex++;
               continue;
            }

            // Keeps track of the duplicated index's so frames border can be adjusted
            replacedRobotTimestampIndex[currentIndex + 1] = true;

            int nextNonDuplicateIndex = getNextNonDuplicateIndex(currentIndex);
            for (int i = currentIndex; i < nextNonDuplicateIndex; i++)
            {
               long firstAdjustedTimestamp = (long) EuclidCoreTools.interpolate(robotTimestamps[i],
                                                                                robotTimestamps[nextNonDuplicateIndex],
                                                                                (double) 1 / (nextNonDuplicateIndex - i));
               robotTimestamps[i + 1] = firstAdjustedTimestamp;
            }

            currentIndex = nextNonDuplicateIndex;
         }
      }

      private int getNumberOfDuplicatesAtEndOfFile()
      {
         int duplicatesAtEndOfFile = 1;

         for (int i = robotTimestamps.length - 1; i > 0; i--)
         {
            if (robotTimestamps[i] == robotTimestamps[i - 1])
               duplicatesAtEndOfFile++;
         }

         return duplicatesAtEndOfFile;
      }

      private int getNextNonDuplicateIndex(int index)
      {
         while (index < robotTimestamps.length - 1 && robotTimestamps[index] == robotTimestamps[index + 1])
            index++;

         return index + 1;
      }

      /**
       * Searches the list of robotTimestamps for the value closest to queryRobotTimestamp and returns that index. Then sets videoTimestamp to
       * that index in oder to display the right frame.
       *
       * @param queryRobotTimestamp the value sent from the robot data in which we want to find the closest robotTimestamp in the instant file.
       * @return the videoTimestamp that matches the index of the closest robotTimestamp in our instant file.
       */
      public long getVideoTimestamp(long queryRobotTimestamp)
      {
         currentIndex = searchRobotTimestampsForIndex(queryRobotTimestamp);
         videoTimestamp = videoTimestamps[currentIndex];
         currentRobotTimestamp = robotTimestamps[currentIndex];

         return videoTimestamp;
      }

      private int searchRobotTimestampsForIndex(long queryRobotTimestamp)
      {
         if (queryRobotTimestamp <= robotTimestamps[0])
            return 0;

         if (queryRobotTimestamp >= robotTimestamps[robotTimestamps.length - 1])
            return robotTimestamps.length - 1;

         int index = Arrays.binarySearch(robotTimestamps, queryRobotTimestamp);

         if (index < 0)
         {
            int nextIndex = -index - 1; // insertionPoint
            index = nextIndex;
         }

         return index;
      }

      public int getCurrentIndex()
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

      public long[] getRobotTimestampsArray()
      {
         return robotTimestamps;
      }

      public long[] getVideoTimestampsArray()
      {
         return videoTimestamps;
      }

      public long getCurrentVideoTimestamp()
      {
         return videoTimestamp;
      }
   }
}

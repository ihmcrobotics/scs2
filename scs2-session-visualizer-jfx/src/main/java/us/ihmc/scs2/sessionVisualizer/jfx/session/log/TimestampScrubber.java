package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import gnu.trove.list.array.TLongArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;

public class TimestampScrubber
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
   public long getVideoTimestampFromRobotTimestamp(long queryRobotTimestamp)
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

   public long[] getCroppedRobotTimestamps(long startRobotTimestamp, long endRobotTimestamp)
   {
      int startIndex = findIndexOfRobotTimestamps(startRobotTimestamp);
      int endIndex = findIndexOfRobotTimestamps(endRobotTimestamp);

      return Arrays.copyOfRange(robotTimestamps, startIndex, endIndex + 1);
   }

   private int findIndexOfRobotTimestamps(long value)
   {
      if (value <= robotTimestamps[0])
         return 0;

      if (value >= robotTimestamps[robotTimestamps.length - 1])
         return robotTimestamps.length - 1;

      int index = Arrays.binarySearch(robotTimestamps, value);

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

   public long getVideoTimestampAtIndex(int i)
   {
      return videoTimestamps[i];
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

   public boolean getReplacedRobotTimestampIndex(int index)
   {
      return replacedRobotTimestampIndex[index];
   }
}

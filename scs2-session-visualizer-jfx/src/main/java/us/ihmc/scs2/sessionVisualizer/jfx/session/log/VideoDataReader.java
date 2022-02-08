package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

import org.apache.commons.lang3.mutable.MutableObject;

import gnu.trove.list.array.TLongArrayList;
import javafx.scene.image.WritableImage;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.scs2.session.log.ProgressConsumer;

public class VideoDataReader
{
   private final String name;
   private final boolean hasTimebase;
   private final boolean interlaced;

   private long[] robotTimestamps;
   private long[] videoTimestamps;

   private long bmdTimeBaseNum;
   private long bmdTimeBaseDen;

   private final MP4VideoDemuxer demuxer;
   private final JavaFXPictureConverter converter = new JavaFXPictureConverter();

   private int currentlyShowingIndex = 0;
   private long currentlyShowingRobotTimestamp = 0;
   private long upcomingRobotTimestamp = 0;

   private final File videoFile;
   private final Camera camera;
   private final ConcurrentCopier<MutableObject<WritableImage>> imageBuffer = new ConcurrentCopier<>(MutableObject::new);

   public VideoDataReader(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      this.camera = camera;
      name = camera.getNameAsString();
      interlaced = camera.getInterlaced();
      hasTimebase = hasTimeBase;

      if (!hasTimebase)
      {
         System.err.println("Video data is using timestamps instead of frame numbers. Falling back to seeking based on timestamp.");
      }

      videoFile = new File(dataDirectory, camera.getVideoFileAsString());

      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());

      parseTimestampData(timestampFile);

      demuxer = new MP4VideoDemuxer(videoFile);
   }

   public void readVideoFrame(long timestamp)
   {
      if (timestamp >= currentlyShowingRobotTimestamp && timestamp < upcomingRobotTimestamp)
      {
         return;
      }

      long previousTimestamp = videoTimestamps[currentlyShowingIndex];

      long videoTimestamp;
      if (robotTimestamps.length > currentlyShowingIndex + 1 && robotTimestamps[currentlyShowingIndex + 1] == timestamp)
      {
         currentlyShowingIndex++;
         videoTimestamp = videoTimestamps[currentlyShowingIndex];
         currentlyShowingRobotTimestamp = robotTimestamps[currentlyShowingIndex];

      }
      else
      {
         videoTimestamp = getVideoTimestamp(timestamp);
      }

      if (currentlyShowingIndex + 1 < robotTimestamps.length)
         upcomingRobotTimestamp = robotTimestamps[currentlyShowingIndex + 1];
      else
         upcomingRobotTimestamp = currentlyShowingRobotTimestamp;

      if (previousTimestamp == videoTimestamp)
         return;

      try
      {
         demuxer.seekToPTS(videoTimestamp);
         YUVPicture nextFrame = demuxer.getNextFrame();
         MutableObject<WritableImage> copyForWriting = imageBuffer.getCopyForWriting();
         copyForWriting.setValue(converter.toFXImage(nextFrame, copyForWriting.getValue()));
         
         imageBuffer.commit();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private long getVideoTimestamp(long timestamp)
   {
      currentlyShowingIndex = Arrays.binarySearch(robotTimestamps, timestamp);

      if (currentlyShowingIndex < 0)
      {
         int nextIndex = -currentlyShowingIndex + 1;
         if (nextIndex < robotTimestamps.length && Math.abs(robotTimestamps[-currentlyShowingIndex] - timestamp) > Math.abs(robotTimestamps[nextIndex]))
         {
            currentlyShowingIndex = nextIndex;
         }
         else
         {
            currentlyShowingIndex = -currentlyShowingIndex;
         }
      }

      if (currentlyShowingIndex < 0)
         currentlyShowingIndex = 0;
      if (currentlyShowingIndex >= robotTimestamps.length)
         currentlyShowingIndex = robotTimestamps.length - 1;
      currentlyShowingRobotTimestamp = robotTimestamps[currentlyShowingIndex];

      long videoTimestamp = videoTimestamps[currentlyShowingIndex];

      if (hasTimebase)
      {
         videoTimestamp = videoTimestamp * bmdTimeBaseNum * demuxer.getTimescale() / bmdTimeBaseDen;
      }

      return videoTimestamp;
   }

   private void parseTimestampData(File timestampFile) throws IOException
   {
      BufferedReader reader = null;
      try
      {
         reader = new BufferedReader(new FileReader(timestampFile));

         String line;
         if (hasTimebase)
         {
            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseNum = Long.valueOf(line);
            }
            else
            {
               throw new IOException("Cannot read numerator");
            }

            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseDen = Long.valueOf(line);
            }
            else
            {
               throw new IOException("Cannot read denumerator");
            }
         }

         TLongArrayList robotTimestamps = new TLongArrayList();
         TLongArrayList videoTimestamps = new TLongArrayList();

         while ((line = reader.readLine()) != null)
         {
            String[] stamps = line.split("\\s");
            long robotStamp = Long.valueOf(stamps[0]);
            long videoStamp = Long.valueOf(stamps[1]);

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
      finally
      {
         try
         {
            if (reader != null)
            {
               reader.close();
            }
         }
         catch (IOException e)
         {
         }
      }
   }

   public void exportVideo(File selectedFile, long startTimestamp, long endTimestamp, ProgressConsumer progreesConsumer)
   {

      long startVideoTimestamp = getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = getVideoTimestamp(endTimestamp);

      try
      {
         VideoConverter.convert(videoFile, selectedFile, startVideoTimestamp, endVideoTimestamp, progreesConsumer);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor) throws IOException
   {

      long startVideoTimestamp = getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = getVideoTimestamp(endTimestamp);

      int framerate = VideoConverter.crop(videoFile, outputFile, startVideoTimestamp, endVideoTimestamp, monitor);

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(1);
      timestampWriter.println(framerate);

      long pts = 0;
      /*
       * PTS gets reordered to be monotonically increasing starting from 0
       */
      for (int i = 0; i < robotTimestamps.length; i++)
      {
         long robotTimestamp = robotTimestamps[i];

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

   public String getName()
   {
      return name;
   }

   public Camera getCamera()
   {
      return camera;
   }

   public WritableImage pollCurrentFrame()
   {
      MutableObject<WritableImage> copyForReading = imageBuffer.getCopyForReading();
      if (copyForReading == null)
         return null;
      return copyForReading.getValue();
   }
}

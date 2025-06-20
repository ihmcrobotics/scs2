package us.ihmc.scs2.session.log;

import org.bytedeco.javacpp.Pointer;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.zed.SL_InitParameters;
import us.ihmc.zed.SL_RuntimeParameters;
import us.ihmc.zed.ZEDTools;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

import static us.ihmc.zed.global.zed.*;

/**
 * Reads video data from one or more ZED SDK SVO2 files recorded during the log.
 */
public class ZEDSVOScrubber
{
   private static final boolean ZED_SDK_LOADED;
   static
   {
      ZED_SDK_LOADED = ZEDJavaAPINativeLibrary.load();
   }

   private static final AtomicInteger NEXT_CAMERA_ID = new AtomicInteger(0);

   private final String sensorName;
   private final String perceptionDirectory;
   private final TimestampScrubber timestampScrubber;
   private long currentTimestamp;

   private record SVOFile(int cameraID, String currentVideoFilename, SL_InitParameters initParameters, SL_RuntimeParameters runtimeParameters) { }
   private final Map<String, SVOFile> svoFiles = new HashMap<>();
   private int imageWidth;
   private int imageHeight;
   private float fps;
   private Pointer leftColorImageSlMatPointer;
   private Pointer rightColorImageSlMatPointer;

   private record SVOToCrop(String svoFileName, boolean copyOnly, int startFrame, int endFrame) { }

   public static File[] findZEDSensorDatFiles(File logDataDirectory)
   {
      List<File> zedSensorDatFiles = new ArrayList<>();
      Path perceptionDirectory = logDataDirectory.toPath().resolve("perception");
      try
      {
         if (perceptionDirectory.toFile().exists())
         {
            File[] files = perceptionDirectory.toFile().listFiles();
            if (files != null)
            {
               Set<String> zedSensorNames = new TreeSet<>();
               for (File file : files)
               {
                  if (file.getName().endsWith(".svo2"))
                  {
                     String partAfterDate = file.getName().substring("yyyyMMdd_HHmmss_".length());
                     String partBeforeExtension = partAfterDate.substring(0, partAfterDate.length() - ".svo2".length());
                     zedSensorNames.add(partBeforeExtension);
                  }
               }

               for (String sensorName : zedSensorNames)
               {
                  for (File file : files)
                  {
                     if (file.getName().equals("%s_Timestamps.dat".formatted(sensorName)))
                     {
                        if (ZED_SDK_LOADED)
                        {
                           zedSensorDatFiles.add(file);
                        }
                        else
                        {
                           LogTools.warn("ZED sensor data is present but ZED SDK is not installed. ZED data will be excluded.");
                        }
                     }
                  }
               }
            }
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      return zedSensorDatFiles.toArray(new File[0]);
   }

   public ZEDSVOScrubber(File timestampsDatFile)
   {
      this.sensorName = timestampsDatFile.getName().replaceAll("_Timestamps.dat$", "");

      perceptionDirectory = timestampsDatFile.toPath().getParent().toAbsolutePath().toString();
      timestampScrubber = ExceptionTools.handle(() -> new TimestampScrubber(timestampsDatFile, false, false), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   int lastGrabbedFrameNumber = -1;

   public void scrub(long timestamp)
   {
      timestampScrubber.getVideoTimestampFromRobotTimestamp(timestamp);

      String videoFilename = timestampScrubber.getCurrentVideoFilename();
      SVOFile svoFile = svoFiles.get(videoFilename);

      if (svoFile == null)
      {
         int cameraID = NEXT_CAMERA_ID.getAndIncrement();

         String svoFileName = "%s%s%s_%s.svo2".formatted(perceptionDirectory, File.separator, videoFilename, sensorName);

         SL_InitParameters initParameters = new SL_InitParameters();
         initParameters.camera_device_id(cameraID);
         initParameters.input_type(SL_INPUT_TYPE_SVO);
         initParameters.sdk_verbose(0); // false
         initParameters.svo_real_time_mode(false);
         initParameters.coordinate_unit(SL_UNIT_METER);
         initParameters.coordinate_system(SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD);

         SL_RuntimeParameters runtimeParameters = new SL_RuntimeParameters();
         runtimeParameters.enable_depth(false);

         sl_create_camera(cameraID);

         LogTools.info("Opening SVO file: " + svoFileName); // Only print once per SVO file
         printOnError(sl_open_camera(cameraID, initParameters, 0, svoFileName, "", 0, "", "", ""));

         svoFile = new SVOFile(cameraID, svoFileName, initParameters, runtimeParameters);
         svoFiles.put(videoFilename, svoFile);
      }

      int cameraID = svoFile.cameraID;

      imageWidth = sl_get_width(cameraID);
      imageHeight = sl_get_height(cameraID);
      int numberOfFrames = sl_get_svo_number_of_frames(cameraID);
      fps = sl_get_camera_fps(cameraID);

      if (leftColorImageSlMatPointer == null)
         leftColorImageSlMatPointer = sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_CPU); // JavaFX WritableImage is CPU
      if (rightColorImageSlMatPointer == null)
         rightColorImageSlMatPointer = sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_CPU); // JavaFX WritableImage is CPU

      int frameNumber = (int) timestampScrubber.getCurrentVideoFrameNumber();

      if (frameNumber < 0 || frameNumber >= numberOfFrames - 1) // The timestamp scrubber can ask for out of bounds frames
      {
         return;
      }

      if (lastGrabbedFrameNumber == frameNumber) // We already grabbed this frame
      {
         return;
      }

      // Prevent doing this unless necessary, It's very expensive.
      boolean haventGrabbedAFrameYet = lastGrabbedFrameNumber < 0;
      int framesForward = frameNumber - lastGrabbedFrameNumber;
      boolean requestedFrameIsOneOfTheNext10 = framesForward > 0 && framesForward <= 10;
      if (haventGrabbedAFrameYet || !requestedFrameIsOneOfTheNext10)
      {
         sl_set_svo_position(cameraID, frameNumber);
         framesForward = 1;
      }

      int errorCode = SL_ERROR_CODE_SUCCESS;
      for (int i = 0; i < framesForward; i++) // If the requested frame is 10 or less, grab until we get there -- it's faster
      {
         errorCode = sl_grab(cameraID, svoFile.runtimeParameters);
      }

      if (errorCode != SL_ERROR_CODE_SUCCESS)
      {
         LogTools.warn(1, ZEDTools.errorMessage(errorCode));
         return;
      }

      printOnError(sl_retrieve_image(cameraID, leftColorImageSlMatPointer, SL_VIEW_LEFT, SL_MEM_CPU, imageWidth, imageHeight, null));
      printOnError(sl_retrieve_image(cameraID, rightColorImageSlMatPointer, SL_VIEW_RIGHT, SL_MEM_CPU, imageWidth, imageHeight, null));

      currentTimestamp = sl_get_current_timestamp(cameraID);
      lastGrabbedFrameNumber = frameNumber;
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor)
   {
      int totalFrames = timestampScrubber.getRobotTimestampsArray().length;

      timestampScrubber.getVideoTimestampFromRobotTimestamp(startTimestamp);
      int cropStartIndex = timestampScrubber.getCurrentIndex();
      timestampScrubber.getVideoTimestampFromRobotTimestamp(endTimestamp);
      int cropEndIndex = timestampScrubber.getCurrentIndex();
      if (cropEndIndex >= totalFrames)
         cropEndIndex = totalFrames - 1;

      int startOfDatToCopy = -1;
      int endOfDatToCopy = -1;

      List<SVOToCrop> svosToCrop = new ArrayList<>();
      String[] videoFileNames = timestampScrubber.getVideoFileNames();
      for (int i = 0; i < videoFileNames.length; i++)
      {
         int videoFileStartIndex = (int) timestampScrubber.getVideoFileStartIndices()[i];
         int videoFileEndIndex = totalFrames - 1;
         if (videoFileNames.length > i + 1)
            videoFileEndIndex = (int) timestampScrubber.getVideoFileStartIndices()[i + 1] - 1;
         videoFileEndIndex--; // svos want an earlier end for cropping

         boolean isEntirelyBeforeCropZone = videoFileEndIndex <= cropStartIndex;
         boolean isEntirelyAfterCropZone = videoFileStartIndex >= cropEndIndex;
         boolean skip = isEntirelyBeforeCropZone || isEntirelyAfterCropZone;

         if (!skip)
         {
            boolean copyOnly = true;
            if (videoFileStartIndex < cropStartIndex)
            {
               videoFileStartIndex = cropStartIndex;
               copyOnly = false;
            }
            if (videoFileEndIndex > cropEndIndex)
            {
               videoFileEndIndex = cropEndIndex;
               copyOnly = false;
            }

            int length = videoFileEndIndex - videoFileStartIndex;

            if (length > 5) // Make sure there's at least 5 frames or so
            {
               if (startOfDatToCopy == -1)
                  startOfDatToCopy = videoFileStartIndex;
               if (videoFileEndIndex > endOfDatToCopy)
                  endOfDatToCopy = videoFileEndIndex;

               svosToCrop.add(new SVOToCrop("%s%s%s_%s.svo2".formatted(perceptionDirectory, File.separator, videoFileNames[i], sensorName),
                                            copyOnly,
                                            timestampScrubber.calculateVideoFrameNumber(videoFileStartIndex, i),
                                            timestampScrubber.calculateVideoFrameNumber(videoFileEndIndex, i)));
            }
         }
      }

      Path outputDirectory = outputFile.toPath().getParent().resolve("perception");
      FileTools.ensureDirectoryExists(outputDirectory, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      for (int i = 0; i < svosToCrop.size(); i++)
      {
         SVOToCrop svoToCrop = svosToCrop.get(i);
         Path sourceFile = Paths.get(svoToCrop.svoFileName);
         Path destinationFile = outputDirectory.resolve(sourceFile.getFileName());

         if (svoToCrop.copyOnly)
         {
            try
            {
               String message = "Moving SVO file from " + sourceFile.toAbsolutePath() + " to " + destinationFile.toAbsolutePath();
               monitor.info(message);
               LogTools.info(message);
               Files.copy(sourceFile, destinationFile, StandardCopyOption.REPLACE_EXISTING);
            }
            catch (IOException e)
            {
               LogTools.error("Failed to move SVO file: " + e.getMessage());
            }
         }
         else
         {
            try
            {
               String message = "Cropping SVO file from " + sourceFile.toAbsolutePath() + " to " + destinationFile.toAbsolutePath();
               monitor.info(message);
               LogTools.info(message);
               String[] command = new String[] {"/usr/local/zed/tools/ZED_SVO_Editor",
                                                "-cut", svoToCrop.svoFileName,
                                                "-s", svoToCrop.startFrame + "",
                                                "-e", svoToCrop.endFrame + "",
                                                destinationFile.toAbsolutePath().toString()};
               LogTools.info(String.join(" ", command));
               ProcessBuilder processBuilder = new ProcessBuilder(command);
               processBuilder.redirectErrorStream(true);
               Process process = processBuilder.start();

               try (var reader = new BufferedReader(new InputStreamReader(process.getInputStream())))
               {
                  String line;
                  while ((line = reader.readLine()) != null)
                  {
                     LogTools.info(line);
                  }
               }

               int exitCode = process.waitFor();
               if (exitCode != 0)
               {
                  LogTools.error("Command exited with non-zero code: " + exitCode);
               }
            }
            catch (IOException | InterruptedException e)
            {
               LogTools.error("Error executing system command: " + e.getMessage());
            }
         }

         monitor.progress((i + 1) / (float) svosToCrop.size());
      }

      Path sourceDatFile = Paths.get(perceptionDirectory).resolve(timestampFile.getName());
      Path outputDatFile = outputDirectory.resolve(timestampFile.getName());

      LogTools.info("Writing lines %d-%d to %s".formatted(startOfDatToCopy, endOfDatToCopy, outputDatFile.toAbsolutePath().toString()));

      try (BufferedReader reader = Files.newBufferedReader(sourceDatFile); PrintWriter writer = new PrintWriter(Files.newBufferedWriter(outputDatFile)))
      {
         int lineNumber = 0;
         String line;
         while ((line = reader.readLine()) != null)
         {
            lineNumber++;
            if (lineNumber >= startOfDatToCopy && lineNumber <= endOfDatToCopy)
            {
               writer.println(line);
            }
         }
      }
      catch (IOException e)
      {
         LogTools.error("Failed to write to output .dat file: " + e.getMessage());
      }
   }

   public void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());

      if (leftColorImageSlMatPointer != null && !leftColorImageSlMatPointer.isNull())
      {
         sl_mat_free(leftColorImageSlMatPointer, SL_MEM_CPU);
         leftColorImageSlMatPointer.close();
      }
      if (rightColorImageSlMatPointer != null && !rightColorImageSlMatPointer.isNull())
      {
         sl_mat_free(rightColorImageSlMatPointer, SL_MEM_CPU);
         rightColorImageSlMatPointer.close();
      }

      for (SVOFile svoFile : svoFiles.values())
      {
         if (sl_is_opened(svoFile.cameraID))
            sl_close_camera(svoFile.cameraID);

         svoFile.initParameters.close();
         svoFile.runtimeParameters.close();
      }

      System.out.println("Closed " + getClass().getSimpleName());
   }

   public int getCameraID()
   {
      String videoFilename = timestampScrubber.getCurrentVideoFilename();
      SVOFile svoFile = svoFiles.get(videoFilename);
      if (svoFile != null)
         return svoFile.cameraID;
      else
         return -1;
   }

   public long getCurrentTimestamp()
   {
      return currentTimestamp;
   }

   public Pointer getLeftColorImageSlMatPointer()
   {
      return leftColorImageSlMatPointer;
   }

   public Pointer getRightColorImageSlMatPointer()
   {
      return rightColorImageSlMatPointer;
   }

   public TimestampScrubber getTimestampScrubber()
   {
      return timestampScrubber;
   }

   public String getName()
   {
      return sensorName;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public float getFps()
   {
      return fps;
   }

   private void printOnError(int errorCode)
   {
      if (errorCode != SL_ERROR_CODE_SUCCESS)
         LogTools.error(1, ZEDTools.errorMessage(errorCode));
   }
}

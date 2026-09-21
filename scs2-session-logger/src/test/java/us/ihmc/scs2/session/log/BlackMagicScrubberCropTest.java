package us.ihmc.scs2.session.log;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import logger_msgs.Camera;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.Java2DFrameConverter;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import us.ihmc.robotDataLogger.logger.FFmpegMuxer;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.util.List;

/**
 * Crops a BlackMagic style log (Timestamps.dat's second column holds video frame numbers, {@code hasTimebase=true}) and
 * checks the cropped log can be read back frame by frame.
 */
public class BlackMagicScrubberCropTest
{
   private static final String VIDEO_NAME = "Camera_Video.mov";
   private static final String TIMESTAMP_NAME = "Camera_Timestamps.dat";
   private static final int NUMBER_OF_BITS = 8; // Bits of the frame number drawn on each frame, enough for the 200 frames below
   private static final int BLOCK_SIZE = 8; // Width in pixels of the block drawn for each bit
   private static final int WIDTH = NUMBER_OF_BITS * BLOCK_SIZE;
   private static final int HEIGHT = 48; // Arbitrary, the blocks are as tall as the image
   private static final int FRAME_RATE = 30; // Same frame rate as the real BlackMagic log this was reproduced with
   private static final long MICROS_PER_FRAME = 1_000_000L / FRAME_RATE; // 1,000,000 microseconds in a second
   private static final int NUMBER_OF_FRAMES = 200; // Length of the source video, must fit in NUMBER_OF_BITS bits (max 255)
   private static final long ROBOT_START_TIMESTAMP = 6_957_273_445_750L; // Arbitrary, taken from the first line of the real log
   private static final long NANOS_PER_FRAME = 1_000_000_000L / FRAME_RATE; // Robot timestamps are in nanoseconds

   @TempDir
   File tempDirectory;

   @Test
   void croppedLogCanBeReadBackFrameByFrame() throws IOException
   {
      File sourceLog = new File(tempDirectory, "source");
      File croppedLog = new File(tempDirectory, "cropped");
      assertTrue(sourceLog.mkdirs() && croppedLog.mkdirs());
      writeSourceLog(sourceLog);

      Camera camera = newCamera();
      // true is hasTimeBase: what video.hasTimebase is set to in every BlackMagic log, which makes the second column a frame number
      BlackMagicScrubber source = new BlackMagicScrubber(camera, sourceLog, true);

      // Crop from the middle of the source video, so the crop start is not a keyframe or the first frame
      int firstCroppedFrame = 50;
      int lastCroppedFrame = 150;
      // +1 because the last frame is included in the crop
      int numberOfCroppedFrames = lastCroppedFrame - firstCroppedFrame + 1;
      source.cropVideo(new File(croppedLog, VIDEO_NAME),
                       new File(croppedLog, TIMESTAMP_NAME),
                       robotTimestamp(firstCroppedFrame),
                       robotTimestamp(lastCroppedFrame),
                       null);

      // The cropped log keeps the video.hasTimebase=true of the source log, so the frame numbers have to restart from 0.
      List<String> croppedTimestamps = Files.readAllLines(new File(croppedLog, TIMESTAMP_NAME).toPath());
      // 2 header lines (timebase numerator and denominator) before the timestamps
      assertEquals(2 + numberOfCroppedFrames, croppedTimestamps.size(), "Header plus one line per cropped frame");

      for (int i = 0; i < numberOfCroppedFrames; i++)
      {
         // Skip the 2 header lines, then column 0 is the robot timestamp and column 1 the video frame number
         String[] line = croppedTimestamps.get(2 + i).split(" ");
         assertEquals(robotTimestamp(firstCroppedFrame + i), Long.parseLong(line[0]));
         assertEquals(i, Long.parseLong(line[1]), "Frame number of cropped frame " + i);
      }

      BlackMagicScrubber cropped = new BlackMagicScrubber(camera, croppedLog, true);

      for (int i = 0; i < numberOfCroppedFrames; i++)
      {
         Frame frame = cropped.readVideoFrame(robotTimestamp(firstCroppedFrame + i));
         assertNotNull(frame, "No frame for cropped frame " + i);
         assertNotNull(frame.image, "Frame without an image for cropped frame " + i);

         // Half a frame is the largest error that still identifies the right frame, the next frame is a whole MICROS_PER_FRAME away
         long ptsError = Math.abs(frame.timestamp - i * MICROS_PER_FRAME);
         assertTrue(ptsError < MICROS_PER_FRAME / 2, "Cropped frame " + i + " was read at " + frame.timestamp + " us, expected " + i * MICROS_PER_FRAME);

         assertEquals(firstCroppedFrame + i, decodeFrameNumber(frame), "Cropped frame " + i + " shows the wrong source frame");
      }
   }

   private static void writeSourceLog(File directory) throws IOException
   {
      FFmpegMuxer muxer = new FFmpegMuxer(new File(directory, VIDEO_NAME), WIDTH, HEIGHT, FRAME_RATE);
      muxer.start();

      try (Java2DFrameConverter converter = new Java2DFrameConverter(); PrintWriter timestampWriter = new PrintWriter(new File(directory, TIMESTAMP_NAME)))
      {
         timestampWriter.println(1); // Timebase numerator, the real logs have 1 here
         timestampWriter.println(FRAME_RATE); // Timebase denominator, the real logs have the frame rate here

         for (int frameNumber = 0; frameNumber < NUMBER_OF_FRAMES; frameNumber++)
         {
            BufferedImage image = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
            Graphics2D graphics = image.createGraphics();
            // The frame number is drawn as black and white blocks, one per bit, which survives being lossily re-encoded.
            for (int bit = 0; bit < NUMBER_OF_BITS; bit++)
            {
               graphics.setColor(((frameNumber >> bit) & 1) == 1 ? Color.WHITE : Color.BLACK);
               graphics.fillRect(bit * BLOCK_SIZE, 0, BLOCK_SIZE, HEIGHT);
            }
            graphics.dispose();

            // Frame N is shown at N / FRAME_RATE seconds, converted to microseconds
            muxer.recordFrame(converter.convert(image), Math.round(1.0e6 * frameNumber / FRAME_RATE));
            timestampWriter.println(robotTimestamp(frameNumber) + " " + frameNumber);
         }
      }

      muxer.stopRecording();
   }

   private static long robotTimestamp(int frameNumber)
   {
      return ROBOT_START_TIMESTAMP + frameNumber * NANOS_PER_FRAME;
   }

   private static int decodeFrameNumber(Frame frame)
   {
      ByteBuffer buffer = (ByteBuffer) frame.image[0];
      int y = frame.imageHeight / 2; // Any row works, the blocks span the whole height
      int frameNumber = 0;

      for (int bit = 0; bit < NUMBER_OF_BITS; bit++)
      {
         int x = bit * BLOCK_SIZE + BLOCK_SIZE / 2; // Middle of the block, away from the edges that the lossy encoding blurs
         int value = buffer.get(y * frame.imageStride + x * frame.imageChannels) & 0xFF;

         // 128 is halfway between black (0) and white (255)
         if (value > 128)
            frameNumber |= 1 << bit;
      }

      return frameNumber;
   }

   private static Camera newCamera()
   {
      Camera camera = new Camera();
      camera.setName("Camera");
      camera.setInterlaced(false);
      camera.setVideoFile(VIDEO_NAME);
      camera.setTimestampFile(TIMESTAMP_NAME);
      return camera;
   }
}

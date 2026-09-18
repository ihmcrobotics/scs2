package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.scene.image.WritableImage;
import logger_msgs.Camera;
import org.bytedeco.javacv.Frame;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.scs2.session.log.MagewellScrubber;
import us.ihmc.scs2.session.log.ProgressConsumer;

import java.io.File;
import java.io.IOException;

public class MagewellVideoDataReader implements VideoDataReader
{
   /** Cap on consecutive non-video packets to skip per seek (audio/timecode interleaved with video). */
   private static final int MAX_NON_VIDEO_FRAMES_TO_SKIP = 256;

   private final MagewellScrubber magewellScrubber;
   /**
    * 3-slot lock-free rotation (see {@link ConcurrentCopier}): the slot currently being written on
    * the background reader thread is never the slot currently being read/displayed on the FX thread,
    * so each slot's {@link FrameData#frame} can safely be reused (its pixels overwritten in place)
    * across frames instead of allocating a new {@link WritableImage} every call.
    */
   private final ConcurrentCopier<FrameData> imageBuffer = new ConcurrentCopier<>(FrameData::new);

   public MagewellVideoDataReader(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      magewellScrubber = new MagewellScrubber(camera, dataDirectory, hasTimeBase);
   }

   public int getImageHeight()
   {
      return magewellScrubber.getMagewellDemuxer().getImageHeight();
   }

   public int getImageWidth()
   {
      return magewellScrubber.getMagewellDemuxer().getImageWidth();
   }

   public void readVideoFrame(long queryRobotTimestamp)
   {
      Frame nextFrame = magewellScrubber.readVideoFrame(queryRobotTimestamp);

      // The underlying FFmpegFrameGrabber.grabFrame() returns the next packet from any stream,
      // so a multi-stream MP4 (video + audio + timecode) may yield non-image frames here.
      int skipped = 0;
      while (nextFrame != null && !FrameImageConverter.hasImageData(nextFrame) && skipped < MAX_NON_VIDEO_FRAMES_TO_SKIP)
      {
         nextFrame = magewellScrubber.getMagewellDemuxer().getNextFrame();
         skipped++;
      }

      FrameData copyForWriting = imageBuffer.getCopyForWriting();
      copyForWriting.queryRobotTimestamp = queryRobotTimestamp;
      copyForWriting.currentRobotTimestamp = magewellScrubber.getCurrentRobotTimestamp();
      copyForWriting.currentVideoTimestamp = magewellScrubber.getCurrentVideoTimestamp();
      copyForWriting.currentDemuxerTimestamp = magewellScrubber.getMagewellDemuxer().getCurrentPTS();
      copyForWriting.frame = convertFrameToWritableImage(nextFrame, copyForWriting.frame);

      imageBuffer.commit();
   }

   /**
    * This class converts a {@link Frame} to a {@link WritableImage} in order to be displayed correctly in JavaFX.
    * <p>
    * Always allocates a new {@link WritableImage}; prefer {@link #convertFrameToWritableImage(Frame, WritableImage)}
    * on a hot path so a correctly-sized image can be reused instead.
    *
    * @param frameToConvert is the next frame we want to visualize so we convert it to be compatible with JavaFX
    * @return {@link WritableImage}
    */
   public static WritableImage convertFrameToWritableImage(Frame frameToConvert)
   {
      return convertFrameToWritableImage(frameToConvert, null);
   }

   /**
    * Same as {@link #convertFrameToWritableImage(Frame)}, but reuses {@code imageToPack} instead of
    * allocating a new {@link WritableImage} when its dimensions already match the frame - allocating
    * one is a substantial fraction of the per-frame conversion cost. See {@link FrameImageConverter}.
    *
    * @param frameToConvert is the next frame we want to visualize so we convert it to be compatible with JavaFX
    * @param imageToPack    image to write into if its size already matches; a new one is allocated otherwise (or if
    *                       {@code null})
    * @return {@code imageToPack} if its size matched, a new {@link WritableImage} otherwise
    */
   public static WritableImage convertFrameToWritableImage(Frame frameToConvert, WritableImage imageToPack)
   {
      return FrameImageConverter.convertFrameToWritableImage(frameToConvert, imageToPack);
   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer progressConsumer) throws IOException
   {
      magewellScrubber.cropVideo(outputFile, timestampFile, startTimestamp, endTimestamp, progressConsumer);
   }

   public String getName()
   {
      return magewellScrubber.getName();
   }

   public Camera getCamera()
   {
      return magewellScrubber.getCamera();
   }

   public FrameData pollCurrentFrame()
   {
      return imageBuffer.getCopyForReading();
   }

   public int getCurrentIndex()
   {
      return magewellScrubber.getCurrentIndex();
   }

   public boolean replacedRobotTimestampsContainsIndex(int index)
   {
      return magewellScrubber.replacedRobotTimestampsContainsIndex(index);
   }

   @Override
   public boolean supportsFrameDelayAdjustment()
   {
      return true;
   }

   @Override
   public int getFrameDelay()
   {
      return magewellScrubber.getFrameDelay();
   }

   @Override
   public void setFrameDelay(int frames)
   {
      magewellScrubber.setFrameDelay(frames);
   }
}

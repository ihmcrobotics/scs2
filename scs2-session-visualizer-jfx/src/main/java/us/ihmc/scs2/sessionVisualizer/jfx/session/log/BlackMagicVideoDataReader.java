package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import logger_msgs.msg.dds.Camera;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.scs2.session.log.BlackMagicScrubber;
import us.ihmc.scs2.session.log.ProgressConsumer;

import java.io.File;
import java.io.IOException;

public class BlackMagicVideoDataReader implements VideoDataReader
{
   private final BlackMagicScrubber blackMagicScrubber;
   private final JavaFXPictureConverter converter = new JavaFXPictureConverter();
   private final ConcurrentCopier<FrameData> imageBuffer = new ConcurrentCopier<>(FrameData::new);

   public BlackMagicVideoDataReader(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      blackMagicScrubber = new BlackMagicScrubber(camera, dataDirectory, hasTimeBase);
   }

   @Override
   public int getImageHeight()
   {
      return 0;
   }

   @Override
   public int getImageWidth()
   {
      return 0;
   }

   public void readVideoFrame(long queryRobotTimestamp)
   {
      try
      {
         YUVPicture nextFrame = blackMagicScrubber.readVideoFrame(queryRobotTimestamp);

         FrameData copyForWriting = imageBuffer.getCopyForWriting();
         copyForWriting.queryRobotTimestamp = queryRobotTimestamp;
         copyForWriting.currentRobotTimestamp = blackMagicScrubber.getCurrentRobotTimestamp();
         copyForWriting.currentVideoTimestamp = blackMagicScrubber.getVideoTimestamp();
         copyForWriting.currentDemuxerTimestamp = blackMagicScrubber.getDemuxer().getCurrentPTS();
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
      blackMagicScrubber.cropVideo(outputFile, timestampFile, startTimestamp, endTimestamp, monitor);
   }

   public String getName()
   {
      return blackMagicScrubber.getName();
   }

   public Camera getCamera()
   {
      return blackMagicScrubber.getCamera();
   }

   public FrameData pollCurrentFrame()
   {
      return imageBuffer.getCopyForReading();
   }

   public int getCurrentIndex()
   {
      return blackMagicScrubber.getCurrentIndex();
   }

   public boolean replacedRobotTimestampsContainsIndex(int index)
   {
      return blackMagicScrubber.replacedRobotTimestampsContainsIndex(index);
   }
}

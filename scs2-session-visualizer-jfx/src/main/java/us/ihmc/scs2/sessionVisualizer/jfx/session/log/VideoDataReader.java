package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import logger_msgs.msg.dds.Camera;
import us.ihmc.scs2.session.log.ProgressConsumer;

import java.io.File;
import java.io.IOException;

/**
 * This interface allows supporting different types of capture methods to be viewed back with SCS2.
 */
public interface VideoDataReader
{

   int getImageHeight();

   int getImageWidth();

   void readVideoFrame(long timestamp);

   void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressConsumer monitor) throws IOException;

   String getName();

   Camera getCamera();

   default FrameData pollCurrentFrame()
   {
      return null;
   };

   int getCurrentIndex();

   boolean replacedRobotTimestampsContainsIndex(int index);
}

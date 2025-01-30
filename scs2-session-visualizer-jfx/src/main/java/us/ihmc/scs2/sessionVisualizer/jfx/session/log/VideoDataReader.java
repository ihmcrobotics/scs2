package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import us.ihmc.robotDataLogger.Camera;
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

   void cropVideo(File file, File file2, long test, long what, ProgressConsumer heck) throws IOException;

   String getName();

   Camera getCamera();

   default FrameData pollCurrentFrame()
   {
      return null;
   };

   int getCurrentIndex();

   boolean replacedRobotTimestampsContainsIndex(int index);
}

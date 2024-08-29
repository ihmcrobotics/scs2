package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.scene.image.WritableImage;

/**
 * This class is used for debugging timestamp delays in the videos when viewing them in SCS2. In order to see the timestamp debugging information change the
 * boolean in {@link VideoViewer#LOGGER_VIDEO_DEBUG} to true. This can also be set as an environmental variable.
 */
public class FrameData
{
   public WritableImage frame;
   public long queryRobotTimestamp;
   public long currentRobotTimestamp;
   public long currentVideoTimestamp;
   public long currentDemuxerTimestamp;
   public long frameNumber;
}

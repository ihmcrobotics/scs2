package us.ihmc.scs2.session.log.heightMap;

import us.ihmc.scs2.session.log.perception.PerceptionMcapScrubber;
import us.ihmc.scs2.session.mcap.specs.records.Message;

/**
 * Thin typed view of one height-map channel within a shared {@link PerceptionMcapScrubber}-indexed
 * {@code perception.mcap}: decodes the messages {@link PerceptionMcapScrubber} hands back for
 * {@link #HEIGHT_SCAN_TOPIC_NAME} into {@link HeightMapData}, and caches the last decode so repeated queries at the
 * same log time (typical during playback - many timestamp updates land on the same nearest message) don't redecode.
 * Deliberately does not open or index the file itself - construct one shared {@link PerceptionMcapScrubber} per
 * log session and hand it to this (and, in the future, e.g. a {@code VoxelMapMcapScrubber}) so multiple grid/map
 * sources in the same file don't each parse it independently.
 * <p>
 * {@link #HEIGHT_SCAN_TOPIC_NAME} is a literal copy of {@code PerceptionMcapLogger.HEIGHT_SCAN_TOPIC}'s name (in
 * ihmc-robot-data-logger, which this module cannot depend on) - if that topic is ever renamed, this literal must be
 * updated to match.
 */
public class HeightMapMcapScrubber
{
   public static final String HEIGHT_SCAN_TOPIC_NAME = "/stepping_camera/realsense/height_scan/height_scan_message";

   private final PerceptionMcapScrubber perceptionMcapScrubber;
   private final String topicName;

   private long lastScrubbedLogTime = Long.MIN_VALUE;
   private HeightMapData lastDecoded;

   /** Reads the {@link #HEIGHT_SCAN_TOPIC_NAME} channel. Use {@link #HeightMapMcapScrubber(PerceptionMcapScrubber, String)} for a different topic. */
   public HeightMapMcapScrubber(PerceptionMcapScrubber perceptionMcapScrubber)
   {
      this(perceptionMcapScrubber, HEIGHT_SCAN_TOPIC_NAME);
   }

   public HeightMapMcapScrubber(PerceptionMcapScrubber perceptionMcapScrubber, String topicName)
   {
      this.perceptionMcapScrubber = perceptionMcapScrubber;
      this.topicName = topicName;
   }

   /** @return whether the underlying {@code perception.mcap} declares a channel for this scrubber's topic. */
   public boolean isAvailable()
   {
      return perceptionMcapScrubber.hasChannel(topicName);
   }

   public int getMessageCount()
   {
      return perceptionMcapScrubber.getMessageCount(topicName);
   }

   /**
    * Decodes and returns the height map message with the log time nearest to the given timestamp, or {@code null}
    * if the underlying file has no channel for this topic, or that channel has no messages.
    */
   public HeightMapData scrub(long timestamp)
   {
      Message message = perceptionMcapScrubber.scrub(topicName, timestamp);
      if (message == null)
         return null;

      if (message.logTime() == lastScrubbedLogTime && lastDecoded != null)
         return lastDecoded;

      lastDecoded = HeightMapMessageDecoder.decode(message);
      lastScrubbedLogTime = message.logTime();
      return lastDecoded;
   }
}

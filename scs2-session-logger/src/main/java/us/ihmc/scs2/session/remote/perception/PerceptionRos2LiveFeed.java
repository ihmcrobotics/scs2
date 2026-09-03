package us.ihmc.scs2.session.remote.perception;

import perception_msgs.HeightScanMessage;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2SubscriptionCallbackSampler;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.robotDataLogger.handshake.LoggingROS2API;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.log.heightMap.HeightMapData;
import us.ihmc.scs2.session.log.heightMap.HeightMapMessageDecoder;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * The one class callers construct to get live perception data. Owns a single {@link ROS2Node} for the live viewer,
 * shared by every perception source built on top of it via {@link #subscribe}. Each source's state and behavior
 * lives directly on this class as prefixed fields/methods (e.g. the height-map members below), rather than a
 * separate class per source - adding a new source means adding its prefixed members here, not a new class to
 * construct and thread through every call site that wants perception data. Mirrors {@code PerceptionMcapLogger}'s
 * multi-channel design (one node, N independent subscriptions) on the write side.
 * <p>
 * Constructed once a session is fully connected/started, closed when that session ends.
 * <p>
 * == Height map ==
 * {@link #startHeightMap(Consumer)} drives a consumer (in practice {@code YoHeightGridFX3D::setData}) so it stays
 * in sync with the session's buffer position - including scrubbing backward through recently-received history
 * while a live session is paused, not just "whatever arrived most recently".
 * <p>
 * This relies on the controller (see {@code HeightScanTerm} in ihmc-closed-source-control) publishing its own
 * {@code lastHeightScanTimestamp} YoLong alongside {@code HeightScanMessage} - a real registered variable, so it
 * rides the session's existing {@code YoSharedBuffer} tick-for-tick for free (sample-and-hold across ticks with no
 * new height-scan message, exact restore on scrub) with no client-side buffering of our own to get wrong. If that
 * variable isn't present (older controller build, or a robot config without {@code HeightScanTerm}), the live
 * height map is simply unavailable - see {@link #isHeightMapAvailable()} - mirroring how
 * {@code HeightMapMcapScrubber}/{@code PerceptionMcapScrubber} report absence on the log-file side. Callers should
 * check {@link #isHeightMapAvailable()} before deciding whether to build a graphic at all, then call
 * {@link #startHeightMap(Consumer)}.
 * <p>
 * The one thing kept locally is a small bounded history of decoded {@link HeightMapData}, keyed by the exact
 * {@code controllerTimestamp} of each received message - looked up by the (possibly time-traveled) value of
 * {@code lastHeightScanTimestamp} on every buffer-position change. This is an exact lookup, not a nearest-match
 * search, since that YoLong always holds the precise timestamp that was authoritative at that tick.
 * <p>
 * RL control mode loads every available policy up front (see {@code RLController}), so the live registry tree
 * typically has several sibling {@code HeightScanTerm} registries - one per model - all sharing the same leaf name.
 * Only the currently-active policy's {@code lastHeightScanTimestamp} actually advances; the rest sit frozen at
 * whatever value they had when last active (or their initial 0). Rather than hardcoding which model is "the" active
 * one - which would break the moment the operator switches policies mid-session - every matching candidate is
 * tracked and, each tick, whichever holds the largest value is used. Since {@code controllerTimestamp} comes from a
 * single shared monotonic clock, the max across candidates is always the one that's currently (or most recently)
 * advancing, with no need to know the active model's name.
 */
public class PerceptionRos2LiveFeed implements Closeable
{
   private static final String HEIGHT_SCAN_TERM_REGISTRY_NAME = "HeightScanTerm";
   private static final String LAST_HEIGHT_SCAN_TIMESTAMP_VARIABLE_NAME = "lastHeightScanTimestamp";
   /** Bounded history of recently decoded height-map messages, keyed by controllerTimestamp - oldest entries are evicted. */
   private static final int HEIGHT_MAP_HISTORY_CAPACITY = 100;

   private final ROS2Node ros2Node;
   private final Session session;

   /** One entry per RL model that has a {@code HeightScanTerm} - see class javadoc for why this isn't a single match. */
   private final List<YoLong> lastHeightScanTimestampCandidates;
   private final Map<Long, HeightMapData> heightMapHistory = new LinkedHashMap<>(HEIGHT_MAP_HISTORY_CAPACITY, 0.75f, false)
   {
      @Override
      protected boolean removeEldestEntry(Map.Entry<Long, HeightMapData> eldest)
      {
         return size() > HEIGHT_MAP_HISTORY_CAPACITY;
      }
   };

   public PerceptionRos2LiveFeed(Session session)
   {
      this.session = session;
      ros2Node = new ROS2Node("scs2_perception_live_feed_node");

      List<YoVariable> matches = session.getRootRegistry().findVariables(HEIGHT_SCAN_TERM_REGISTRY_NAME, LAST_HEIGHT_SCAN_TIMESTAMP_VARIABLE_NAME);
      lastHeightScanTimestampCandidates = new ArrayList<>();
      for (YoVariable variable : matches)
      {
         if (variable instanceof YoLong)
            lastHeightScanTimestampCandidates.add((YoLong) variable);
      }
   }

   /** @return whether any controller-side {@code HeightScanTerm.lastHeightScanTimestamp} was found - if not, there is nothing to show live. */
   public boolean isHeightMapAvailable()
   {
      return !lastHeightScanTimestampCandidates.isEmpty();
   }

   /** Subscribes and starts feeding {@code dataConsumer} on every buffer-position change. Only call if {@link #isHeightMapAvailable()}. */
   public void startHeightMap(Consumer<HeightMapData> dataConsumer)
   {
      subscribe(LoggingROS2API.STEPPING_HEIGHT_SCAN, this::onHeightScanMessage);

      session.addCurrentBufferPropertiesListener(bufferProperties ->
      {
         long timestamp = Long.MIN_VALUE;
         for (int i = 0; i < lastHeightScanTimestampCandidates.size(); i++)
            timestamp = Math.max(timestamp, lastHeightScanTimestampCandidates.get(i).getValue());

         HeightMapData data;
         synchronized (heightMapHistory)
         {
            data = heightMapHistory.get(timestamp);
         }
         if (data != null)
            dataConsumer.accept(data);
      });
   }

   private void onHeightScanMessage(HeightScanMessage message)
   {
      HeightMapData data = HeightMapMessageDecoder.decode(message);
      synchronized (heightMapHistory)
      {
         heightMapHistory.put(data.getControllerTimestamp(), data);
      }
   }

   public <T extends ROS2Message<T>> void subscribe(ROS2Topic<T> topic, ROS2SubscriptionCallbackSampler<T> callback)
   {
      ros2Node.createSubscriptionSampler(topic, callback);
   }

   @Override
   public void close()
   {
      ros2Node.close();
   }
}

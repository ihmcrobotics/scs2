package us.ihmc.scs2.session.remote.perception;

import perception_msgs.HeightScanMessage;
import us.ihmc.robotDataLogger.handshake.LoggingROS2API;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.log.heightMap.HeightMapData;
import us.ihmc.scs2.session.log.heightMap.HeightMapMessageDecoder;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Live view of the height-map channel on a shared {@link PerceptionRos2LiveFeed}, driving a consumer (in practice
 * {@code YoHeightGridFX3D::setData}) so it stays in sync with the session's buffer position - including scrubbing
 * backward through recently-received history while a live session is paused, not just "whatever arrived most
 * recently".
 * <p>
 * This relies on the controller (see {@code HeightScanTerm} in ihmc-closed-source-control) publishing its own
 * {@code lastHeightScanTimestamp} YoLong alongside {@code HeightScanMessage} - a real registered variable, so it
 * rides the session's existing {@code YoSharedBuffer} tick-for-tick for free (sample-and-hold across ticks with no
 * new height-scan message, exact restore on scrub) with no client-side buffering of our own to get wrong. If that
 * variable isn't present (older controller build, or a robot config without {@code HeightScanTerm}), the live
 * height map is simply unavailable - see {@link #isAvailable()} - mirroring how {@code HeightMapMcapScrubber}/
 * {@code PerceptionMcapScrubber} report absence on the log-file side. Callers should check {@link #isAvailable()}
 * before deciding whether to build a graphic at all, then call {@link #start(Consumer)}.
 * <p>
 * The one thing this class does keep locally is a small bounded history of decoded {@link HeightMapData}, keyed by
 * the exact {@code controllerTimestamp} of each received message - looked up by the (possibly time-traveled) value
 * of {@code lastHeightScanTimestamp} on every buffer-position change. This is an exact lookup, not a nearest-match
 * search, since that YoLong always holds the precise timestamp that was authoritative at that tick.
 * <p>
 * RL control mode loads every available policy up front (see {@code RLController}), so the live registry tree
 * typically has several sibling {@code HeightScanTerm} registries - one per model - all sharing the same leaf name.
 * Only the currently-active policy's {@code lastHeightScanTimestamp} actually advances; the rest sit frozen at
 * whatever value they had when last active (or their initial 0). Rather than hardcoding which model is "the" active
 * one - which would break the moment the operator switches policies mid-session - this class tracks every matching
 * candidate and, each tick, uses whichever holds the largest value. Since {@code controllerTimestamp} comes from a
 * single shared monotonic clock, the max across candidates is always the one that's currently (or most recently)
 * advancing, with no need to know the active model's name.
 */
public class HeightMapRos2LiveFeed
{
   private static final String HEIGHT_SCAN_TERM_REGISTRY_NAME = "HeightScanTerm";
   private static final String LAST_HEIGHT_SCAN_TIMESTAMP_VARIABLE_NAME = "lastHeightScanTimestamp";

   /** Bounded history of recently decoded messages, keyed by controllerTimestamp - oldest entries are evicted. */
   private static final int HISTORY_CAPACITY = 100;

   private final PerceptionRos2LiveFeed perceptionLiveFeed;
   private final Session session;
   /** One entry per RL model that has a {@code HeightScanTerm} - see class javadoc for why this isn't a single match. */
   private final List<YoLong> lastHeightScanTimestampCandidates;

   private final Map<Long, HeightMapData> history = new LinkedHashMap<>(HISTORY_CAPACITY, 0.75f, false)
   {
      @Override
      protected boolean removeEldestEntry(Map.Entry<Long, HeightMapData> eldest)
      {
         return size() > HISTORY_CAPACITY;
      }
   };

   public HeightMapRos2LiveFeed(PerceptionRos2LiveFeed perceptionLiveFeed, Session session)
   {
      this.perceptionLiveFeed = perceptionLiveFeed;
      this.session = session;

      List<YoVariable> matches = session.getRootRegistry().findVariables(HEIGHT_SCAN_TERM_REGISTRY_NAME, LAST_HEIGHT_SCAN_TIMESTAMP_VARIABLE_NAME);
      lastHeightScanTimestampCandidates = new ArrayList<>();
      for (YoVariable variable : matches)
      {
         if (variable instanceof YoLong)
            lastHeightScanTimestampCandidates.add((YoLong) variable);
      }
   }

   /** @return whether any controller-side {@code HeightScanTerm.lastHeightScanTimestamp} was found - if not, there is nothing to show live. */
   public boolean isAvailable()
   {
      return !lastHeightScanTimestampCandidates.isEmpty();
   }

   /** Subscribes and starts feeding {@code dataConsumer} on every buffer-position change. Only call if {@link #isAvailable()}. */
   public void start(Consumer<HeightMapData> dataConsumer)
   {
      perceptionLiveFeed.subscribe(LoggingROS2API.STEPPING_HEIGHT_SCAN, this::onMessage);

      session.addCurrentBufferPropertiesListener(bufferProperties ->
      {
         long timestamp = Long.MIN_VALUE;
         for (int i = 0; i < lastHeightScanTimestampCandidates.size(); i++)
            timestamp = Math.max(timestamp, lastHeightScanTimestampCandidates.get(i).getValue());

         HeightMapData data;
         synchronized (history)
         {
            data = history.get(timestamp);
         }
         if (data != null)
            dataConsumer.accept(data);
      });
   }

   private void onMessage(HeightScanMessage message)
   {
      HeightMapData data = HeightMapMessageDecoder.decode(message);
      synchronized (history)
      {
         history.put(data.getControllerTimestamp(), data);
      }
   }
}

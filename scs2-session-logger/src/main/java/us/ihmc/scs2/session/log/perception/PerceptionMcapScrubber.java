package us.ihmc.scs2.session.log.perception;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Channel;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;

import java.io.Closeable;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Reads {@code perception.mcap} - a sibling file next to a classic (non-MCAP) log session directory, written
 * independently by {@code PerceptionMcapLogger} - and exposes a nearest-timestamp-match lookup per ROS2 topic,
 * mirroring {@code ZEDSVOScrubber}'s role for the ZED SVO2 sibling files.
 * <p>
 * Unlike its predecessor {@code HeightScanMcapScrubber} (which assumed exactly one channel in the file), this
 * class indexes every channel the file declares, keyed by topic name - {@code perception.mcap} can hold any number
 * of grid/map sources (multiple height maps, a future voxel map, etc.) multiplexed into one file, each on its own
 * channel, independently starting late/stopping early/resuming with gaps (see {@code PerceptionMcapLogger}). One
 * instance should be shared by every per-source scrubber reading this file (e.g. {@code HeightMapMcapScrubber})
 * rather than each opening/indexing the file separately.
 * <p>
 * Message decoding is deliberately NOT done here: the CDR field layout is different per message type, so decoding
 * is left to a type-specific decoder (e.g. {@code HeightMapMessageDecoder}) that a caller applies to the
 * {@link Message} this class hands back from {@link #scrub}. This class only knows how to find "the message on
 * topic X nearest time T" - construction indexes each message's log time and its position (top-level chunk record
 * index + index within that chunk) per channel; it does not decode payloads or hold decompressed chunk data beyond
 * what the last {@link #scrub} call needed.
 */
public class PerceptionMcapScrubber implements Closeable
{
   private static final String PERCEPTION_MCAP_FILENAME = "perception.mcap";

   private record MessageRef(long logTime, int chunkRecordIndex, int indexWithinChunk)
   {
   }

   private final FileInputStream mcapFileInputStream;
   private final MCAP mcap;
   private final Map<String, Integer> channelIdByTopic = new HashMap<>();
   private final Map<Integer, List<MessageRef>> messageRefsByChannelId = new HashMap<>();

   private final Map<Integer, Long> lastScrubbedLogTimeByChannelId = new HashMap<>();
   private final Map<Integer, Message> lastMessageByChannelId = new HashMap<>();

   /**
    * @return the sibling {@code perception.mcap} file next to the given log session directory, or {@code null} if
    *       it does not exist (older logs, or logs recorded without any perception data).
    */
   public static File findMcapFile(File sessionDirectory)
   {
      File file = new File(sessionDirectory, PERCEPTION_MCAP_FILENAME);
      return file.isFile() ? file : null;
   }

   public PerceptionMcapScrubber(File perceptionMcapFile) throws IOException
   {
      mcapFileInputStream = new FileInputStream(perceptionMcapFile);
      mcap = new MCAP(mcapFileInputStream.getChannel());

      List<Record> topLevelRecords = mcap.records();

      for (Record record : topLevelRecords)
      {
         if (record.op() == Opcode.CHANNEL)
         {
            Channel channel = record.body();
            channelIdByTopic.put(channel.topic(), channel.id());
         }
      }

      if (channelIdByTopic.isEmpty())
         LogTools.warn("No channels found in " + perceptionMcapFile + ", perception data will not be available.");

      for (int chunkRecordIndex = 0; chunkRecordIndex < topLevelRecords.size(); chunkRecordIndex++)
      {
         Record chunkRecord = topLevelRecords.get(chunkRecordIndex);
         if (chunkRecord.op() != Opcode.CHUNK)
            continue;

         Chunk chunk = chunkRecord.body();
         List<Record> innerRecords = chunk.records();

         for (int indexWithinChunk = 0; indexWithinChunk < innerRecords.size(); indexWithinChunk++)
         {
            Record innerRecord = innerRecords.get(indexWithinChunk);
            if (innerRecord.op() != Opcode.MESSAGE)
               continue;

            Message message = innerRecord.body();
            messageRefsByChannelId.computeIfAbsent(message.channelId(), k -> new ArrayList<>())
                                  .add(new MessageRef(message.logTime(), chunkRecordIndex, indexWithinChunk));
         }
      }

      for (List<MessageRef> refs : messageRefsByChannelId.values())
         refs.sort((a, b) -> Long.compare(a.logTime(), b.logTime()));
   }

   /** @return whether this file declares a channel for the given ROS2 topic name. */
   public boolean hasChannel(String topicName)
   {
      return channelIdByTopic.containsKey(topicName);
   }

   /** @return the number of messages logged on {@code topicName}'s channel, or {@code 0} if this file has no such channel. */
   public int getMessageCount(String topicName)
   {
      List<MessageRef> refs = messageRefsByChannelId.get(channelIdByTopic.get(topicName));
      return refs == null ? 0 : refs.size();
   }

   /**
    * Finds the message on {@code topicName}'s channel with the log time nearest to the given timestamp - still CDR
    * encoded, decode with the message type's own decoder (e.g. {@code HeightMapMessageDecoder}) - or {@code null}
    * if this file has no channel for that topic, or that channel has no messages.
    */
   public Message scrub(String topicName, long timestamp)
   {
      Integer channelId = channelIdByTopic.get(topicName);
      if (channelId == null)
         return null;

      List<MessageRef> refs = messageRefsByChannelId.get(channelId);
      if (refs == null || refs.isEmpty())
         return null;

      MessageRef nearest = refs.get(findNearestIndex(refs, timestamp));

      Long lastLogTime = lastScrubbedLogTimeByChannelId.get(channelId);
      if (lastLogTime != null && lastLogTime == nearest.logTime())
         return lastMessageByChannelId.get(channelId);

      Chunk chunk = mcap.records().get(nearest.chunkRecordIndex()).body();
      Message message = chunk.records().get(nearest.indexWithinChunk()).body();

      lastMessageByChannelId.put(channelId, message);
      lastScrubbedLogTimeByChannelId.put(channelId, nearest.logTime());
      return message;
   }

   private static int findNearestIndex(List<MessageRef> refs, long timestamp)
   {
      int low = 0;
      int high = refs.size() - 1;

      while (low < high)
      {
         int mid = (low + high) >>> 1;
         if (refs.get(mid).logTime() < timestamp)
            low = mid + 1;
         else
            high = mid;
      }

      if (low > 0 && Math.abs(refs.get(low - 1).logTime() - timestamp) <= Math.abs(refs.get(low).logTime() - timestamp))
         return low - 1;
      return low;
   }

   @Override
   public void close() throws IOException
   {
      mcap.close();
      mcapFileInputStream.close();
   }
}

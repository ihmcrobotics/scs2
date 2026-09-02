package us.ihmc.scs2.session.log.heightScan;

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
import java.util.List;

/**
 * Reads {@code heightScan.mcap} - a sibling file next to a classic (non-MCAP) log session directory, written
 * independently by {@code HeightScanMcapLogger} - and exposes it as a nearest-timestamp-match lookup, mirroring
 * {@code ZEDSVOScrubber}'s role for the ZED SVO2 sibling files.
 * <p>
 * The file contains exactly one schema/channel ({@code perception_msgs/HeightScanMessage}), chunked/indexed MCAP
 * 1.0 written by {@code McapWriter}. Message decoding is deferred to {@link #scrub(long)}: construction only
 * indexes each message's log time and its position (top-level chunk record index + index within that chunk), it
 * does not decode payloads or hold decompressed chunk data beyond what the last {@link #scrub(long)} needed.
 */
public class HeightScanMcapScrubber implements Closeable
{
   private static final String HEIGHT_SCAN_MCAP_FILENAME = "heightScan.mcap";

   private record MessageRef(long logTime, int chunkRecordIndex, int indexWithinChunk)
   {
   }

   private final FileInputStream mcapFileInputStream;
   private final MCAP mcap;
   private final int channelId;
   private final List<MessageRef> messageRefs = new ArrayList<>();

   private long lastScrubbedLogTime = Long.MIN_VALUE;
   private HeightScanData lastDecoded;

   /**
    * @return the sibling {@code heightScan.mcap} file next to the given log session directory, or {@code null} if
    *       it does not exist (older logs, or logs recorded without height scan data).
    */
   public static File findHeightScanMcapFile(File sessionDirectory)
   {
      File file = new File(sessionDirectory, HEIGHT_SCAN_MCAP_FILENAME);
      return file.isFile() ? file : null;
   }

   public HeightScanMcapScrubber(File heightScanMcapFile) throws IOException
   {
      mcapFileInputStream = new FileInputStream(heightScanMcapFile);
      mcap = new MCAP(mcapFileInputStream.getChannel());

      int foundChannelId = -1;
      List<Record> topLevelRecords = mcap.records();

      for (Record record : topLevelRecords)
      {
         if (record.op() == Opcode.CHANNEL)
         {
            Channel channel = record.body();
            foundChannelId = channel.id();
            break;
         }
      }

      if (foundChannelId < 0)
      {
         LogTools.warn("No channel found in " + heightScanMcapFile + ", height scan data will not be available.");
      }
      channelId = foundChannelId;

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
            if (message.channelId() != channelId)
               continue;

            messageRefs.add(new MessageRef(message.logTime(), chunkRecordIndex, indexWithinChunk));
         }
      }

      messageRefs.sort((a, b) -> Long.compare(a.logTime(), b.logTime()));
   }

   public int getMessageCount()
   {
      return messageRefs.size();
   }

   /**
    * Decodes and returns the height scan message with the log time nearest to the given timestamp, or {@code null}
    * if this file has no height scan messages.
    */
   public HeightScanData scrub(long timestamp)
   {
      if (messageRefs.isEmpty())
         return null;

      MessageRef nearest = messageRefs.get(findNearestIndex(timestamp));

      if (nearest.logTime() == lastScrubbedLogTime && lastDecoded != null)
         return lastDecoded;

      Chunk chunk = mcap.records().get(nearest.chunkRecordIndex()).body();
      Message message = chunk.records().get(nearest.indexWithinChunk()).body();

      lastDecoded = HeightScanMessageDecoder.decode(message);
      lastScrubbedLogTime = nearest.logTime();
      return lastDecoded;
   }

   private int findNearestIndex(long timestamp)
   {
      int low = 0;
      int high = messageRefs.size() - 1;

      while (low < high)
      {
         int mid = (low + high) >>> 1;
         if (messageRefs.get(mid).logTime() < timestamp)
            low = mid + 1;
         else
            high = mid;
      }

      if (low > 0 && Math.abs(messageRefs.get(low - 1).logTime() - timestamp) <= Math.abs(messageRefs.get(low).logTime() - timestamp))
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

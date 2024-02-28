package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

public class MCAPSummaryBuilder
{
   // Creating groups in the following order. There will be right after DATA_END
   private final Map<Integer, Record> schemas = new LinkedHashMap<>(); // Schemas in a map to avoid duplicates
   private final Map<Integer, Record> channels = new LinkedHashMap<>(); // Channels in a map to avoid duplicates
   private final List<Record> chunkIndices = new ArrayList<>();
   private final List<Record> attachmentIndices = new ArrayList<>();
   private final List<Record> metadataIndices = new ArrayList<>();
   private final Map<Opcode, List<Record>> indexMap = Map.of(Opcode.CHUNK_INDEX,
                                                             chunkIndices,
                                                             Opcode.ATTACHMENT_INDEX,
                                                             attachmentIndices,
                                                             Opcode.METADATA_INDEX,
                                                             metadataIndices);
   private final MutableStatistics statistics = new MutableStatistics();

   public MCAPSummaryBuilder()
   {
   }

   public void update(Record record)
   {
      boolean isNewRecord = switch (record.op())
      {
         case SCHEMA:
         {
            Schema schema = record.body();
            yield schemas.put(schema.id(), record) == null;
         }
         case CHANNEL:
         {
            Channel channel = record.body();
            yield channels.put(channel.id(), record) == null;
         }
         case CHUNK_INDEX:
         case ATTACHMENT_INDEX:
         case METADATA_INDEX:
         {
            indexMap.get(record.op()).add(record);
            yield false;
         }
         case ATTACHMENT:
         case METADATA:
         {
            yield true;
         }
         case CHUNK:
         {
            Chunk chunk = record.body();
            statistics.updateMessageTimes(chunk.messageStartTime(), chunk.messageEndTime());
            for (Record chunkRecords : chunk.records())
            {
               update(chunkRecords);
            }
            yield true;
         }
         case MESSAGE:
         {
            Message message = record.body();
            statistics.incrementChannelMessageCount(message.channelId());
            yield true;
         }
         default:
         {
            // Do nothing
            yield false;
         }
      };

      if (isNewRecord)
      {
         statistics.incrementCount(record.op());
      }
   }

   public void writeSummary(MCAPDataOutput dataOutput)
   {
      finalizeMCAP(dataOutput, schemas.values(), channels.values(), chunkIndices, attachmentIndices, metadataIndices, new MutableRecord(statistics));
   }

   /**
    * Finalizes the MCAP by writing the data-end, the summary section, the summary offset section, the footer, and the magic bytes.
    *
    * @param dataOutput           The output to write the MCAP to.
    * @param allSchemas           The collection of all the schemas that were found in the data section to be written in the summary section. Optional.
    * @param allChannels          The collection of all the channels that were found in the data section to be written in the summary section. Optional.
    * @param allChunkIndices      The collection of all the chunk indices that were found in the data section to be written in the summary section. Optional.
    * @param allAttachmentIndices The collection of all the attachment indices that were found in the data section to be written in the summary section.
    *                             Optional.
    * @param allMetadataIndices   The collection of all the metadata indices that were found in the data section to be written in the summary section. Optional.
    * @param statistics           The statistics record to be written in the footer.
    */
   public static void finalizeMCAP(MCAPDataOutput dataOutput,
                                   Collection<? extends Record> allSchemas,
                                   Collection<? extends Record> allChannels,
                                   Collection<? extends Record> allChunkIndices,
                                   Collection<? extends Record> allAttachmentIndices,
                                   Collection<? extends Record> allMetadataIndices,
                                   Record statistics)
   {
      List<Record> schemaList = toList(allSchemas);
      List<Record> channelList = toList(allChannels);
      List<Record> chunkIndexList = toList(allChunkIndices);
      List<Record> attachmentIndexList = toList(allAttachmentIndices);
      List<Record> metadataIndexList = toList(allMetadataIndices);

      // Now we can write the groups
      long summarySectionOffset = dataOutput.position();
      long schemaOffset = dataOutput.position();
      schemaList.forEach(r -> r.write(dataOutput));
      long channelOffset = dataOutput.position();
      channelList.forEach(r -> r.write(dataOutput));
      long chunkIndexOffset = dataOutput.position();
      chunkIndexList.forEach(r -> r.write(dataOutput));
      long attachmentIndexOffset = dataOutput.position();
      attachmentIndexList.forEach(r -> r.write(dataOutput));
      long metadataIndexOffset = dataOutput.position();
      metadataIndexList.forEach(r -> r.write(dataOutput));
      long statisticsOffset = dataOutput.position();
      statistics.write(dataOutput);

      List<Record> summaryOffsetSectionRecords = new ArrayList<>();
      writeSummaryOffset(dataOutput, schemaOffset, schemaList, summaryOffsetSectionRecords);
      writeSummaryOffset(dataOutput, channelOffset, channelList, summaryOffsetSectionRecords);
      writeSummaryOffset(dataOutput, chunkIndexOffset, chunkIndexList, summaryOffsetSectionRecords);
      writeSummaryOffset(dataOutput, attachmentIndexOffset, attachmentIndexList, summaryOffsetSectionRecords);
      writeSummaryOffset(dataOutput, metadataIndexOffset, metadataIndexList, summaryOffsetSectionRecords);
      writeSummaryOffset(dataOutput, statisticsOffset, Collections.singletonList(statistics), summaryOffsetSectionRecords);

      List<Record> summarySectionRecords = combine(schemaList, channelList, chunkIndexList, attachmentIndexList, metadataIndexList, List.of(statistics));
      MutableRecord footer = new MutableRecord(new Footer(summarySectionOffset, summarySectionRecords, summaryOffsetSectionRecords));
      footer.write(dataOutput);
   }

   private static <T> List<T> combine(List<T>... lists)
   {
      if (lists == null || lists.length == 0)
         return Collections.emptyList();

      List<T> combinedList = new ArrayList<>(Stream.of(lists).mapToInt(List::size).sum());
      for (List<T> list : lists)
      {
         combinedList.addAll(list);
      }
      return combinedList;
   }

   private static <T> List<T> toList(Collection<? extends T> collection)
   {
      if (collection instanceof List)
         return (List<T>) collection;
      else
         return collection == null ? Collections.emptyList() : new ArrayList<>(collection);
   }

   private static void writeSummaryOffset(MCAPDataOutput dataOutput, long groupOffset, List<Record> groupList, List<Record> summaryOffsetSectionRecordsToPack)
   {
      if (!groupList.isEmpty())
      {
         MutableRecord summaryOffset = new MutableRecord(new SummaryOffset(groupOffset, groupList));
         summaryOffsetSectionRecordsToPack.add(summaryOffset);
         summaryOffset.write(dataOutput);
      }
   }
}

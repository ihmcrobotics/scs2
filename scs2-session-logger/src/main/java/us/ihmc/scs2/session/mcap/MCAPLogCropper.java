package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Attachment;
import us.ihmc.scs2.session.mcap.specs.records.Channel;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Footer;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.scs2.session.mcap.specs.records.MutableStatistics;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Schema;
import us.ihmc.scs2.session.mcap.specs.records.SummaryOffset;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class MCAPLogCropper
{
   private final MCAP mcap;
   private long startTimestamp;
   private long endTimestamp;
   private OutputFormat outputFormat;

   public enum OutputFormat
   {
      MCAP
   }

   public MCAPLogCropper(MCAP mcap)
   {
      this.mcap = mcap;
   }

   public void setStartTimestamp(long startTimestamp)
   {
      this.startTimestamp = startTimestamp;
   }

   public void setEndTimestamp(long endTimestamp)
   {
      this.endTimestamp = endTimestamp;
   }

   public void setOutputFormat(OutputFormat outputFormat)
   {
      this.outputFormat = outputFormat;
   }

   public void crop(FileOutputStream outputStream) throws IOException
   {
      MCAPDataOutput dataOutput = MCAPDataOutput.wrap(outputStream.getChannel());
      dataOutput.putBytes(Magic.MAGIC_BYTES); // header magic

      // Used to store the records from one chunk to the next.
      // Some chunks may not have any message left after cropping, so we'll move the schemas and channels to the next chunk when that happens.
      List<Record> recordsForNextChunk = null;

      // Creating groups in the following order. There will be right after DATA_END
      Map<Integer, Record> schemas = new LinkedHashMap<>(); // Schemas in a map to avoid duplicates
      Map<String, Record> channels = new LinkedHashMap<>(); // Channels in a map to avoid duplicates
      List<Record> chunkIndices = new ArrayList<>();
      List<Record> attachmentIndices = new ArrayList<>();
      List<Record> metadataIndices = new ArrayList<>();
      MutableStatistics statistics = new MutableStatistics();

      for (Record record : mcap.records())
      {
         switch (record.op())
         {
            case CHUNK_INDEX:
            case MESSAGE_INDEX:
            case ATTACHMENT_INDEX:
            case METADATA_INDEX:
            case FOOTER:
            {
               // We re-build the indices and footer from scratch down there.
               continue;
            }
            case HEADER:
            {
               record.write(dataOutput);
               break;
            }
            case DATA_END: // TODO The CRC32 should probably be recalculated
            {
               record.write(dataOutput);
               // Now we can write the groups
               long summarySectionOffset = dataOutput.position();
               long schemaOffset = dataOutput.position();
               List<Record> schemaList = new ArrayList<>(schemas.values());
               schemaList.forEach(r -> r.write(dataOutput));
               long channelOffset = dataOutput.position();
               List<Record> channelList = new ArrayList<>(channels.values());
               channelList.forEach(r -> r.write(dataOutput));
               long chunkIndexOffset = dataOutput.position();
               chunkIndices.forEach(r -> r.write(dataOutput));
               long attachmentIndexOffset = dataOutput.position();
               attachmentIndices.forEach(r -> r.write(dataOutput));
               long metadataIndexOffset = dataOutput.position();
               metadataIndices.forEach(r -> r.write(dataOutput));
               long statisticsOffset = dataOutput.position();
               MutableRecord statisticsRecord = new MutableRecord(statistics);
               statisticsRecord.write(dataOutput);

               List<Record> summarySectionRecords = new ArrayList<>();
               summarySectionRecords.addAll(schemaList);
               summarySectionRecords.addAll(channelList);
               summarySectionRecords.addAll(chunkIndices);
               summarySectionRecords.addAll(attachmentIndices);
               summarySectionRecords.addAll(metadataIndices);
               summarySectionRecords.add(statisticsRecord);

               List<Record> summaryOffsetSectionRecords = new ArrayList<>();
               if (!schemas.isEmpty())
               {
                  MutableRecord summaryOffset = new MutableRecord(new SummaryOffset(schemaOffset, schemaList));
                  summaryOffsetSectionRecords.add(summaryOffset);
                  summaryOffset.write(dataOutput);
               }
               if (!channels.isEmpty())
               {
                  MutableRecord summaryOffset = new MutableRecord(new SummaryOffset(channelOffset, channelList));
                  summaryOffsetSectionRecords.add(summaryOffset);
                  summaryOffset.write(dataOutput);
               }
               if (!chunkIndices.isEmpty())
               {
                  MutableRecord summaryOffset = new MutableRecord(new SummaryOffset(chunkIndexOffset, chunkIndices));
                  summaryOffsetSectionRecords.add(summaryOffset);
                  summaryOffset.write(dataOutput);
               }
               if (!attachmentIndices.isEmpty())
               {
                  MutableRecord summaryOffset = new MutableRecord(new SummaryOffset(attachmentIndexOffset, attachmentIndices));
                  summaryOffsetSectionRecords.add(summaryOffset);
                  summaryOffset.write(dataOutput);
               }
               if (!metadataIndices.isEmpty())
               {
                  MutableRecord summaryOffset = new MutableRecord(new SummaryOffset(metadataIndexOffset, metadataIndices));
                  summaryOffsetSectionRecords.add(summaryOffset);
                  summaryOffset.write(dataOutput);
               }
               {
                  MutableRecord summaryOffset = new MutableRecord(new SummaryOffset(statisticsOffset, Collections.singletonList(statisticsRecord)));
                  summaryOffsetSectionRecords.add(summaryOffset);
                  summaryOffset.write(dataOutput);
               }

               MutableRecord footer = new MutableRecord(new Footer(summarySectionOffset, summarySectionRecords, summaryOffsetSectionRecords));
               footer.write(dataOutput);
               break;
            }
            case SCHEMA:
            {
               Schema schema = record.body();
               if (schemas.put(schema.id(), record) == null)
                  statistics.incrementCount(Opcode.SCHEMA);
               break;
            }
            case CHANNEL:
            {
               Channel channel = record.body();
               if (channels.put(channel.topic(), record) == null)
                  statistics.incrementCount(Opcode.CHANNEL);
               break;
            }
            case CHUNK:
            {
               Chunk chunk = record.body();
               if (chunk.messageStartTime() > endTimestamp)
               {
                  continue;
               }

               // We need to possibly crop the chunk and likely re-build the various indices.
               long chunkOffset = dataOutput.position();

               Chunk croppedChunk = chunk.crop(startTimestamp, endTimestamp);
               if (croppedChunk == null)
                  continue;

               MutableRecord croppedChunkRecord = new MutableRecord(croppedChunk);

               if (croppedChunk.records().stream().noneMatch(r -> r.op() == Opcode.MESSAGE))
               {
                  if (recordsForNextChunk != null)
                     recordsForNextChunk.addAll(croppedChunk.records());
                  else
                     recordsForNextChunk = new ArrayList<>(croppedChunk.records());
                  continue; // We don't want to write a chunk with no message. The schemas and channels will be moved to the next chunk.
               }

               if (recordsForNextChunk != null)
               {
                  croppedChunk.records().addAll(0, recordsForNextChunk);
                  recordsForNextChunk = null;
               }

               for (Record insideCroppedChunkRecord : croppedChunk.records())
               {
                  switch (insideCroppedChunkRecord.op())
                  {
                     case MESSAGE:
                     {
                        statistics.incrementCount(insideCroppedChunkRecord.op());
                        statistics.incrementChannelMessageCount(((Message) insideCroppedChunkRecord.body()).channelId());
                        break;
                     }
                     case SCHEMA:
                     {
                        Schema schema = insideCroppedChunkRecord.body();
                        if (schemas.put(schema.id(), insideCroppedChunkRecord) == null)
                           statistics.incrementCount(Opcode.SCHEMA);
                        break;
                     }
                     case CHANNEL:
                     {
                        Channel channel = insideCroppedChunkRecord.body();
                        if (channels.put(channel.topic(), insideCroppedChunkRecord) == null)
                           statistics.incrementCount(Opcode.CHANNEL);
                        break;
                     }
                  }
               }

               List<MutableRecord> croppedMessageIndexRecords = croppedChunk.records().generateMessageIndexList().stream().map(MutableRecord::new).toList();
               chunkIndices.add(croppedChunkRecord.generateChunkIndexRecord(chunkOffset, croppedMessageIndexRecords));
               // Update statistics
               statistics.incrementChunkCount();
               statistics.updateMessageTimes(croppedChunk.messageStartTime(), croppedChunk.messageEndTime());
               croppedChunkRecord.write(dataOutput, true);
               croppedMessageIndexRecords.forEach(r -> r.write(dataOutput, true));
               break;
            }
            case ATTACHMENT:
            {
               Attachment attachment = record.body();
               if (attachment.logTime() >= startTimestamp && attachment.logTime() <= endTimestamp)
               {
                  long attachmentOffset = dataOutput.position();
                  record.write(dataOutput, true);
                  attachmentIndices.add(record.generateAttachmentIndexRecord(attachmentOffset));
                  statistics.incrementCount(record.op());
               }
               break;
            }
            case METADATA:
            {
               long metadataOffset = dataOutput.position();
               record.write(dataOutput, true);
               metadataIndices.add(record.generateMetadataIndexRecord(metadataOffset));
               statistics.incrementCount(record.op());
            }
            break;
         }
      }
      dataOutput.putBytes(Magic.MAGIC_BYTES); // footer magic
      dataOutput.close();
   }
}

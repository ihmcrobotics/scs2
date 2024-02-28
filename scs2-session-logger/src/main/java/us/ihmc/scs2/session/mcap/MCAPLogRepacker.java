package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.log.ProgressConsumer;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.DataEnd;
import us.ihmc.scs2.session.mcap.specs.records.MCAPSummaryBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.MutableChunk;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Records;

import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * This class is used to repack MCAP files. Essentially rebuilds the MCAP, this can help fix bad indices or timestamps.
 */
public class MCAPLogRepacker
{
   public MCAPLogRepacker()
   {

   }

   public void repack(MCAP mcap, FileOutputStream outputStream, ProgressConsumer progressConsumer)
   {
      repack(mcap, 0, Long.MAX_VALUE, outputStream, progressConsumer);
   }

   public void repack(MCAP mcap, long chunkMinDuration, long chunkMaxDuration, FileOutputStream outputStream, ProgressConsumer progressConsumer)
   {
      MCAPDataOutput dataOutput = MCAPDataOutput.wrap(outputStream.getChannel());
      dataOutput.putBytes(Magic.MAGIC_BYTES); // header magic

      List<Record> recordsForNextChunk = null;
      MCAPSummaryBuilder summaryBuilder = new MCAPSummaryBuilder();

      for (int i = 0; i < mcap.records().size(); i++)
      {
         Record record = mcap.records().get(i);

         if (progressConsumer != null)
            progressConsumer.progress((double) i / (mcap.records().size() - 1));

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
               new MutableRecord(new DataEnd(0)).write(dataOutput);
               summaryBuilder.writeSummary(dataOutput);
               break;
            }
            case SCHEMA, CHANNEL:
            {
               summaryBuilder.update(record);
               break;
            }
            case CHUNK:
            {
               Chunk chunk = record.body();
               // We're going to rebuild the chunk from its records.

               Records records = new Records(chunk.records());
               if (recordsForNextChunk != null)
               {
                  records.addAll(recordsForNextChunk);
                  recordsForNextChunk = null;
               }
               records.sortByTimestamp();

               if (!records.containsMessages())
               {
                  recordsForNextChunk = records;
                  continue; // We don't want to write a chunk with no message. The schemas and channels will be moved to the next chunk.
               }

               if (records.getMessageEndTime() - records.getMessageStartTime() < chunkMinDuration)
               {
                  recordsForNextChunk = records;
                  continue; // We don't want to write a chunk that is too short. The schemas and channels will be moved to the next chunk.
               }

               if (records.getMessageEndTime() - records.getMessageStartTime() > chunkMaxDuration)
               {
                  // We need to split the chunk
                  long splitTime = records.getMessageStartTime() + chunkMaxDuration;
                  int splitIndex = records.indexOf(r -> r.op() == Opcode.MESSAGE && ((Message) r.body()).logTime() > splitTime);
                  recordsForNextChunk = new ArrayList<>(records.subList(splitIndex, records.size()));
                  records = new Records(records.subList(0, splitIndex));
               }

               MutableChunk repackedChunk = new MutableChunk();
               repackedChunk.setRecords(records);
               repackedChunk.setCompression(chunk.compression());
               MutableRecord repackedChunkRecord = new MutableRecord(repackedChunk);

               long chunkOffset = dataOutput.position();
               List<MutableRecord> repackedMessageIndexRecords = repackedChunk.records().generateMessageIndexList().stream().map(MutableRecord::new).toList();
               summaryBuilder.update(repackedChunkRecord);
               summaryBuilder.update(repackedChunkRecord.generateChunkIndexRecord(chunkOffset, repackedMessageIndexRecords));

               repackedChunkRecord.write(dataOutput, true);
               repackedMessageIndexRecords.forEach(r -> r.write(dataOutput, true));
               break;
            }
            case ATTACHMENT:
            {
               long attachmentOffset = dataOutput.position();
               record.write(dataOutput, true);
               summaryBuilder.update(record);
               summaryBuilder.update(record.generateAttachmentIndexRecord(attachmentOffset));
               break;
            }
            case METADATA:
            {
               long metadataOffset = dataOutput.position();
               record.write(dataOutput, true);
               summaryBuilder.update(record);
               summaryBuilder.update(record.generateMetadataIndexRecord(metadataOffset));
               break;
            }
         }
      }

      dataOutput.putBytes(Magic.MAGIC_BYTES);
      dataOutput.close();
   }
}

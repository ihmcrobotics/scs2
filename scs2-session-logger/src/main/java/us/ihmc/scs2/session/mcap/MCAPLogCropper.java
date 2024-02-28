package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Attachment;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.DataEnd;
import us.ihmc.scs2.session.mcap.specs.records.MCAPSummaryBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.scs2.session.mcap.specs.records.Record;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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

      MCAPSummaryBuilder summaryBuilder = new MCAPSummaryBuilder();

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

               if (!croppedChunk.records().containsMessages())
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

               List<MutableRecord> croppedMessageIndexRecords = croppedChunk.records().generateMessageIndexList().stream().map(MutableRecord::new).toList();
               summaryBuilder.update(croppedChunkRecord);
               summaryBuilder.update(croppedChunkRecord.generateChunkIndexRecord(chunkOffset, croppedMessageIndexRecords));

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
                  summaryBuilder.update(record);
                  summaryBuilder.update(record.generateAttachmentIndexRecord(attachmentOffset));
               }
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

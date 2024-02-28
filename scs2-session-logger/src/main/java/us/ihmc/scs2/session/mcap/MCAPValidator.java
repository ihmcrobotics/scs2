package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.ChunkIndex;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;

public class MCAPValidator
{
   public static boolean validateChunkTimestamps(MCAP mcap)
   {
      for (Record record : mcap.records())
      {
         if (record.op() == Opcode.CHUNK)
         {
            Chunk chunk = record.body();
            long startTime = chunk.messageStartTime();
            long endTime = chunk.messageEndTime();
            if (startTime > endTime)
               return false;
            if (startTime != chunk.records().getMessageStartTime())
               return false;
            if (endTime != chunk.records().getMessageEndTime())
               return false;
         }
      }

      for (Record record : mcap.records())
      {
         if (record.op() == Opcode.CHUNK_INDEX)
         {
            ChunkIndex chunkIndex = record.body();
            Chunk chunk = null;
            try
            {
               chunk = chunkIndex.chunk().body();
            }
            catch (Exception e)
            {
               return false;
            }

            long startTime = chunkIndex.messageStartTime();
            long endTime = chunkIndex.messageEndTime();
            if (startTime > endTime)
               return false;
            if (startTime != chunk.messageStartTime())
               return false;
            if (endTime != chunk.messageEndTime())
               return false;
         }
      }

      return true;
   }
}

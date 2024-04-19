package us.ihmc.scs2.session.mcap;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Records;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.scs2.session.mcap.MCAPLogCropperTest.*;

public class MCAPLogRepackerTest
{
   @Test
   public void testRepack() throws IOException
   {
      File demoMCAPFile = getDemoMCAPFile();

      MCAP originalMCAP = MCAP.load(new FileInputStream(demoMCAPFile).getChannel());
      MCAPLogFileReader.exportChunkToFile(MCAPLogFileReader.SCS2_MCAP_DEBUG_HOME, originalMCAP.records().get(1).body(), null);
      MCAPLogRepacker mcapLogRepacker = new MCAPLogRepacker();
      File repackedDemoMCAPFile = createTempMCAPFile("repackedDemo");
      mcapLogRepacker.repack(originalMCAP, new FileOutputStream(repackedDemoMCAPFile), null);

      // Let's compare the original and the repacked files by loading them into memory and comparing their content
      MCAP repackedMCAP = MCAP.load(new FileInputStream(repackedDemoMCAPFile).getChannel());

      // The cropped MCAP is slightly different:
      // - The ZSTD compression gives slightly different sizes, which in turn offsets the records
      // - In the demo.mcap file, there were some empty message index records, the cropping removes them.
      assertChunksEquivalent(originalMCAP.records(), repackedMCAP.records());
      assertSchemasEqual(originalMCAP.records(), repackedMCAP.records());
      assertChannelsEqual(originalMCAP.records(), repackedMCAP.records());
      assertAttachmentsEqual(originalMCAP.records(), repackedMCAP.records());
      assertMetadatasEqual(originalMCAP.records(), repackedMCAP.records());

      validateDataEnd(originalMCAP);
      validateChunkIndices(originalMCAP);
      validateMessageIndices(originalMCAP.records());
      validateFooter(originalMCAP);

      validateDataEnd(repackedMCAP);
      validateChunkIndices(repackedMCAP);
      validateMessageIndices(repackedMCAP.records());
      validateFooter(repackedMCAP);
   }

   public static void assertChunksEquivalent(List<Record> expectedRecords, List<Record> actualRecords)
   {
      List<Chunk> expectedChunks = expectedRecords.stream().filter(r -> r.op() == Opcode.CHUNK).map(r -> (Chunk) r.body()).toList();
      List<Chunk> actualChunks = actualRecords.stream().filter(r -> r.op() == Opcode.CHUNK).map(r -> (Chunk) r.body()).toList();

      if (expectedChunks.size() != actualChunks.size())
      {
         fail("Expected " + expectedChunks.size() + " chunks, but found " + actualChunks.size());
      }

      for (int i = 0; i < expectedChunks.size(); i++)
      {
         Chunk expectedChunk = expectedChunks.get(i);
         Chunk actualChunk = actualChunks.stream().filter(c -> c.messageStartTime() == expectedChunk.messageStartTime()).findFirst().orElse(null);
         assertNotNull(actualChunk, "Could not find a chunk with message start time " + expectedChunk.messageStartTime());

         assertEquals(expectedChunk.messageStartTime(), actualChunk.messageStartTime(), "Chunk " + i + " has different start time");
         assertEquals(expectedChunk.messageEndTime(), actualChunk.messageEndTime(), "Chunk " + i + " has different end time");
         assertEquals(expectedChunk.recordsUncompressedLength(), actualChunk.recordsUncompressedLength(), "Chunk " + i + " has different uncompressed length");
         assertEquals(expectedChunk.compression(), actualChunk.compression(), "Chunk " + i + " has different compression");
         if (expectedChunk.records().equals(actualChunk.records()))
         {
            assertEquals(expectedChunk.uncompressedCRC32(), actualChunk.uncompressedCRC32(), "Chunk " + i + " has different uncompressed CRC32");
         }
         else
         {
            Records expectedChunkRecords = new Records(expectedChunk.records());
            expectedChunkRecords.sortByTimestamp();
            assertEquals(expectedChunkRecords.size(), actualChunk.records().size(), "Chunk " + i + " has different number of records");
            for (int j = 0; j < expectedChunkRecords.size(); j++)
            {
               Record expectedChunkInnerRecord = expectedChunkRecords.get(j);
               Record actualChunkInnerRecord = actualChunk.records().get(j);
               assertEquals(expectedChunkInnerRecord.op(), actualChunkInnerRecord.op(), "Chunk " + i + " has different record at index " + j);
               assertEquals((Object) expectedChunkInnerRecord.body(),
                            (Object) actualChunkInnerRecord.body(),
                            "Chunk " + i + " has different record at index " + j);
            }
            assertEquals(expectedChunkRecords.getCRC32(), actualChunk.uncompressedCRC32(), "Chunk " + i + " has different uncompressed CRC32");
         }
      }
   }
}

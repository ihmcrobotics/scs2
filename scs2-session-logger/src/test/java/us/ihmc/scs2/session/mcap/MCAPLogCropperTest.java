package us.ihmc.scs2.session.mcap;

import org.apache.commons.io.FilenameUtils;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAPLogCropper.OutputFormat;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPByteBufferDataOutput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.ChunkIndex;
import us.ihmc.scs2.session.mcap.specs.records.Footer;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.MessageIndex;
import us.ihmc.scs2.session.mcap.specs.records.MessageIndexEntry;
import us.ihmc.scs2.session.mcap.specs.records.MessageIndexOffset;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Records;
import us.ihmc.scs2.session.mcap.specs.records.Schema;
import us.ihmc.scs2.session.mcap.specs.records.SummaryOffset;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class MCAPLogCropperTest
{
   @Test
   public void testSimpleCloningMCAP() throws IOException
   {
      File demoMCAPFile = getDemoMCAPFile();
      MCAP originalMCAP = new MCAP(new FileInputStream(demoMCAPFile).getChannel());
      File clonedDemoMCAPFile = createTempMCAPFile("clonedDemo");
      MCAPDataOutput dataOutput = MCAPDataOutput.wrap(new FileOutputStream(clonedDemoMCAPFile).getChannel());
      dataOutput.putBytes(Magic.MAGIC_BYTES); // header magic
      originalMCAP.records().forEach(record -> record.write(dataOutput));
      dataOutput.putBytes(Magic.MAGIC_BYTES); // footer magic
      dataOutput.close();

      // Let's compare the original and the cloned files by loading them into memory and comparing their content
      MCAP clonedMCAP = new MCAP(new FileInputStream(clonedDemoMCAPFile).getChannel());

      if (originalMCAP.records().size() != clonedMCAP.records().size())
      {
         fail("Original and cloned MCAPs have different number of records");
      }

      for (int i = 0; i < originalMCAP.records().size(); i++)
      {
         assertEquals(originalMCAP.records().get(i), clonedMCAP.records().get(i), "Record " + i + " is different");
      }

      assertFileEquals(demoMCAPFile, clonedDemoMCAPFile);
   }

   /**
    * Compares the content of two files. This is a simple byte-to-byte comparison.
    *
    * @param expected The expected file.
    * @param actual   The actual file.
    */
   private static void assertFileEquals(File expected, File actual) throws IOException
   {
      try (FileInputStream expectedFileInputStream = new FileInputStream(expected); FileInputStream actualFileInputStream = new FileInputStream(actual))
      {
         byte[] expectedBuffer = new byte[1024];
         byte[] actualBuffer = new byte[1024];

         int expectedRead = 0;
         int actualRead = 0;

         while ((expectedRead = expectedFileInputStream.read(expectedBuffer)) != -1)
         {
            actualRead = actualFileInputStream.read(actualBuffer);
            if (actualRead == -1)
            {
               fail("Actual file is shorter than the expected file");
            }

            if (expectedRead != actualRead)
            {
               fail("Files have different lengths");
            }

            for (int i = 0; i < expectedRead; i++)
            {
               if (expectedBuffer[i] != actualBuffer[i])
               {
                  fail("Files are different");
               }
            }
         }
      }
   }

   @Test
   public void testNotActuallyCroppingMCAPDemoFile() throws IOException
   {
      File demoMCAPFile = getDemoMCAPFile();

      MCAP originalMCAP = new MCAP(new FileInputStream(demoMCAPFile).getChannel());
      MCAPLogFileReader.exportChunkToFile(MCAPLogFileReader.SCS2_MCAP_DEBUG_HOME, ((Chunk) originalMCAP.records().get(1).body()), null);
      MCAPLogCropper mcapLogCropper = new MCAPLogCropper(originalMCAP);
      mcapLogCropper.setStartTimestamp(0);
      mcapLogCropper.setEndTimestamp(Long.MAX_VALUE);
      mcapLogCropper.setOutputFormat(OutputFormat.MCAP);
      File croppedDemoMCAPFile = createTempMCAPFile("croppedDemo");
      mcapLogCropper.crop(new FileOutputStream(croppedDemoMCAPFile));

      // Let's compare the original and the cropped files by loading them into memory and comparing their content
      MCAP croppedMCAP = new MCAP(new FileInputStream(croppedDemoMCAPFile).getChannel());

      // The cropped MCAP is slightly different:
      // - The ZSTD compression gives slightly different sizes, which in turn offsets the records
      // - In the demo.mcap file, there were some empty message index records, the cropping removes them.
      assertChunksEqual(originalMCAP.records(), croppedMCAP.records());
      assertSchemasEqual(originalMCAP.records(), croppedMCAP.records());
      assertChannelsEqual(originalMCAP.records(), croppedMCAP.records());
      assertAttachmentsEqual(originalMCAP.records(), croppedMCAP.records());
      assertMetadatasEqual(originalMCAP.records(), croppedMCAP.records());

      validateChunkIndices(originalMCAP);
      validateMessageIndices(originalMCAP.records());
      validateFooter(originalMCAP);

      validateChunkIndices(croppedMCAP);
      validateMessageIndices(croppedMCAP.records());
      validateFooter(croppedMCAP);
   }

   private void assertChunksEqual(List<Record> expectedRecords, List<Record> actualRecords)
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
         assertEquals(expectedChunk.uncompressedCRC32(), actualChunk.uncompressedCRC32(), "Chunk " + i + " has different uncompressed CRC32");
         assertEquals(expectedChunk.compression(), actualChunk.compression(), "Chunk " + i + " has different compression");
         assertEquals(expectedChunk.records(), actualChunk.records(), "Chunk " + i + " has different records");
      }
   }

   private void assertSchemasEqual(List<Record> expectedRecords, List<Record> actualRecords)
   {
      List<Schema> expectedSchemas = expectedRecords.stream().filter(r -> r.op() == Opcode.SCHEMA).map(r -> (Schema) r.body()).toList();
      List<Schema> actualSchemas = actualRecords.stream().filter(r -> r.op() == Opcode.SCHEMA).map(r -> (Schema) r.body()).toList();

      if (expectedSchemas.size() != actualSchemas.size())
      {
         fail("Expected " + expectedSchemas.size() + " schemas, but found " + actualSchemas.size());
      }

      for (int i = 0; i < expectedSchemas.size(); i++)
      {
         Schema expectedSchema = expectedSchemas.get(i);
         Schema actualSchema = actualSchemas.stream().filter(s -> s.id() == expectedSchema.id()).findFirst().orElse(null);
         assertNotNull(actualSchema, "Could not find a schema with ID " + expectedSchema.id());

         assertEquals(expectedSchema, actualSchema, "Schema " + i + " is different");
      }
   }

   private void assertChannelsEqual(List<Record> expectedRecords, List<Record> actualRecords)
   {
      List<Record> expectedChannels = expectedRecords.stream().filter(r -> r.op() == Opcode.CHANNEL).toList();
      List<Record> actualChannels = actualRecords.stream().filter(r -> r.op() == Opcode.CHANNEL).toList();

      if (expectedChannels.size() != actualChannels.size())
      {
         fail("Expected " + expectedChannels.size() + " channels, but found " + actualChannels.size());
      }

      for (int i = 0; i < expectedChannels.size(); i++)
      {
         assertEquals(expectedChannels.get(i), actualChannels.get(i), "Channel " + i + " is different");
      }
   }

   private void assertAttachmentsEqual(List<Record> expectedRecords, List<Record> actualRecords)
   {
      List<Record> expectedAttachments = expectedRecords.stream().filter(r -> r.op() == Opcode.ATTACHMENT).toList();
      List<Record> actualAttachments = actualRecords.stream().filter(r -> r.op() == Opcode.ATTACHMENT).toList();

      if (expectedAttachments.size() != actualAttachments.size())
      {
         fail("Expected " + expectedAttachments.size() + " attachments, but found " + actualAttachments.size());
      }

      for (int i = 0; i < expectedAttachments.size(); i++)
      {
         assertEquals(expectedAttachments.get(i), actualAttachments.get(i), "Attachment " + i + " is different");
      }
   }

   private void assertMetadatasEqual(List<Record> expectedRecords, List<Record> actualRecords)
   {
      List<Record> expectedMetadatas = expectedRecords.stream().filter(r -> r.op() == Opcode.METADATA).toList();
      List<Record> actualMetadatas = actualRecords.stream().filter(r -> r.op() == Opcode.METADATA).toList();

      if (expectedMetadatas.size() != actualMetadatas.size())
      {
         fail("Expected " + expectedMetadatas.size() + " metadatas, but found " + actualMetadatas.size());
      }

      for (int i = 0; i < expectedMetadatas.size(); i++)
      {
         assertEquals(expectedMetadatas.get(i), actualMetadatas.get(i), "Metadata " + i + " is different");
      }
   }

   private void validateChunkIndices(MCAP mcap)
   {
      List<Record> chunkIndices = mcap.records().stream().filter(r -> r.op() == Opcode.CHUNK_INDEX).toList();
      if (chunkIndices.isEmpty())
      {
         fail("No chunk index found");
      }

      for (int i = 0; i < chunkIndices.size(); i++)
      {
         ChunkIndex chunkIndex = chunkIndices.get(i).body();
         Record chunkRecord = chunkIndex.chunk();

         if (chunkRecord == null)
            fail("Chunk index " + i + " has no chunk");
         if (chunkRecord.op() != Opcode.CHUNK)
            fail("Chunk index " + i + " has a record that is not a chunk");

         Chunk chunk = chunkRecord.body();

         assertEquals(chunkIndex.messageStartTime(), chunk.messageStartTime(), "Chunk index " + i + " has different start time");
         assertEquals(chunkIndex.messageEndTime(), chunk.messageEndTime(), "Chunk index " + i + " has different end time");
         assertEquals(chunkIndex.recordsUncompressedLength(), chunk.recordsUncompressedLength(), "Chunk index " + i + " has different uncompressed length");
         assertEquals(chunkIndex.compression(), chunk.compression(), "Chunk index " + i + " has different compression");
         assertEquals(chunkIndex.recordsCompressedLength(), chunk.recordsCompressedLength(), "Chunk index " + i + " has different compressed length");

         for (MessageIndexOffset messageIndexOffset : chunkIndex.messageIndexOffsets())
         {
            assertTrue(messageIndexOffset.offset() >= 0, "Chunk index " + i + " has a message index offset that is negative");

            int channelId = messageIndexOffset.channelId();
            long offset = messageIndexOffset.offset();

            Record messageIndexRecord = Record.load(mcap.getDataInput(), offset);
            assertEquals(Opcode.MESSAGE_INDEX, messageIndexRecord.op(), "Chunk index " + i + " has a message index record that is not a message index");

            MessageIndex messageIndex = messageIndexRecord.body();
            assertEquals(channelId, messageIndex.channelId(), "Chunk index " + i + " has a message index record with different channel ID");
         }
      }
   }

   private void validateMessageIndices(List<Record> records)
   {
      List<MessageIndex> messageIndices = records.stream().filter(r -> r.op() == Opcode.MESSAGE_INDEX).map(r -> (MessageIndex) r.body()).toList();
      if (messageIndices.isEmpty())
      {
         fail("No message index found");
      }
      List<ChunkIndex> chunkIndices = records.stream().filter(r -> r.op() == Opcode.CHUNK_INDEX).map(r -> (ChunkIndex) r.body()).toList();

      for (int i = 0; i < messageIndices.size(); i++)
      {
         MessageIndex messageIndex = messageIndices.get(i);
         // Find the chunk containing the message from the timestamp:

         for (MessageIndexEntry messageIndexEntry : messageIndex.messageIndexEntries())
         {
            long logTime = messageIndexEntry.logTime();
            ChunkIndex chunkIndex = chunkIndices.stream()
                                                .filter(c -> c.messageStartTime() <= logTime && c.messageEndTime() >= logTime)
                                                .findFirst()
                                                .orElse(null);
            assertNotNull(chunkIndex, "Could not find a chunk index for message index entry " + messageIndexEntry);

            Chunk chunk = chunkIndex.chunk().body();
            ByteBuffer recordsUncompressedBuffer = chunk.getRecordsUncompressedBuffer();
            MCAPDataInput dataInput = MCAPDataInput.wrap(recordsUncompressedBuffer);
            Record messageRecord = Record.load(dataInput, messageIndexEntry.offset());
            assertEquals(Opcode.MESSAGE, messageRecord.op(), "Message index entry " + messageIndexEntry + " does not point to a message record");
            Message message = messageRecord.body();
            assertEquals(logTime, message.logTime(), "Message index entry " + messageIndexEntry + " points to a message with different log time");
         }
      }
   }

   private void validateFooter(MCAP mcap)
   {
      List<Record> footerRecords = mcap.records().stream().filter(r -> r.op() == Opcode.FOOTER).toList();
      assertEquals(1, footerRecords.size(), "Expected one footer, but found " + footerRecords.size());

      Record footerRecord = footerRecords.get(0);
      Footer footer = footerRecord.body();
      Records summarySection = footer.summarySection();
      Records summaryOffsetSection = footer.summaryOffsetSection();

      // Computing the CRC in different ways to validate the footer
      MCAPCRC32Helper crc32 = new MCAPCRC32Helper();
      summarySection.forEach(record -> record.updateCRC(crc32));
      summaryOffsetSection.forEach(record -> record.updateCRC(crc32));
      crc32.addUnsignedByte(footerRecord.op().id());
      crc32.addLong(footerRecord.bodyLength());
      crc32.addLong(footer.summarySectionOffset());
      crc32.addLong(footer.summaryOffsetSectionOffset());
      assertEquals(crc32.getValue(), footer.summaryCRC32(), "Footer has different summary CRC32");

      MCAPByteBufferDataOutput dataOutput = new MCAPByteBufferDataOutput();
      summarySection.forEach(record -> record.write(dataOutput));
      summaryOffsetSection.forEach(record -> record.write(dataOutput));
      dataOutput.putUnsignedByte(footerRecord.op().id());
      dataOutput.putLong(footerRecord.bodyLength());
      dataOutput.putLong(footer.summarySectionOffset());
      dataOutput.putLong(footer.summaryOffsetSectionOffset());
      dataOutput.close();
      byte[] expectedSummaryCRC32Input = new byte[dataOutput.getBuffer().remaining()];
      dataOutput.getBuffer().get(expectedSummaryCRC32Input);

      assertArrayEquals(expectedSummaryCRC32Input, footer.summaryCRC32Input(), "Footer has different summary CRC32 input");
      assertEquals(footer.summarySectionLength() + footer.summaryOffsetSectionLength() + Record.RECORD_HEADER_LENGTH + 2 * Long.BYTES,
                   footer.summaryCRC32Input().length,
                   "Footer has different summary CRC32 input length");
      crc32.reset();
      crc32.addBytes(footer.summaryCRC32Input());
      assertEquals(crc32.getValue(), footer.summaryCRC32(), "Footer has different summary CRC32");

      assertTrue(summarySection.stream()
                               .allMatch(r -> r.op() == Opcode.SCHEMA || r.op() == Opcode.CHANNEL || r.op() == Opcode.CHUNK_INDEX
                                              || r.op() == Opcode.ATTACHMENT_INDEX || r.op() == Opcode.METADATA_INDEX || r.op() == Opcode.STATISTICS),
                 "Summary section contains a record that is not a schema, channel, chunk index, attachment index, metadata index, or statistics");

      assertTrue(summaryOffsetSection.stream().allMatch(r -> r.op() == Opcode.SUMMARY_OFFSET),
                 "Summary offset section contains a record that is not a summary offset");

      for (Record summaryOffsetRecord : summaryOffsetSection)
      {
         SummaryOffset summaryOffset = summaryOffsetRecord.body();
         Records group = summaryOffset.group();
         assertTrue(group.stream().allMatch(r -> r.op() == summaryOffset.groupOpcode()), "Group contains a record that is not of the expected type");
      }
   }

   private static File getDemoMCAPFile() throws IOException
   {
      File demoMCAPFile;
      // Check if the demo file is already downloaded, allowing for faster local testing
      Path localFileVersion = Paths.get(System.getProperty("user.home"), "Downloads", "demo.mcap");
      if (Files.exists(localFileVersion))
      {
         demoMCAPFile = localFileVersion.toFile();
      }
      else
      {
         URL demoMCAPURL = new URL("https://github.com/foxglove/mcap/raw/main/testdata/mcap/demo.mcap");
         demoMCAPFile = downloadFile(demoMCAPURL);
      }
      return demoMCAPFile;
   }

   private static File downloadFile(URL url) throws IOException
   {
      File file = createTempMCAPFile(FilenameUtils.getBaseName(url.getFile()));
      LogTools.info("Downloading file from " + url);
      try (InputStream in = url.openStream())
      {
         Files.copy(in, file.toPath(), StandardCopyOption.REPLACE_EXISTING);
      }
      LogTools.info("Downloaded file to " + file.getAbsolutePath());
      return file;
   }

   public static File createTempMCAPFile(String name) throws IOException
   {
      File file = File.createTempFile(name, ".mcap");
      LogTools.info("Created temporary file: " + file.getAbsolutePath());
      file.deleteOnExit();
      return file;
   }
}

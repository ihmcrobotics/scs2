package us.ihmc.scs2.session.mcap;

import gnu.trove.map.hash.TIntObjectHashMap;
import io.kaitai.struct.ByteBufferKaitaiStream;
import javafx.scene.Scene;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.SessionIOTools;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.*;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;
import java.util.stream.Collectors;

public class MCAPLogFileReader
{
   private static final Path SCS2_MCAP_DEBUG_HOME = SessionIOTools.SCS2_HOME.resolve("mcap-debug");

   static
   {
      try
      {
         FileTools.ensureDirectoryExists(SCS2_MCAP_DEBUG_HOME);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private final YoRegistry propertiesRegistry = new YoRegistry("MCAPProperties");

   private final File mcapFile;
   private final MCAPDebugPrinter printer;
   private final YoRegistry mcapRegistry;
   private final Mcap mcap;
   private final TIntObjectHashMap<ROS2MessageSchema> schemas = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<YoROS2Message> yoMessageMap = new TIntObjectHashMap<>();

   private final List<Mcap.Chunk> allChunks = new ArrayList<>();
   private final YoInteger chunkIndex = new YoInteger("MCAPChunkIndex", propertiesRegistry);
   private final YoLong currentChunkStartTimestamp = new YoLong("MCAPCurrentChunkStartTimestamp", propertiesRegistry);
   private final YoLong currentChunkEndTimestamp = new YoLong("MCAPCurrentChunkEndTimestamp", propertiesRegistry);

   private final YoLong currentTimestamp = new YoLong("MCAPCurrentTimestamp", propertiesRegistry);
   private final PriorityQueue<Mcap.Message> currentChunkMessages = new PriorityQueue<>((m1, m2) -> Long.compare(m1.logTime(), m2.logTime()));

   public MCAPLogFileReader(File mcapFile, MCAPDebugPrinter printer, YoRegistry mcapRegistry) throws IOException
   {
      this.mcapFile = mcapFile;
      this.printer = printer;
      this.mcapRegistry = mcapRegistry;

      mcapRegistry.addChild(propertiesRegistry);
      mcap = Mcap.fromFile(mcapFile.getAbsolutePath());
      allChunks.addAll(mcap.records().stream().filter(r -> r.op() == Mcap.Opcode.CHUNK).map(r -> (Mcap.Chunk) r.body()).collect(Collectors.toList()));
   }

   public long getInitialTimestamp()
   {
      return allChunks.get(0).messageStartTime();
   }

   public void loadSchemas() throws IOException
   {
      for (Mcap.Record record : mcap.records())
      {
         if (record.op() != Mcap.Opcode.SCHEMA)
            continue;

         Mcap.Schema schema = (Mcap.Schema) record.body();

         try
         {
            schemas.put(schema.id(), ROS2MessageSchema.loadSchema(schema));
         }
         catch (Exception e)
         {
            File debugFile = SCS2_MCAP_DEBUG_HOME.resolve("schema-%s-%s.txt".formatted(schema.name().str(), e.getClass().getSimpleName())).toFile();
            if (debugFile.exists())
               debugFile.delete();
            debugFile.createNewFile();
            FileOutputStream os = new FileOutputStream(debugFile);
            os.write(schema.data());
            os.close();
            LogTools.info("Failed to load schema: " + schema.name().str() + ", saved to: " + debugFile.getAbsolutePath());
            throw e;
         }
      }
   }

   public void loadChannels() throws IOException
   {
      for (Mcap.Record record : mcap.records())
      {
         if (record.op() != Mcap.Opcode.CHANNEL)
            continue;

         Mcap.Channel channel = (Mcap.Channel) record.body();
         ROS2MessageSchema schema = schemas.get(channel.schemaId());

         if (schema == null)
         {
            LogTools.error("Failed to find schema for channel: " + channel.id());
            continue;
         }

         try
         {
            if (!"cdr".equalsIgnoreCase(channel.messageEncoding().str()))
            {
               throw new UnsupportedOperationException("Only CDR encoding is supported for now.");
            }

            String topic = channel.topic().str();
            topic = topic.replace("/", YoTools.NAMESPACE_SEPERATOR_STRING);
            if (topic.startsWith(YoTools.NAMESPACE_SEPERATOR_STRING))
            {
               topic = topic.substring(YoTools.NAMESPACE_SEPERATOR_STRING.length());
            }
            YoNamespace namespace = new YoNamespace(topic).prepend(mcapRegistry.getNamespace());
            YoRegistry channelRegistry = SharedMemoryTools.ensurePathExists(mcapRegistry, namespace);
            yoMessageMap.put(channel.id(), YoROS2Message.newMessage(schema, channel.id(), channelRegistry));
         }
         catch (Exception e)
         {
            File debugFile = SCS2_MCAP_DEBUG_HOME.resolve("channel-%d-schema-%s-%s.txt".formatted(channel.id(), schema.getName(), e.getClass().getSimpleName()))
                                                 .toFile();
            if (debugFile.exists())
               debugFile.delete();
            debugFile.createNewFile();
            PrintWriter pw = new PrintWriter(debugFile);
            pw.write(channel.toString());
            pw.close();
            throw e;
         }
      }
   }

   public double getCurrentTimeInLog()
   {
      return (currentTimestamp.getValue() - getInitialTimestamp()) / 1.0e9;
   }

   public void initialize()
   {
      chunkIndex.set(0);
      Mcap.Chunk chunk = allChunks.get(chunkIndex.getValue());
      currentChunkStartTimestamp.set(chunk.messageStartTime());
      currentChunkEndTimestamp.set(chunk.messageEndTime());
      currentTimestamp.set(chunk.messageStartTime());
      initializeMessages(chunk);
      Mcap.Message message = currentChunkMessages.peek();
      if (chunk.messageStartTime() != message.logTime())
         throw new IllegalStateException("First message time (%d) does not match chunk start time (%d)".formatted(message.logTime(), chunk.messageStartTime()));

      readMessagesAtCurrentTimestamp();
   }

   public boolean loadNextMessageBatch()
   {
      if (currentChunkMessages.isEmpty())
      {
         // Load the next chunk
         chunkIndex.increment();
         if (chunkIndex.getValue() >= allChunks.size())
            return true;
         Mcap.Chunk chunk = allChunks.get(chunkIndex.getValue());
         currentTimestamp.set(chunk.messageStartTime());
         currentChunkStartTimestamp.set(chunk.messageStartTime());
         currentChunkEndTimestamp.set(chunk.messageEndTime());

         initializeMessages(chunk);

         Mcap.Message message = currentChunkMessages.peek();
         if (chunk.messageStartTime() != message.logTime())
            throw new IllegalStateException("First message time (%d) does not match chunk start time (%d)".formatted(message.logTime(),
                                                                                                                     chunk.messageStartTime()));
      }
      else
      {
         currentTimestamp.set(currentChunkMessages.peek().logTime());
      }

      readMessagesAtCurrentTimestamp();
      return false;
   }

   private final LZ4FrameDecoder chunkDataDecoder = new LZ4FrameDecoder();

   public void initializeMessages(Mcap.Chunk chunk)
   {
      byte[] decompressedChunk = new byte[(int) chunk.uncompressedSize()];
      chunkDataDecoder.decode((byte[]) chunk.records(), decompressedChunk);
      Mcap.Records records = new Mcap.Records(new ByteBufferKaitaiStream(decompressedChunk));
      currentChunkMessages.clear();
      for (Mcap.Record record : records.records())
      {
         if (record.op() != Mcap.Opcode.MESSAGE)
            continue;
         Mcap.Message message = (Mcap.Message) record.body();

         currentChunkMessages.add(message);
      }
   }

   public void readMessagesAtCurrentTimestamp()
   {
      Mcap.Message message = currentChunkMessages.peek();

      while (message.logTime() == currentTimestamp.getValue())
      {
         YoROS2Message yoROS2Message = yoMessageMap.get(message.channelId());
         if (yoROS2Message == null)
            throw new IllegalStateException("No YoROS2Message found for channel ID " + message.channelId());
         try
         {
            yoROS2Message.readMessage(message);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
         currentChunkMessages.poll();
         if (currentChunkMessages.isEmpty())
            break;
         message = currentChunkMessages.peek();
      }
   }

   public void printStatistics() throws UnsupportedEncodingException
   {
      Mcap.Magic headerMagic = mcap.headerMagic();
      byte[] magic = headerMagic.magic();
      byte[] restOfMagic = Arrays.copyOfRange(magic, 1, magic.length);
      printer.print("headerMagic = " + String.format("%02X", magic[0]) + " " + new String(restOfMagic, "UTF-8"));

      List<Mcap.Record> records = mcap.records();
      printer.println("Number of records: " + records.size());

      Mcap.Record footer = mcap.footer();
      if (footer != null)
      {
         Mcap.Footer footerBody = (Mcap.Footer) footer.body();
         long summaryCrc32 = footerBody.summaryCrc32();
         printer.println("Footer: summaryCrc32 = " + summaryCrc32);
         printer.println("");
      }
   }

   public File getMcapFile()
   {
      return mcapFile;
   }
}

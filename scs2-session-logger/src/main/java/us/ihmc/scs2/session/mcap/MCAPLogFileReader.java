package us.ihmc.scs2.session.mcap;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.channels.FileChannel;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.SessionIOTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoLong;

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
   private final FileInputStream mcapFileInputStream;
   private final FileChannel mcapFileChannel;
   private final MCAPDebugPrinter printer;
   private final YoRegistry mcapRegistry;
   private final Mcap mcap;
   private final MCAPChunkManager chunkManager = new MCAPChunkManager();
   private final TIntObjectHashMap<ROS2MessageSchema> schemas = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<YoROS2Message> yoMessageMap = new TIntObjectHashMap<>();
   private final MCAPFrameTransformManager frameTransformManager;
   private final YoLong currentChunkStartTimestamp = new YoLong("MCAPCurrentChunkStartTimestamp", propertiesRegistry);
   private final YoLong currentChunkEndTimestamp = new YoLong("MCAPCurrentChunkEndTimestamp", propertiesRegistry);
   private final YoLong currentTimestamp = new YoLong("MCAPCurrentTimestamp", propertiesRegistry);
   private final long initialTimestamp, finalTimestamp;

   public MCAPLogFileReader(File mcapFile, MCAPDebugPrinter printer, ReferenceFrame inertialFrame, YoRegistry mcapRegistry) throws IOException
   {
      this.mcapFile = mcapFile;
      this.printer = printer;
      this.mcapRegistry = mcapRegistry;
      mcapRegistry.addChild(propertiesRegistry);
      mcapFileInputStream = new FileInputStream(mcapFile);
      mcapFileChannel = mcapFileInputStream.getChannel();
      mcap = new Mcap(mcapFileChannel);
      chunkManager.loadFromMCAP(mcap);
      initialTimestamp = chunkManager.firstMessageTimestamp();
      finalTimestamp = chunkManager.lastMessageTimestamp();
      frameTransformManager = new MCAPFrameTransformManager(inertialFrame);
      mcapRegistry.addChild(frameTransformManager.getRegistry());
   }

   public long getInitialTimestamp()
   {
      return initialTimestamp;
   }

   public long getFinalTimestamp()
   {
      return finalTimestamp;
   }

   public long getTimestampAtIndex(int index)
   {
      return chunkManager.getTimestampAtIndex(index);
   }

   public long getRelativeTimestampAtIndex(int index)
   {
      return chunkManager.getRelativeTimestampAtIndex(index);
   }

   public int getCurrentIndex()
   {
      return chunkManager.getIndexFromTimestamp(currentTimestamp.getValue());
   }

   public int getNumberOfEntries()
   {
      return chunkManager.getNumberOfEntries();
   }

   public void loadSchemas() throws IOException
   {
      frameTransformManager.initialize(mcap);

      // TODO Skip creation of YoROS2Message for the frame transform stuff.
      for (Mcap.Record record : mcap.records())
      {
         if (record.op() != Mcap.Opcode.SCHEMA)
            continue;
         Mcap.Schema schema = (Mcap.Schema) record.body();
         if (schema.id() == frameTransformManager.getFrameTransformSchema().getId())
            continue;
         try
         {
            if (schema.encoding().str().equalsIgnoreCase("ros2msg"))
            {
               schemas.put(schema.id(), ROS2MessageSchema.loadSchema(schema));
            }
            else
            {
               throw new UnsupportedOperationException("Unsupported encoding: " + schema.encoding().str());
            }
         }
         catch (Exception e)
         {
            File debugFile = exportSchemaToFile(SCS2_MCAP_DEBUG_HOME, schema, e);
            LogTools.info("Failed to load schema: " + schema.name().str() + ", saved to: " + debugFile.getAbsolutePath());
            throw e;
         }
         finally
         {
            record.unloadBody();
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
         if (channel.schemaId() == frameTransformManager.getFrameTransformSchema().getId())
            continue;
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
            exportChannelToFile(SCS2_MCAP_DEBUG_HOME, channel, schema, e);
            throw e;
         }
      }
   }

   public double getCurrentTimeInLog()
   {
      return (currentTimestamp.getValue() - getInitialTimestamp()) / 1.0e9;
   }

   public void initialize() throws IOException
   {
      chunkManager.loadChunk(initialTimestamp);
      currentChunkStartTimestamp.set(chunkManager.getActiveChunkStartTimestamp());
      currentChunkEndTimestamp.set(chunkManager.getActiveChunkEndTimestamp());
      currentTimestamp.set(chunkManager.getActiveChunkStartTimestamp());
      readMessagesAtCurrentTimestamp();
   }

   public void setCurrentTimestamp(long timestamp)
   {
      currentTimestamp.set(timestamp);
      try
      {
         chunkManager.loadChunk(timestamp);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public YoGraphicDefinition getYoGraphic()
   {
      return frameTransformManager.getYoGraphic();
   }

   public boolean incrementTimestamp()
   {
      long nextTimestamp = chunkManager.nextMessageTimestamp(currentTimestamp.getValue());
      if (nextTimestamp == -1)
         return true;
      currentTimestamp.set(nextTimestamp);
      return false;
   }

   public void readMessagesAtCurrentTimestamp() throws IOException
   {
      List<Mcap.Message> messages = chunkManager.loadMessages(currentTimestamp.getValue());
      currentChunkStartTimestamp.set(chunkManager.getActiveChunkStartTimestamp());
      currentChunkEndTimestamp.set(chunkManager.getActiveChunkEndTimestamp());

      for (Mcap.Message message : messages)
      {
         try
         {
            boolean wasAFrameTransform = frameTransformManager.readMessage(message);
            if (wasAFrameTransform)
               continue;

            YoROS2Message yoROS2Message = yoMessageMap.get(message.channelId());
            if (yoROS2Message == null)
               throw new IllegalStateException("No YoROS2Message found for channel ID " + message.channelId());
            yoROS2Message.readMessage(message);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }
      // Update the Tf transforms wrt to world.
      frameTransformManager.update();
   }

   public void printStatistics() throws IOException
   {
      //      mcap.records().stream().filter(r -> r.op() != Opcode.MESSAGE_INDEX && r.op() != Opcode.CHUNK_INDEX).forEach(r -> printer.println(r.toString()));
      //      for (Mcap.Record record : mcap.records())
      //      {
      //         if (record.body() instanceof Mcap.Schema schema)
      //            exportSchemaToFile(SCS2_MCAP_DEBUG_HOME, schema, null);
      //         if (record.body() instanceof Mcap.Channel channel)
      //            exportChannelToFile(SCS2_MCAP_DEBUG_HOME, channel, yoMessageMap.get(channel.id()).getSchema(), null);
      //      }
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

   public File exportSchemaToFile(Path path, Mcap.Schema schema, Exception e) throws IOException, FileNotFoundException
   {
      String filename;
      if (e != null)
         filename = "schema-%s-%s.txt".formatted(schema.name().str().replace(':', '-'), e.getClass().getSimpleName());
      else
         filename = "schema-%s.txt".formatted(schema.name().str().replace(':', '-'));
      File debugFile = path.resolve(filename).toFile();
      if (debugFile.exists())
         debugFile.delete();
      debugFile.createNewFile();
      FileOutputStream os = new FileOutputStream(debugFile);
      os.write(schema.data());
      os.close();
      return debugFile;
   }

   private static void exportChannelToFile(Path path, Mcap.Channel channel, ROS2MessageSchema schema, Exception e) throws IOException
   {
      File debugFile;
      if (e != null)
         debugFile = path.resolve("channel-%d-schema-%s-%s.txt".formatted(channel.id(), schema.getName().replace(':', '-'), e.getClass().getSimpleName()))
                         .toFile();
      else
         debugFile = path.resolve("channel-%d-schema-%s.txt".formatted(channel.id(), schema.getName().replace(':', '-'))).toFile();
      if (debugFile.exists())
         debugFile.delete();
      debugFile.createNewFile();
      PrintWriter pw = new PrintWriter(debugFile);
      pw.write(channel.toString());
      pw.close();
   }

   public MCAPChunkManager getChunkManager()
   {
      return chunkManager;
   }

   public File getMcapFile()
   {
      return mcapFile;
   }

   public MCAPFrameTransformManager getFrameTransformManager()
   {
      return frameTransformManager;
   }
}

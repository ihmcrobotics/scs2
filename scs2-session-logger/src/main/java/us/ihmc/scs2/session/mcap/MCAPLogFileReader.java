package us.ihmc.scs2.session.mcap;

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

import java.io.*;
import java.nio.channels.FileChannel;
import java.nio.file.Path;
import java.util.List;

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
   private final YoRegistry mcapRegistry;
   private final MCAP mcap;
   private final MCAPChunkManager chunkManager = new MCAPChunkManager();
   private final TIntObjectHashMap<MCAPSchema> schemas = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<YoMCAPMessage> yoMessageMap = new TIntObjectHashMap<>();
   private final MCAPFrameTransformManager frameTransformManager;
   private final YoLong currentChunkStartTimestamp = new YoLong("MCAPCurrentChunkStartTimestamp", propertiesRegistry);
   private final YoLong currentChunkEndTimestamp = new YoLong("MCAPCurrentChunkEndTimestamp", propertiesRegistry);
   private final YoLong currentTimestamp = new YoLong("MCAPCurrentTimestamp", propertiesRegistry);
   private final long initialTimestamp, finalTimestamp;

   public MCAPLogFileReader(File mcapFile, ReferenceFrame inertialFrame, YoRegistry mcapRegistry) throws IOException
   {
      this.mcapFile = mcapFile;
      this.mcapRegistry = mcapRegistry;
      mcapRegistry.addChild(propertiesRegistry);
      mcapFileInputStream = new FileInputStream(mcapFile);
      mcapFileChannel = mcapFileInputStream.getChannel();
      mcap = new MCAP(mcapFileChannel);
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
      try
      {
         frameTransformManager.initialize(mcap);
      }
      catch (Exception e)
      {
         MCAP.Schema schema = frameTransformManager.getMCAPSchema();
         File debugFile = exportSchemaToFile(SCS2_MCAP_DEBUG_HOME, schema, e);
         LogTools.error("Failed to load schema: " + schema.name() + ", saved to: " + debugFile.getAbsolutePath());
         throw e;
      }

      for (MCAP.Record record : mcap.records())
      {
         if (record.op() != MCAP.Opcode.SCHEMA)
            continue;
         MCAP.Schema schema = (MCAP.Schema) record.body();
         if (schema.id() == frameTransformManager.getFrameTransformSchema().getId())
            continue;
         try
         {
            if (schema.encoding().equalsIgnoreCase("ros2msg"))
            {
               schemas.put(schema.id(), ROS2SchemaParser.loadSchema(schema));
            }
            else if (schema.encoding().equalsIgnoreCase("omgidl"))
            {
               schemas.put(schema.id(), OMGIDLSchemaParser.loadSchema(schema));
            }
            else
            {
               throw new UnsupportedOperationException("Unsupported encoding: " + schema.encoding());
            }
         }
         catch (Exception e)
         {
            File debugFile = exportSchemaToFile(SCS2_MCAP_DEBUG_HOME, schema, e);
            LogTools.error("Failed to load schema: " + schema.name() + ", saved to: " + debugFile.getAbsolutePath());
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
      for (MCAP.Record record : mcap.records())
      {
         if (record.op() != MCAP.Opcode.CHANNEL)
            continue;
         MCAP.Channel channel = (MCAP.Channel) record.body();
         if (channel.schemaId() == frameTransformManager.getFrameTransformSchema().getId())
            continue;

         MCAPSchema schema = schemas.get(channel.schemaId());

         if (schema == null)
         {
            LogTools.error("Failed to find schema for channel: " + channel.id());
            continue;
         }
         try
         {
            if (!"cdr".equalsIgnoreCase(channel.messageEncoding()))
            {
               throw new UnsupportedOperationException("Only CDR encoding is supported for now.");
            }
            String topic = channel.topic();
            topic = topic.replace("/", YoTools.NAMESPACE_SEPERATOR_STRING);
            if (topic.startsWith(YoTools.NAMESPACE_SEPERATOR_STRING))
            {
               topic = topic.substring(YoTools.NAMESPACE_SEPERATOR_STRING.length());
            }
            YoNamespace namespace = new YoNamespace(topic).prepend(mcapRegistry.getNamespace());
            YoRegistry channelRegistry = SharedMemoryTools.ensurePathExists(mcapRegistry, namespace);
            yoMessageMap.put(channel.id(), YoMCAPMessage.newMessage(schema, channel.id(), channelRegistry));
         }
         catch (Exception e)
         {
            exportChannelToFile(SCS2_MCAP_DEBUG_HOME, channel, schema, e);
            e.printStackTrace();
            //            throw e;
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
      List<MCAP.Message> messages = chunkManager.loadMessages(currentTimestamp.getValue());
      currentChunkStartTimestamp.set(chunkManager.getActiveChunkStartTimestamp());
      currentChunkEndTimestamp.set(chunkManager.getActiveChunkEndTimestamp());

      for (MCAP.Message message : messages)
      {
         try
         {
            boolean wasAFrameTransform = frameTransformManager.readMessage(message);
            if (wasAFrameTransform)
               continue;

            YoMCAPMessage yoMCAPMessage = yoMessageMap.get(message.channelId());

            if (yoMCAPMessage == null)
            {
               throw new IllegalStateException("No YoMCAP message found for channel ID " + message.channelId());
            }
            yoMCAPMessage.readMessage(message);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }
      // Update the Tf transforms wrt to world.
      frameTransformManager.update();
   }

   public File exportSchemaToFile(Path path, MCAP.Schema schema, Exception e) throws IOException
   {
      String filename;
      if (e != null)
         filename = "schema-%s-%s.txt".formatted(schema.name().replace(':', '-'), e.getClass().getSimpleName());
      else
         filename = "schema-%s.txt".formatted(schema.name().replace(':', '-'));
      File debugFile = path.resolve(filename).toFile();
      if (debugFile.exists())
         debugFile.delete();
      debugFile.createNewFile();
      FileOutputStream os = new FileOutputStream(debugFile);
      os.write(schema.data());
      os.close();
      return debugFile;
   }

   private static void exportChannelToFile(Path path, MCAP.Channel channel, MCAPSchema schema, Exception e) throws IOException
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

package us.ihmc.scs2.session.mcap;

import gnu.trove.map.hash.TIntObjectHashMap;
import org.apache.commons.io.FileUtils;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.SessionIOTools;
import us.ihmc.scs2.session.mcap.MCAP.Schema;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.channels.FileChannel;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;

public class MCAPLogFileReader
{
   private static final Set<String> SCHEMA_TO_IGNORE = Set.of("foxglove::Grid", "foxglove::SceneUpdate", "foxglove::FrameTransforms", "HandDeviceHealth");
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
   private final YoRegistry mcapRegistry;
   private final MCAP mcap;
   private final MCAPBufferedChunk chunkBuffer;
   private final MCAPMessageManager messageManager;
   private final MCAPConsoleLogManager consoleLogManager;
   private final TIntObjectHashMap<MCAPSchema> schemas = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<MCAP.Schema> rawSchemas = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<YoMCAPMessage> yoMessageMap = new TIntObjectHashMap<>();
   private final MCAPFrameTransformManager frameTransformManager;
   private final YoLong currentChunkStartTimestamp = new YoLong("MCAPCurrentChunkStartTimestamp", propertiesRegistry);
   private final YoLong currentChunkEndTimestamp = new YoLong("MCAPCurrentChunkEndTimestamp", propertiesRegistry);
   private final YoLong currentTimestamp = new YoLong("MCAPCurrentTimestamp", propertiesRegistry);
   /**
    * When greater than 0, the log reader will enforce a regular time step. The log time of messages will be rounded to the closest multiple of this value.
    */
   private final long desiredLogDT;
   private final long initialTimestamp, finalTimestamp;

   public MCAPLogFileReader(File mcapFile, long desiredLogDT, ReferenceFrame inertialFrame, YoRegistry mcapRegistry, YoRegistry internalRegistry)
         throws IOException
   {
      if (SCS2_MCAP_DEBUG_HOME.toFile().exists())
      {
         // Cleaning up the debug folder.
         FileUtils.cleanDirectory(SCS2_MCAP_DEBUG_HOME.toFile());
      }
      this.mcapFile = mcapFile;
      this.desiredLogDT = desiredLogDT;
      this.mcapRegistry = mcapRegistry;
      mcapRegistry.addChild(propertiesRegistry);
      long startTime = System.nanoTime();
      FileInputStream mcapFileInputStream = new FileInputStream(mcapFile);
      FileChannel mcapFileChannel = mcapFileInputStream.getChannel();
      LogTools.info("Opened file channel in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));
      startTime = System.nanoTime();
      mcap = new MCAP(mcapFileChannel); // On 10GB log file, this takes about 4-5 seconds.
      LogTools.info("Created MCAP object in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));
      startTime = System.nanoTime();
      chunkBuffer = new MCAPBufferedChunk(mcap, desiredLogDT); // On 10GB log file, this takes about 9 seconds.
      LogTools.info("Created chunk buffer in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));

      startTime = System.nanoTime();
      messageManager = new MCAPMessageManager(mcap, chunkBuffer, desiredLogDT); // On 10GB log file, this takes about 7 seconds.
      LogTools.info("Created message manager in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));

      currentTimestamp.addListener(v -> chunkBuffer.preloadChunks(currentTimestamp.getValue(), TimeUnit.MILLISECONDS.toNanos(500)));

      initialTimestamp = messageManager.firstMessageTimestamp();
      finalTimestamp = messageManager.lastMessageTimestamp();
      startTime = System.nanoTime();
      frameTransformManager = new MCAPFrameTransformManager(inertialFrame); // This is fast.
      mcapRegistry.addChild(frameTransformManager.getRegistry());
      LogTools.info("Created frame transform manager in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));

      startTime = System.nanoTime();
      loadSchemas(); // On 10GB log file, this takes about 32 seconds.
      LogTools.info("Loaded schemas in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));
      startTime = System.nanoTime();
      loadChannels(); // This is fast.
      LogTools.info("Loaded channels in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));

      startTime = System.nanoTime();
      // Doing this last to not slow down the loading.
      consoleLogManager = new MCAPConsoleLogManager(mcap, chunkBuffer, desiredLogDT); // This is fast on the main thread, loading in a separate thread.
      LogTools.info("Created console log manager in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));
   }

   public long getDesiredLogDT()
   {
      return desiredLogDT;
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
      return messageManager.getTimestampAtIndex(index);
   }

   public YoLong getCurrentTimestamp()
   {
      return currentTimestamp;
   }

   public long getRelativeTimestampAtIndex(int index)
   {
      return messageManager.getRelativeTimestampAtIndex(index);
   }

   public int getCurrentIndex()
   {
      return messageManager.getIndexFromTimestamp(currentTimestamp.getValue());
   }

   public int getIndexFromTimestamp(long timestamp)
   {
      return messageManager.getIndexFromTimestamp(timestamp);
   }

   public int getNumberOfEntries()
   {
      return messageManager.getNumberOfEntries();
   }

   private void loadSchemas() throws IOException
   {
      try
      {
         frameTransformManager.initialize(mcap, chunkBuffer);
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

         rawSchemas.put(schema.id(), schema);

         if (SCHEMA_TO_IGNORE.contains(schema.name()))
            continue;

         if (frameTransformManager.hasMCAPFrameTransforms() && schema.id() == frameTransformManager.getFrameTransformSchema().getId())
            continue;

         try
         {
            if (schema.encoding().equalsIgnoreCase("ros2msg"))
               schemas.put(schema.id(), ROS2SchemaParser.loadSchema(schema));
            else if (schema.encoding().equalsIgnoreCase("omgidl"))
               schemas.put(schema.id(), OMGIDLSchemaParser.loadSchema(schema));
            else
               throw new UnsupportedOperationException("Unsupported encoding: " + schema.encoding());
         }
         catch (Exception e)
         {
            File debugFile = exportSchemaToFile(SCS2_MCAP_DEBUG_HOME, schema, e);
            LogTools.error("Failed to load schema: " + schema.name() + ", saved to: " + debugFile.getAbsolutePath());
            throw e;
         }
      }
   }

   private void loadChannels() throws IOException
   {
      for (MCAP.Record record : mcap.records())
      {
         if (record.op() != MCAP.Opcode.CHANNEL)
            continue;
         MCAP.Channel channel = (MCAP.Channel) record.body();
         if (frameTransformManager.hasMCAPFrameTransforms() && channel.schemaId() == frameTransformManager.getFrameTransformSchema().getId())
            continue;

         MCAPSchema schema = schemas.get(channel.schemaId());

         if (schema == null)
         {
            Schema rawSchema = rawSchemas.get(channel.schemaId());
            if (rawSchema != null && SCHEMA_TO_IGNORE.contains(rawSchema.name()))
               continue;

            LogTools.error("Failed to find schema for channel: " + channel.id() + ", schema ID: " + channel.schemaId());
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
            YoMCAPMessage newMessage = YoMCAPMessage.newMessage(schema, channel.id(), channelRegistry);
            if (channelRegistry.getNumberOfVariablesDeep() > 15000)
            {
               LogTools.warn("Message registry has more than 15000 variables, schema {}, topic {}. This may cause performance issues.",
                             schema.getName(),
                             channel.topic());
            }
            yoMessageMap.put(channel.id(), newMessage);
         }
         catch (Exception e)
         {
            exportChannelToFile(SCS2_MCAP_DEBUG_HOME, channel, schema, e);
            LogTools.error("Failed to load channel: " + channel.id() + ", schema ID: " + channel.schemaId() + ", saved to: " + SCS2_MCAP_DEBUG_HOME);
            e.printStackTrace();
         }
      }
   }

   public double getCurrentTimeInLog()
   {
      return (currentTimestamp.getValue() - getInitialTimestamp()) / 1.0e9;
   }

   public long getCurrentRelativeTimestamp()
   {
      return currentTimestamp.getValue() - getInitialTimestamp();
   }

   public void initialize() throws IOException
   {
      currentTimestamp.set(initialTimestamp);
      readMessagesAtCurrentTimestamp();
   }

   public void setCurrentTimestamp(long timestamp)
   {
      currentTimestamp.set(timestamp);
      chunkBuffer.requestLoadChunk(timestamp, false);
   }

   public YoGraphicDefinition getYoGraphic()
   {
      return frameTransformManager.getYoGraphic();
   }

   public boolean incrementTimestamp()
   {
      long nextTimestamp = messageManager.nextMessageTimestamp(currentTimestamp.getValue());
      if (nextTimestamp == -1)
         return true;
      currentTimestamp.set(nextTimestamp);
      return false;
   }

   public void readMessagesAtCurrentTimestamp() throws IOException
   {
      List<MCAP.Message> messages = messageManager.loadMessages(currentTimestamp.getValue());
      if (messages == null)
      {
         LogTools.warn("No messages at timestamp {}.", currentTimestamp.getValue());
         return;
      }
      currentChunkStartTimestamp.set(messageManager.getActiveChunkStartTimestamp());
      currentChunkEndTimestamp.set(messageManager.getActiveChunkEndTimestamp());

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
               //               throw new IllegalStateException("No YoMCAP message found for channel ID " + message.channelId());
               continue;
            }
            yoMCAPMessage.readMessage(message);
         }
         catch (Exception e)
         {
            e.printStackTrace();

            YoMCAPMessage yoMCAPMessage = yoMessageMap.get(message.channelId());
            if (yoMCAPMessage != null)
            {
               LogTools.error("Failed to read message. Channel ID {}, schema name: {}. Exporting message data & schema to file.",
                              message.channelId(),
                              yoMCAPMessage.getSchema().getName());
               exportMessageDataToFile(SCS2_MCAP_DEBUG_HOME, message, yoMCAPMessage.getSchema(), e);
               exportSchemaToFile(SCS2_MCAP_DEBUG_HOME, rawSchemas.get(yoMCAPMessage.getSchema().getId()), e);
            }
         }
      }
      // Update the Tf transforms wrt to world.
      frameTransformManager.update();
   }

   public File exportSchemaToFile(Path path, MCAP.Schema schema, Exception e) throws IOException
   {
      String filename;
      if (e != null)
         filename = "schema-%s-%s.txt".formatted(cleanupName(schema.name()), e.getClass().getSimpleName());
      else
         filename = "schema-%s.txt".formatted(cleanupName(schema.name()));
      File debugFile = path.resolve(filename).toFile();
      if (debugFile.exists())
         debugFile.delete();
      debugFile.createNewFile();
      FileOutputStream os = new FileOutputStream(debugFile);
      os.getChannel().write(schema.data());
      os.close();
      return debugFile;
   }

   private static void exportChannelToFile(Path path, MCAP.Channel channel, MCAPSchema schema, Exception e) throws IOException
   {
      File debugFile;
      if (e != null)
         debugFile = path.resolve("channel-%d-schema-%s-%s.txt".formatted(channel.id(), cleanupName(schema.getName()), e.getClass().getSimpleName())).toFile();
      else
         debugFile = path.resolve("channel-%d-schema-%s.txt".formatted(channel.id(), cleanupName(schema.getName()))).toFile();
      if (debugFile.exists())
         debugFile.delete();
      debugFile.createNewFile();
      PrintWriter pw = new PrintWriter(debugFile);
      pw.write(channel.toString());
      pw.close();
   }

   private static void exportMessageDataToFile(Path path, MCAP.Message message, MCAPSchema schema, Exception e) throws IOException
   {
      File debugFile;
      String prefix = "messageData-timestamp-%d-schema-%s";
      if (e != null)
         debugFile = path.resolve((prefix + "-%s.txt").formatted(message.logTime(), cleanupName(schema.getName()), e.getClass().getSimpleName())).toFile();
      else
         debugFile = path.resolve((prefix + ".txt").formatted(message.logTime(), cleanupName(schema.getName()))).toFile();

      if (debugFile.exists())
         debugFile.delete();
      debugFile.createNewFile();
      FileOutputStream os = new FileOutputStream(debugFile);
      os.write(message.messageData());
      os.close();
   }

   private static String cleanupName(String name)
   {
      return name.replace(':', '-');
   }

   public MCAPMessageManager getMessageManager()
   {
      return messageManager;
   }

   public MCAPConsoleLogManager getConsoleLogManager()
   {
      return consoleLogManager;
   }

   public File getMcapFile()
   {
      return mcapFile;
   }

   public MCAPFrameTransformManager getFrameTransformManager()
   {
      return frameTransformManager;
   }

   public RobotStateUpdater createRobotStateUpdater(Robot robot)
   {
      if (frameTransformManager.hasMCAPFrameTransforms())
      {
         return new MCAPFrameTransformBasedRobotStateUpdater(robot, frameTransformManager);
      }

      for (YoMCAPMessage yoMCAPMessage : yoMessageMap.valueCollection())
      {
         if (MCAPMujocoBasedRobotStateUpdater.isRobotMujocoStateMessage(robot, yoMCAPMessage))
            return new MCAPMujocoBasedRobotStateUpdater(robot, yoMCAPMessage);
      }

      return null;
   }
}

package us.ihmc.scs2.session.mcap;

import gnu.trove.map.hash.TIntLongHashMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import org.apache.commons.io.FileUtils;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.SessionIOTools;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Channel;
import us.ihmc.scs2.session.mcap.specs.records.ChannelMessageCount;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Schema;
import us.ihmc.scs2.session.mcap.specs.records.Statistics;
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
import java.util.function.Predicate;

public class MCAPLogFileReader
{
   public static final Set<String> SCHEMA_TO_IGNORE = Set.of("foxglove::Grid", "foxglove::SceneUpdate", "foxglove::FrameTransforms", "HandDeviceHealth");
   public static final Path SCS2_MCAP_DEBUG_HOME = SessionIOTools.SCS2_HOME.resolve("mcap-debug");

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
   private final TIntObjectHashMap<Schema> rawSchemas = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<MCAPMessageDecoder> yoMessageMap = new TIntObjectHashMap<>();
   /**
    * Per-channel message counts from the file's {@link Statistics} record, used to skip building YoVariables for
    * channels that never actually appear in the log (e.g. schemas with large/nested unbounded arrays can otherwise
    * blow up into hundreds of thousands of never-updated variables). {@code null} if the file has no Statistics
    * record, in which case no channel is skipped.
    */
   private final TIntLongHashMap channelMessageCounts = new TIntLongHashMap();
   private boolean hasChannelMessageCounts = false;
   private final MCAPFrameTransformManager frameTransformManager;
   private final MCAPJointStateManager jointStateManager = new MCAPJointStateManager();
   private final MCAPOdometryManager odometryManager = new MCAPOdometryManager();
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

      loadStatistics();

      startTime = System.nanoTime();
      // Must run before loadSchemas()/loadChannels(), which exclude this schema from generic decoding: a real
      // nav_msgs/Odometry message can't be generically decoded at all (PoseWithCovariance.pose nests a field also
      // named "pose", which SCS2's YoRegistry namespace rules reject), so it's hand-parsed here instead.
      odometryManager.initialize(mcap);
      LogTools.info("Created odometry manager in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));

      startTime = System.nanoTime();
      loadSchemas(); // On 10GB log file, this takes about 32 seconds.
      LogTools.info("Loaded schemas in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));
      startTime = System.nanoTime();
      loadChannels(); // This is fast.
      LogTools.info("Loaded channels in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));

      startTime = System.nanoTime();
      // Runs after loadChannels() (unlike frameTransformManager/odometryManager, which must run before schema
      // loading to exclude their own schema from generic decoding) since this manager wants the opposite: for
      // /joint_states to remain generically decoded, and to look up its already-built YoMCAPMessage from yoMessageMap.
      jointStateManager.initialize(mcap, chunkBuffer, yoMessageMap);
      LogTools.info("Created joint state manager in {} ms.", TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));

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

   /**
    * Reads the file's {@link Statistics} record (if any) to find out, per channel, how many messages were actually
    * recorded. Schemas can be declared (and channels opened) for topics that were never actually published to during
    * the recording; building a full YoVariable tree for those wastes memory/UI performance for data that will never
    * update, so {@link #loadChannels()} uses this to skip them entirely.
    */
   private void loadStatistics()
   {
      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.STATISTICS)
            continue;
         Statistics statistics = (Statistics) record.body();
         for (ChannelMessageCount count : statistics.channelMessageCounts())
            channelMessageCounts.put(count.channelId(), count.messageCount());
         hasChannelMessageCounts = true;
      }
   }

   private void loadSchemas() throws IOException
   {
      try
      {
         frameTransformManager.initialize(mcap, chunkBuffer);
      }
      catch (Exception e)
      {
         Schema schema = frameTransformManager.getMCAPSchema();
         File debugFile = exportSchemaToFile(SCS2_MCAP_DEBUG_HOME, schema, e);
         LogTools.error("Failed to load schema: " + schema.name() + ", saved to: " + debugFile.getAbsolutePath());
         throw e;
      }

      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.SCHEMA)
            continue;
         Schema schema = (Schema) record.body();

         rawSchemas.put(schema.id(), schema);

         if (SCHEMA_TO_IGNORE.contains(schema.name()))
            continue;

         if (frameTransformManager.hasMCAPFrameTransforms() && schema.id() == frameTransformManager.getFrameTransformSchema().getId())
            continue;

         if (odometryManager.hasOdometrySchema() && schema.id() == odometryManager.getOdometrySchemaId())
            continue;

         try
         {
            if (schema.encoding().equalsIgnoreCase("ros2msg"))
               schemas.put(schema.id(), ROS2SchemaParser.loadSchema(schema));
            else if (schema.encoding().equalsIgnoreCase("omgidl"))
               schemas.put(schema.id(), OMGIDLSchemaParser.loadSchema(schema));
            else if (schema.encoding().equalsIgnoreCase("protobuf"))
               schemas.put(schema.id(), ProtobufSchemaParser.loadSchema(schema));
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
      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.CHANNEL)
            continue;
         Channel channel = (Channel) record.body();

         if (hasChannelMessageCounts && channelMessageCounts.get(channel.id()) == 0L)
         {
            // No point building a YoVariable tree for a channel that never actually appears in the log; some
            // schemas (e.g. ROS2 messages with several unbounded array fields) can otherwise explode into hundreds
            // of thousands of variables that would never be updated.
            continue;
         }

         if (frameTransformManager.hasMCAPFrameTransforms() && channel.schemaId() == frameTransformManager.getFrameTransformSchema().getId())
            continue;

         if (odometryManager.hasOdometrySchema() && channel.schemaId() == odometryManager.getOdometrySchemaId())
            continue;
         if (yoMessageMap.containsKey(channel.id()))
            // Channels can legitimately be redeclared across multiple chunks; only build the message once.
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
            String topic = channel.topic();
            topic = topic.replace("/", YoTools.NAMESPACE_SEPERATOR_STRING);
            if (topic.startsWith(YoTools.NAMESPACE_SEPERATOR_STRING))
            {
               topic = topic.substring(YoTools.NAMESPACE_SEPERATOR_STRING.length());
            }
            YoNamespace namespace = new YoNamespace(topic).prepend(mcapRegistry.getNamespace());
            YoRegistry channelRegistry = SharedMemoryTools.ensurePathExists(mcapRegistry, namespace);

            MCAPMessageDecoder newMessage;
            if ("cdr".equalsIgnoreCase(channel.messageEncoding()))
               newMessage = YoMCAPMessage.newMessage(schema, channel.id(), channelRegistry);
            else if ("protobuf".equalsIgnoreCase(channel.messageEncoding()) && schema instanceof MCAPProtobufSchema protobufSchema)
            {
               // A sample message is needed to resolve protobuf `map` fields' key sets (see YoMCAPProtobufMessage);
               // the descriptor alone can't tell us which keys to build YoVariables for.
               Message sampleMessage = MCAPJointStateManager.findFirstMessage(mcap, chunkBuffer, channel.id());
               byte[] sampleMessageData = sampleMessage == null ? null : sampleMessage.messageData();
               newMessage = YoMCAPProtobufMessage.newMessage(protobufSchema, channel.id(), channelRegistry, sampleMessageData);
            }
            else
               throw new UnsupportedOperationException("Unsupported message encoding: " + channel.messageEncoding());

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
      List<Message> messages = messageManager.loadMessages(currentTimestamp.getValue());
      if (messages == null)
      {
         LogTools.warn("No messages at timestamp {}.", currentTimestamp.getValue());
         return;
      }
      currentChunkStartTimestamp.set(messageManager.getActiveChunkStartTimestamp());
      currentChunkEndTimestamp.set(messageManager.getActiveChunkEndTimestamp());

      for (Message message : messages)
      {
         try
         {
            boolean wasAFrameTransform = frameTransformManager.readMessage(message);
            if (wasAFrameTransform)
               continue;

            boolean wasOdometry = odometryManager.readMessage(message);
            if (wasOdometry)
               continue;

            MCAPMessageDecoder yoMCAPMessage = yoMessageMap.get(message.channelId());

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

            MCAPMessageDecoder yoMCAPMessage = yoMessageMap.get(message.channelId());
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

   public static File exportSchemaToFile(Path path, Schema schema, Exception e) throws IOException
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

   public static void exportChannelToFile(Path path, Channel channel, MCAPSchema schema, Exception e) throws IOException
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

   public static void exportMessageDataToFile(Path path, Message message, MCAPSchema schema, Exception e) throws IOException
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

   public static void exportChunkToFile(Path path, Chunk chunk, Exception e) throws IOException
   {
      File debugFile;
      if (e != null)
         debugFile = path.resolve("chunk-%d-%s.txt".formatted(chunk.messageStartTime(), e.getClass().getSimpleName())).toFile();
      else
         debugFile = path.resolve("chunk-%d.txt".formatted(chunk.messageStartTime())).toFile();
      if (debugFile.exists())
         debugFile.delete();
      debugFile.createNewFile();
      FileOutputStream os = new FileOutputStream(debugFile);
      MCAPDataOutput dataOutput = MCAPDataOutput.wrap(os.getChannel());
      chunk.write(dataOutput);
      dataOutput.close();
   }

   private static String cleanupName(String name)
   {
      return name.replace(':', '-').replace('/', '-');
   }

   public MCAPMessageManager getMessageManager()
   {
      return messageManager;
   }

   public MCAPConsoleLogManager getConsoleLogManager()
   {
      return consoleLogManager;
   }

   public MCAP getMCAP()
   {
      return mcap;
   }

   public File getMCAPFile()
   {
      return mcapFile;
   }

   public MCAPFrameTransformManager getFrameTransformManager()
   {
      return frameTransformManager;
   }

   /**
    * Tries each robot-state-driving strategy in priority order, returning the first one that can actually cover the
    * whole robot - not just the first one that superficially applies (a strategy "applying" and "actually covering
    * every joint" aren't the same thing, see {@link #tryFrameTransformBasedUpdater}). Falls back to {@code null} if
    * none of them do, e.g. an MCAP file with no robot-state channel this reader knows how to interpret at all.
    */
   public RobotStateUpdater createRobotStateUpdater(Robot robot)
   {
      RobotStateUpdater updater;
      updater = tryFrameTransformBasedUpdater(robot);

      if (updater != null)
         return updater;

      updater = tryControllerStateBasedUpdater(robot);
      if (updater != null)
         return updater;

      updater = tryMujocoBasedUpdater(robot);
      if (updater != null)
         return updater;

      // We return null if we don't have any valid robot updater, have to check if this is null wherever this is used
      return null;
   }

   private RobotStateUpdater tryFrameTransformBasedUpdater(Robot robot)
   {
      if (!frameTransformManager.hasMCAPFrameTransforms())
         return null;

      // hasMCAPFrameTransforms() only reflects that a /tf-shaped schema exists in the file, not that any
      // transforms were actually published (e.g. a /tf_static-only file with an empty /tf channel) - only commit
      // to this strategy if it actually resolved every joint, otherwise fall through to the others.
      MCAPFrameTransformBasedRobotStateUpdater updater = new MCAPFrameTransformBasedRobotStateUpdater(robot,
                                                                                                       frameTransformManager,
                                                                                                       jointStateManager,
                                                                                                       odometryManager,
                                                                                                       currentTimestamp);
      return updater.coversAllJoints() ? updater : null;
   }

   private RobotStateUpdater tryControllerStateBasedUpdater(Robot robot)
   {
      YoMCAPProtobufMessage message = findFirstDecoder(YoMCAPProtobufMessage.class, candidate -> MCAPControllerStateBasedRobotStateUpdater.isRobotControllerStateMessage(robot, candidate));
      return message == null ? null : new MCAPControllerStateBasedRobotStateUpdater(robot, message);
   }

   private RobotStateUpdater tryMujocoBasedUpdater(Robot robot)
   {
      YoMCAPMessage message = findFirstDecoder(YoMCAPMessage.class, candidate -> MCAPMujocoBasedRobotStateUpdater.isRobotMujocoStateMessage(robot, candidate));
      return message == null ? null : new MCAPMujocoBasedRobotStateUpdater(robot, message);
   }

   private <T extends MCAPMessageDecoder> T findFirstDecoder(Class<T> decoderType, Predicate<T> predicate)
   {
      for (MCAPMessageDecoder messageDecoder : yoMessageMap.valueCollection())
      {
         if (decoderType.isInstance(messageDecoder) && predicate.test(decoderType.cast(messageDecoder)))
            return decoderType.cast(messageDecoder);
      }

      return null;
   }
}

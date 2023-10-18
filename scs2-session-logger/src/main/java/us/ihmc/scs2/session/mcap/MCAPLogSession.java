package us.ihmc.scs2.session.mcap;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import java.util.stream.Collectors;

import gnu.trove.map.hash.TIntObjectHashMap;
import io.kaitai.struct.ByteBufferKaitaiStream;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.mcap.Mcap.Attachment;
import us.ihmc.scs2.session.mcap.Mcap.AttachmentIndex;
import us.ihmc.scs2.session.mcap.Mcap.Channel;
import us.ihmc.scs2.session.mcap.Mcap.Chunk;
import us.ihmc.scs2.session.mcap.Mcap.ChunkIndex;
import us.ihmc.scs2.session.mcap.Mcap.DataEnd;
import us.ihmc.scs2.session.mcap.Mcap.Footer;
import us.ihmc.scs2.session.mcap.Mcap.Header;
import us.ihmc.scs2.session.mcap.Mcap.Magic;
import us.ihmc.scs2.session.mcap.Mcap.MapStrStr;
import us.ihmc.scs2.session.mcap.Mcap.MessageIndex;
import us.ihmc.scs2.session.mcap.Mcap.MessageIndex.MessageIndexEntry;
import us.ihmc.scs2.session.mcap.Mcap.Metadata;
import us.ihmc.scs2.session.mcap.Mcap.MetadataIndex;
import us.ihmc.scs2.session.mcap.Mcap.Opcode;
import us.ihmc.scs2.session.mcap.Mcap.Record;
import us.ihmc.scs2.session.mcap.Mcap.Schema;
import us.ihmc.scs2.session.mcap.Mcap.SummaryOffset;
import us.ihmc.scs2.session.mcap.Mcap.TupleStrStr;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoTools;

public class MCAPLogSession extends Session
{
   // FIXME Figure out how to name the session
   private final String sessionName = getClass().getSimpleName();
   private final List<Robot> robots = new ArrayList<>();
   private final List<RobotDefinition> robotDefinitions = new ArrayList<>();
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();
   private final Runnable robotStateUpdater;
   private Mcap mcap;

   private final YoRegistry mcapRegistry = new YoRegistry("MCAP");
   private final File mcapFile;

   public MCAPLogSession(File mcapFile, MCAPDebugPrinter printer) throws IOException
   {
      // FIXME Do we need this guy?
      this.mcapFile = mcapFile;
      robotStateUpdater = null;
      mcap = Mcap.fromFile(mcapFile.getAbsolutePath());
      Magic headerMagic = mcap.headerMagic();
      byte[] magic = headerMagic.magic();
      byte[] restOfMagic = Arrays.copyOfRange(magic, 1, magic.length);
      printer.print("headerMagic = " + String.format("%02X", magic[0]) + " " + new String(restOfMagic, "UTF-8"));

      List<Record> records = mcap.records();
      printer.println("Number of records: " + records.size());

      TIntObjectHashMap<ROS2MessageSchema> schemas = new TIntObjectHashMap<>();

      for (Record record : records)
      {
         if (record.op() == Opcode.SCHEMA)
         {
            Schema schema = (Schema) record.body();
            schemas.put(schema.id(), ROS2MessageSchema.loadSchema(schema.name().str(), schema.data()));
         }
      }

      List<YoROS2Message> loadedChannels = new ArrayList<>();

      for (Record record : records)
      {
         if (record.op() == Opcode.CHANNEL)
         {
            Channel channel = (Channel) record.body();
            loadedChannels.add(instantiateChannel(channel, schemas, mcapRegistry));
         }
      }



      EnumSet<Opcode> alreadyPrintedOp = EnumSet.noneOf(Opcode.class);

      for (Record record : records)
      {
         Opcode op = record.op();

         if (alreadyPrintedOp.add(op))
         {
            printer.println(record.toString());
         }
      }

      Record footer = mcap.footer();
      if (footer != null)
      {
         Footer footerBody = (Footer) footer.body();
         long summaryCrc32 = footerBody.summaryCrc32();
         printer.println("Footer: summaryCrc32 = " + summaryCrc32);
         printer.println("");
      }

      rootRegistry.addChild(mcapRegistry);
   }

   private static YoROS2Message instantiateChannel(Channel channel, TIntObjectHashMap<ROS2MessageSchema> schemas, YoRegistry mcapRegistry)
   {
      String topic = channel.topic().str();
      topic = topic.replace("/", YoTools.NAMESPACE_SEPERATOR_STRING);
      if (topic.startsWith(YoTools.NAMESPACE_SEPERATOR_STRING))
         topic = topic.substring(YoTools.NAMESPACE_SEPERATOR_STRING.length());
      YoNamespace namespace = new YoNamespace(topic);
      namespace = namespace.prepend(mcapRegistry.getNamespace());
      System.out.println(namespace);
      YoRegistry channelRegistry = SharedMemoryTools.ensurePathExists(mcapRegistry, namespace);
      YoROS2Message message = new YoROS2Message(schemas.get(channel.schemaId()), channelRegistry);
      return message;
   }

   @Override
   protected double doSpecificRunTick()
   {
      return 0;
   }

   @Override
   public String getSessionName()
   {
      return sessionName;
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      return robotDefinitions;
   }

   @Override
   public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return Collections.emptyList();
   }

   @Override
   public List<YoGraphicDefinition> getYoGraphicDefinitions()
   {
      return yoGraphicDefinitions;
   }

   @Override
   public List<RobotStateDefinition> getCurrentRobotStateDefinitions(boolean initialState)
   {
      return robots.stream().map(Robot::getCurrentRobotStateDefinition).collect(Collectors.toList());
   }

   public File getMCAPFile()
   {
      return mcapFile;
   }
}

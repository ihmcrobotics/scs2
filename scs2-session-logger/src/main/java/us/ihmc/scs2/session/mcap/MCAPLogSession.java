package us.ihmc.scs2.session.mcap;

import java.io.File;
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
import us.ihmc.scs2.simulation.robot.Robot;

public class MCAPLogSession extends Session
{
   // FIXME Figure out how to name the session
   private final String sessionName = getClass().getSimpleName();
   private final List<Robot> robots = new ArrayList<>();
   private final List<RobotDefinition> robotDefinitions = new ArrayList<>();
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();
   private final Runnable robotStateUpdater;
   private Mcap mcap;

   public MCAPLogSession(File mcapFile, MCAPDebugPrinter printer) throws IOException
   {
      // FIXME Do we need this guy?
      robotStateUpdater = null;
      mcap = Mcap.fromFile(mcapFile.getAbsolutePath());
      Magic headerMagic = mcap.headerMagic();
      byte[] magic = headerMagic.magic();
      byte[] restOfMagic = Arrays.copyOfRange(magic, 1, magic.length);
      printer.print("headerMagic = " + String.format("%02X", magic[0]) + " " + new String(restOfMagic, "UTF-8"));

      List<Record> records = mcap.records();
      printer.println("Number of records: " + records.size());

      int metadataCount = 0;
      int metadataIndexCount = 0;
      int dataEndCount = 0;
      int messageIndexCount = 0;
      int chunkIndexCount = 0;
      int chunkCount = 0;
      int schemaCount = 0;
      int channelCount = 0;
      int summaryOffsetCount = 0;
      int attachmentCount = 0;

      int unsuccessfulDecompressedChunks = 0;
      EnumSet<Opcode> alreadyPrintedOp = EnumSet.noneOf(Opcode.class);

      for (Record record : records)
      {
         long lenBody = record.lenBody();
         Opcode op = record.op();

         if (alreadyPrintedOp.add(op))
         {
            printer.println(record.toString());
         }
         if (record.body() instanceof Schema schema)
         {
            if ("ros2msg".equals(schema.encoding().str()))
            {
               File schemaFile = new File(schema.name().str() + "-schema.ros2msg");
               if (schemaFile.exists())
                  schemaFile.delete();
               PrintWriter writer = new PrintWriter(schemaFile);
               writer.write(new String(schema.data()));
               writer.close();
            }
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

      printer.println("summaryOffsetCount = " + summaryOffsetCount);
      printer.println("attachmentCount = " + attachmentCount);
      printer.println("channelCount = " + channelCount);
      printer.println("chunkCount = " + chunkCount + ", successful decomp. = " + (chunkCount - unsuccessfulDecompressedChunks));
      printer.println("chunkIndexCount = " + chunkIndexCount);
      printer.println("dataEndCount = " + dataEndCount);

      printer.println("messageIndexCount = " + messageIndexCount);
      printer.println("metadataCount = " + metadataCount);
      printer.println("metadataIndexCount = " + metadataIndexCount);

      printer.println("schemaCount = " + schemaCount);
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
}

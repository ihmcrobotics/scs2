package us.ihmc.scs2.session.mcap;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
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

      for (Record record : records)
      {
         long lenBody = record.lenBody();
         Opcode op = record.op();

         switch (op)
         {
            case HEADER:
            {
               Header header = (Header) record.body();
               printer.println("------------------- Header -------------------");
               printer.println("profile = " + header.profile().str());
               printer.println("library = " + header.library().str());
               break;
            }
            case ATTACHMENT:
            {
               if (attachmentCount == 0)
               {
                  Attachment attachment = (Attachment) record.body();
                  printer.println("------------------- Attachment -------------------");
                  printer.println("log_time = " + attachment.logTime());
                  printer.println("create_time = " + attachment.createTime());
                  printer.println("name = " + attachment.name().str());
                  printer.println("content_type = " + attachment.mediaType().str()); // MIME Type
                  printer.println("data = " + new String(attachment.data(), StandardCharsets.UTF_8));
                  printer.println("crc = " + attachment.crc32());
                  printer.println("");
               }

               attachmentCount++;
               break;
            }
            case ATTACHMENT_INDEX:
            {
               AttachmentIndex attachmentIndex = (AttachmentIndex) record.body();
               printer.println("------------------- Attachment Index -------------------");
               printer.println("offset = " + attachmentIndex.ofsAttachment());
               printer.println("length = " + attachmentIndex.lenAttachment());
               printer.println("log_time = " + attachmentIndex.logTime());
               printer.println("create_time = " + attachmentIndex.createTime());
               printer.println("data_size = " + attachmentIndex.dataSize());
               printer.println("name = " + attachmentIndex.name().str());
               printer.println("content_type = " + attachmentIndex.mediaType().str());
               break;
            }
            case METADATA:
            {
               if (metadataCount == 0)
               {

                  Metadata metadata = (Metadata) record.body();
                  MapStrStr metadataMap = metadata.metadata();
                  ArrayList<TupleStrStr> metadataEntries = metadataMap.entries().entries();
                  printer.println("------------------- Metadata -------------------");
                  printer.println("name = " + metadata.name().str());
                  printer.println("number of entries = " + metadataEntries.size());
                  printer.println(EuclidCoreIOTools.getCollectionString("[",
                                                                        "]",
                                                                        ", ",
                                                                        metadata.metadata().entries().entries(),
                                                                        e -> "(key: %s)".formatted(e.key().str())));
               }

               metadataCount++;
               break;
            }
            case METADATA_INDEX:
            {
               MetadataIndex metadataIndex = (MetadataIndex) record.body();
               printer.println("------------------- Metadata Index -------------------");
               printer.println("offset = " + metadataIndex.ofsMetadata());
               printer.println("length = " + metadataIndex.lenMetadata());
               printer.println("name = " + metadataIndex.name().str());
               metadataIndexCount++;
               break;
            }
            case SUMMARY_OFFSET:
            {
               SummaryOffset summaryOffset = (SummaryOffset) record.body();
               printer.println("------------------- Summary Offset -------------------");
               printer.println("group_opcode = " + summaryOffset.groupOpcode());
               printer.println("group_start = " + summaryOffset.ofsGroup());
               printer.println("group_length = " + summaryOffset.lenGroup());

               summaryOffsetCount++;
               break;
            }
            case MESSAGE_INDEX:
            {
               if (messageIndexCount == 0)
               {
                  MessageIndex messageIndex = (MessageIndex) record.body();
                  ArrayList<MessageIndexEntry> messageIndexRecordEntries = messageIndex.records().entries();
                  MessageIndexEntry messageIndexEntry = messageIndexRecordEntries.get(0);

                  printer.println("------------------- Message Index -------------------");
                  printer.println("channelId = " + messageIndex.channelId());
                  printer.println("messageIndexRecordEntries.size() = " + messageIndexRecordEntries.size());
                  printer.println("message Log Time = " + messageIndexEntry.logTime());
                  printer.println("");
               }

               messageIndexCount++;
               break;

            }
            case SCHEMA:
            {
               if (schemaCount == 0)
               {
                  Schema schema = (Schema) record.body();
                  printer.println("------------------- Schema -------------------");
                  printer.println("id = %d, schema = %s, encoding = %s".formatted(schema.id(), schema.name().str(), schema.encoding().str()));
                  printer.println("");
               }

               schemaCount++;
               break;
            }
            case CHANNEL:
            {
               //               if (channelCount == 0)
               {
                  Channel channel = (Channel) record.body();
                  printer.println("------------------- Channel -------------------");
                  String topic = channel.topic().str();
                  String messageEncoding = channel.messageEncoding().str();
                  String metadata = EuclidCoreIOTools.getCollectionString(", ",
                                                                          channel.metadata().entries().entries(),
                                                                          e -> "(%s: %s)".formatted(e.key().str(), e.value().str()))
                                                     .replace("\n", "; ");
                  printer.println("id = %d, schema_id = %d, topic = %s, message_encoding = %s, metadata.size() = [%s]".formatted(channel.id(),
                                                                                                                                 channel.schemaId(),
                                                                                                                                 topic,
                                                                                                                                 messageEncoding,
                                                                                                                                 metadata));
                  printer.println("");
               }

               channelCount++;
               break;
            }
            case CHUNK:
            {
               if (chunkCount == 0)
               {
                  Chunk chunk = (Chunk) record.body();
                  printer.println("------------------- Chunk -------------------");
                  int compressedSize = (int) chunk.lenRecords();
                  if (compressedSize != chunk.lenRecords())
                     LogTools.error("int OVERFLOW!!");
                  int uncompressedSize = (int) chunk.uncompressedSize();
                  if (uncompressedSize != chunk.uncompressedSize())
                     LogTools.error("int OVERFLOW!!");
                  printer.println("messageStartTime = " + chunk.messageStartTime());
                  printer.println("messageEndTime = " + chunk.messageEndTime());
                  printer.println("compression = " + chunk.compression().str());
                  printer.println("compressedSize = " + compressedSize);
                  printer.println("uncompressedSize = " + uncompressedSize);
                  try
                  {
                     byte[] sourceByteArray = (byte[]) chunk.records();
//                     ByteBuffer compressedBuffer = ByteBuffer.wrap(sourceByteArray);
//                     ByteBuffer decompressedBuffer = ByteBuffer.allocate(uncompressedSize);
//                     LZ4SafeDecompressor decompressor = LZ4Factory.safeInstance().safeDecompressor();
//                     System.out.println(getDecompressedLength(compressedBuffer, 0));
//                     decompressor.decompress(compressedBuffer, 0, compressedSize, decompressedBuffer, 0, uncompressedSize);

                     ByteBuf compressedBuffer = Unpooled.copiedBuffer(sourceByteArray);
                     List<ByteBuf> data = new Lz4FrameDecoder().decode(compressedBuffer);
                     System.out.println(data.size());

                  }
                  catch (Exception e)
                  {
                     LogTools.error("compression = " + chunk.compression().str() + ", compressedSize = " + compressedSize + ", uncompressedSize = "
                                    + uncompressedSize);
                     e.printStackTrace();
                     unsuccessfulDecompressedChunks++;
                  }

                  printer.println("");
               }

               chunkCount++;
               break;
            }

            case CHUNK_INDEX:
            {
               if (chunkIndexCount == 0)
               {
                  ChunkIndex chunkIndex = (ChunkIndex) record.body();
                  printer.println("------------------- ChunkIndex -------------------");
                  printer.println("messageStartTime = " + chunkIndex.messageStartTime());
                  printer.println("messageEndTime = " + chunkIndex.messageEndTime());
                  printer.println("compression = " + chunkIndex.compression().str());
                  printer.println("compressed size = " + chunkIndex.compressedSize());
                  printer.println("uncompressed size = " + chunkIndex.uncompressedSize());
                  printer.println("");
               }

               chunkIndexCount++;
               break;

            }
            case DATA_END:
            {
               if (dataEndCount == 0)
               {
                  DataEnd dataEnd = (DataEnd) record.body();
                  printer.println("------------------- DataEnd -------------------");
                  printer.println("dataSectionCrc32 = " + dataEnd.dataSectionCrc32());
                  printer.println("");
               }

               dataEndCount++;
               break;
            }
            default:
            {
               printer.println("------------------- Other -------------------");
               printer.println("Opcode = " + op);
               printer.println("");
               break;
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

   public static int getDecompressedLength(ByteBuffer src, int srcOff)
   {
      System.out.println(Long.toHexString(src.order(ByteOrder.LITTLE_ENDIAN).getInt(0)));
      byte byte0 = src.get(srcOff + 3);
      byte byte1 = src.get(srcOff + 2);
      byte byte2 = src.get(srcOff + 1);
      byte byte3 = src.get(srcOff + 0);
      return (byte0 & 0xFF) | (byte1 & 0xFF) << 8 | (byte2 & 0xFF) << 16 | byte3 << 24;
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

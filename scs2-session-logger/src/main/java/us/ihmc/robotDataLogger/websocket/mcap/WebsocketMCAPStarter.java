package us.ihmc.robotDataLogger.websocket.mcap;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.server.WebsocketAnnouncementMetadata;
import us.ihmc.robotDataLogger.websocket.server.WebsocketChannelStarterChunk;
import us.ihmc.robotDataLogger.websocket.server.WebsocketSchemaStarterChunk;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.DataEnd;
import us.ihmc.scs2.session.mcap.specs.records.Footer;
import us.ihmc.scs2.session.mcap.specs.records.Header;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.MCAPElement;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.RecordDataInputBacked;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;

public class WebsocketMCAPStarter
{
   public static final String MCAP_PROFILE = "us.ihmc.mcap-starter";
   public static final String MCAP_VERSION = "version 1.0";

   private final Header header;
   private final WebsocketAnnouncementMetadata announcementMetadata;
   private final WebsocketSchemaStarterChunk schemaStarterChunk;
   private final WebsocketChannelStarterChunk channelStarterChunk;
   private WebsocketRobotModelAttachment robotModelAttachment;
   private WebsocketResourcesAttachment resourcesAttachment;
   private final DataEnd dataEnd;
   private final Footer footer;

   private final Record headerRecord;
   private final Record announcementMetadataRecord;
   private final Record schemaStarterChunkRecord;
   private final Record channelStarterChunkRecord;
   private Record robotModelAttachmentRecord;
   private Record resourcesAttachmentRecord;
   private final Record dataEndRecord;
   private final Record footerRecord;

   private final List<Record> records = new ArrayList<>();

   public WebsocketMCAPStarter(String serverName,
                               String hostName,
                               LogModelProvider logModelProvider,
                               DataServerSettings serverSettings,
                               MCAPBuilder mcapBuilder)
   {
      this.header = new Header(MCAP_PROFILE, MCAP_VERSION);
      this.announcementMetadata = WebsocketAnnouncementMetadata.create(serverName, MCAP_VERSION, hostName, logModelProvider, serverSettings);
      this.schemaStarterChunk = WebsocketSchemaStarterChunk.create(mcapBuilder);
      this.channelStarterChunk = WebsocketChannelStarterChunk.create(mcapBuilder);
      this.robotModelAttachment = WebsocketRobotModelAttachment.create(logModelProvider);
      this.dataEnd = new DataEnd(0);
      this.footer = new Footer();

      headerRecord = new MutableRecord(header);
      announcementMetadataRecord = new MutableRecord(announcementMetadata);
      schemaStarterChunkRecord = new MutableRecord(schemaStarterChunk);
      channelStarterChunkRecord = new MutableRecord(channelStarterChunk);
      if (robotModelAttachment != null)
         robotModelAttachmentRecord = new MutableRecord(robotModelAttachment);
      dataEndRecord = new MutableRecord(dataEnd);
      footerRecord = new MutableRecord(footer);

      records.add(headerRecord);
      records.add(announcementMetadataRecord);
      records.add(schemaStarterChunkRecord);
      records.add(channelStarterChunkRecord);
      if (robotModelAttachment != null)
         records.add(robotModelAttachmentRecord);
      records.add(dataEndRecord);
      records.add(footerRecord);
   }

   public WebsocketMCAPStarter(MCAPDataInput dataInput)
   {
      long currentPos = 0;
      Magic.readMagic(dataInput, currentPos);
      currentPos += Magic.MAGIC_SIZE;
      Record lastRecord;

      Header header = null;
      WebsocketAnnouncementMetadata announcementMetadata = null;
      WebsocketSchemaStarterChunk schemaStarterChunk = null;
      WebsocketChannelStarterChunk channelStarterChunk = null;
      WebsocketRobotModelAttachment robotModelAttachment = null;
      DataEnd dataEnd = null;
      Footer footer = null;

      do
      {
         lastRecord = new RecordDataInputBacked(dataInput, currentPos);
         if (lastRecord.getElementLength() < 0)
            throw new IllegalArgumentException("Invalid record length: " + lastRecord.getElementLength());
         currentPos += lastRecord.getElementLength();

         switch (lastRecord.op())
         {
            case HEADER:
            {
               header = lastRecord.body();
               break;
            }
            case DATA_END:
            {
               dataEnd = lastRecord.body();
               break;
            }
            case FOOTER:
            {
               footer = lastRecord.body();
               break;
            }
            case METADATA:
            {
               if (announcementMetadata == null)
               {
                  announcementMetadata = WebsocketAnnouncementMetadata.toWebsocketAnnouncementMetadata(lastRecord.body());
                  lastRecord = new MutableRecord(announcementMetadata);
               }
               break;
            }
            case CHUNK:
            {
               if (schemaStarterChunk == null)
               {
                  schemaStarterChunk = WebsocketSchemaStarterChunk.toWebsocketSchemaStarterChunk(lastRecord.body());
                  if (schemaStarterChunk != null)
                     lastRecord = new MutableRecord(schemaStarterChunk);
               }

               if (channelStarterChunk == null)
               {
                  channelStarterChunk = WebsocketChannelStarterChunk.toWebsocketChannelStarterChunk(lastRecord.body());
                  if (channelStarterChunk != null)
                     lastRecord = new MutableRecord(channelStarterChunk);
               }
               break;
            }
            case ATTACHMENT:
            {
               if (robotModelAttachment == null)
               {
                  robotModelAttachment = WebsocketRobotModelAttachment.toWebsocketRobotModelAttachment(lastRecord.body());
                  if (robotModelAttachment != null)
                     lastRecord = new MutableRecord(robotModelAttachment);
               }

               if (resourcesAttachment == null)
               {
                  resourcesAttachment = WebsocketResourcesAttachment.toWebsocketResourcesAttachment(lastRecord.body());
                  if (resourcesAttachment != null)
                     lastRecord = new MutableRecord(resourcesAttachment);
               }
               break;
            }
         }
         records.add(lastRecord);
      }
      while (!(lastRecord.op() == Opcode.FOOTER));

      Magic.readMagic(dataInput, currentPos);

      this.header = Objects.requireNonNull(header, "No header found");
      this.announcementMetadata = Objects.requireNonNull(announcementMetadata, "No announcement metadata found");
      this.schemaStarterChunk = Objects.requireNonNull(schemaStarterChunk, "No schema starter chunk found");
      this.channelStarterChunk = Objects.requireNonNull(channelStarterChunk, "No channel starter chunk found");
      this.robotModelAttachment = robotModelAttachment;
      this.dataEnd = Objects.requireNonNull(dataEnd, "No data end found");
      this.footer = Objects.requireNonNull(footer, "No footer found");

      headerRecord = findFirst(record -> record.body() == this.header);
      announcementMetadataRecord = findFirst(record -> record.body() == this.announcementMetadata);
      schemaStarterChunkRecord = findFirst(record -> record.body() == this.schemaStarterChunk);
      channelStarterChunkRecord = findFirst(record -> record.body() == this.channelStarterChunk);
      if (this.robotModelAttachment != null)
         robotModelAttachmentRecord = findFirst(record -> record.body() == this.robotModelAttachment);
      if (this.resourcesAttachment != null)
         resourcesAttachmentRecord = findFirst(record -> record.body() == this.resourcesAttachment);
      dataEndRecord = findFirst(record -> record.body() == this.dataEnd);
      footerRecord = findFirst(record -> record.body() == this.footer);
   }

   private Record findFirst(Predicate<Record> predicate)
   {
      return records.stream().filter(predicate).findFirst().orElse(null);
   }

   public void write(MCAPDataOutput output)
   {
      Magic.INSTANCE.write(output);
      records.forEach(record -> record.write(output));
      Magic.INSTANCE.write(output);
   }

   public void setRobotModelAttachment(WebsocketRobotModelAttachment robotModelAttachment)
   {
      if (this.robotModelAttachment != null)
         throw new IllegalStateException("Robot model attachment already set");
      this.robotModelAttachment = robotModelAttachment;
      this.robotModelAttachmentRecord = new MutableRecord(robotModelAttachment);
      records.add(records.indexOf(dataEndRecord), robotModelAttachmentRecord);
   }

   public void setResourcesAttachment(WebsocketResourcesAttachment resourcesAttachment)
   {
      if (this.resourcesAttachment != null)
         throw new IllegalStateException("Resources attachment already set");
      this.resourcesAttachment = resourcesAttachment;
      this.resourcesAttachmentRecord = new MutableRecord(resourcesAttachment);
      records.add(records.indexOf(dataEndRecord), resourcesAttachmentRecord);
   }

   public List<Record> records()
   {
      return records;
   }

   public Header header()
   {
      return header;
   }

   public WebsocketAnnouncementMetadata announcementMetadata()
   {
      return announcementMetadata;
   }

   public WebsocketSchemaStarterChunk schemaStarterChunk()
   {
      return schemaStarterChunk;
   }

   public WebsocketChannelStarterChunk channelStarterChunk()
   {
      return channelStarterChunk;
   }

   public DataEnd dataEnd()
   {
      return dataEnd;
   }

   public Footer footer()
   {
      return footer;
   }

   public boolean isSessionCompatible(WebsocketMCAPStarter other)
   {
      // We'll just compare: the version, the CRC32 of the schemas, channels, and the model.
      if (!Objects.equals(header, other.header))
         return false;

      if (!Objects.equals(announcementMetadata, other.announcementMetadata))
         return false;

      if (schemaStarterChunk.uncompressedCRC32() != other.schemaStarterChunk.uncompressedCRC32())
         return false;

      if (channelStarterChunk.uncompressedCRC32() != other.channelStarterChunk.uncompressedCRC32())
         return false;

      if (robotModelAttachment == null && other.robotModelAttachment == null)
         return true;
      if (robotModelAttachment == null || other.robotModelAttachment == null)
         return false;
      if (robotModelAttachment.crc32() != other.robotModelAttachment.crc32())
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "WebsocketMCAPStarter{" + "\nheader=\n" + toString(header, 1) + ",\nannouncementMetadata=\n" + toString(announcementMetadata, 1)
             + ",\nschemaStarterChunk=\n" + toString(schemaStarterChunk, 1) + ",\nchannelStarterChunk=\n" + toString(channelStarterChunk, 1)
             + ",\nrobotModelAttachment=\n" + toString(robotModelAttachment, 1) + ",\nresourcesAttachment=\n" + toString(resourcesAttachment, 1)
             + ",\ndataEnd=\n" + toString(dataEnd, 1) + ",\nfooter=\n" + toString(footer, 1) + "\n}";
   }

   private static String toString(MCAPElement element, int indent)
   {
      return element == null ? "null" : element.toString(indent);
   }
}

package us.ihmc.robotDataLogger.websocket.server;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPRegistrySendBufferBuilder;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.specs.records.Attachment;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Compression;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Metadata;
import us.ihmc.scs2.session.mcap.specs.records.MutableAttachment;
import us.ihmc.scs2.session.mcap.specs.records.MutableChunk;
import us.ihmc.scs2.session.mcap.specs.records.MutableMetadata;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.StringPair;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;

public class MCAPDataServerServerContent
{
   private final Record announcementMetadataRecord;
   private final Record modelAttachmentRecord;
   private final Record resourcesAttachmentRecord;
   private final Record variableSchemasRecord;
   private final Record variableChannelsRecord;

   public MCAPDataServerServerContent(String name,
                                      MCAPBuilder mcapBuilder,
                                      LogModelProvider logModelProvider,
                                      DataServerSettings dataServerSettings,
                                      List<MCAPRegistrySendBufferBuilder> registeredBuffers)
   {

      announcementMetadataRecord = new MutableRecord(createAnnouncement(name, dataServerSettings.isLogSession()));
      modelAttachmentRecord = new MutableRecord(createModel(logModelProvider));
      resourcesAttachmentRecord = new MutableRecord(createResources(logModelProvider));
      variableSchemasRecord = new MutableRecord(createVariableSchemas(mcapBuilder));
      variableChannelsRecord = new MutableRecord(createVariableChannels(mcapBuilder, variables));
   }

   private static Chunk createVariableSchemas(MCAPBuilder mcapBuilder)
   {
      MutableChunk chunk = new MutableChunk();
      List<Record> records = new ArrayList<>();
      records.add(mcapBuilder.getVariableSchemaRecord(YoBoolean.class));
      records.add(mcapBuilder.getVariableSchemaRecord(YoDouble.class));
      records.add(mcapBuilder.getVariableSchemaRecord(YoInteger.class));
      records.add(mcapBuilder.getVariableSchemaRecord(YoLong.class));
      records.add(mcapBuilder.getVariableSchemaRecord(YoEnum.class));
      chunk.setCompression(Compression.NONE);
      chunk.setRecords(records);
      return chunk;
   }

   private static Chunk createVariableChannels(MCAPBuilder mcapBuilder, List<YoVariable> variables)
   {
      MutableChunk chunk = new MutableChunk();
      chunk.setRecords(new ArrayList<>());
      chunk.setCompression(Compression.NONE);
      addVariableChannels(mcapBuilder, variables, chunk);
      return chunk;
   }

   private static void addVariableChannels(MCAPBuilder mcapBuilder, List<YoVariable> variables, MutableChunk chunk)
   {
      for (int i = 0; i < variables.size(); i++)
      {
         chunk.records().add(mcapBuilder.getOrCreateVariableChannelRecord(variables.get(i)));
      }
   }

   private static Metadata createAnnouncement(String name, boolean logSession)
   {
      try
      {
         MutableMetadata metadata = new MutableMetadata();
         metadata.setName("announcement");
         List<StringPair> metadataList = new ArrayList<>();
         metadataList.add(new StringPair("name", name));
         metadataList.add(new StringPair("logSession", Boolean.toString(logSession)));
         metadataList.add(new StringPair("hostName", InetAddress.getLocalHost().getHostName()));
         metadataList.add(new StringPair("sessionKey", "n/a")); // FIXME the key should be computed from the set of variables.
         metadata.setMetadata(metadataList);
         return metadata;
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static Attachment createModel(LogModelProvider logModelProvider)
   {
      MutableAttachment attachment = new MutableAttachment();
      attachment.setName(logModelProvider.getModelName());
      if (logModelProvider.getLoader() == null || logModelProvider.getLoader().getSimpleName().toLowerCase().contains("urdf"))
         attachment.setMediaType("model/urdf");
      else
         attachment.setMediaType("model/sdf");

      attachment.setDataLength(logModelProvider.getModel().length);
      attachment.setData(logModelProvider.getModel());

      computeAttachmentCRC32(attachment);

      return attachment;
   }

   private static Attachment createResources(LogModelProvider logModelProvider)
   {
      MutableAttachment attachment = new MutableAttachment();
      attachment.setName(logModelProvider.getModelName() + "-resources.zip");
      attachment.setMediaType("application/zip");
      attachment.setDataLength(logModelProvider.getResourceZip().length);
      attachment.setData(logModelProvider.getResourceZip());

      computeAttachmentCRC32(attachment);

      return attachment;
   }

   private static void computeAttachmentCRC32(MutableAttachment attachment)
   {
      MCAPCRC32Helper crc32Helper = new MCAPCRC32Helper();
      crc32Helper.addLong(attachment.logTime());
      crc32Helper.addLong(attachment.createTime());
      crc32Helper.addString(attachment.name());
      crc32Helper.addString(attachment.mediaType());
      crc32Helper.addLong(attachment.dataLength());
      crc32Helper.addBytes(attachment.data());
      attachment.setCRC32(crc32Helper.getValue());
   }
}

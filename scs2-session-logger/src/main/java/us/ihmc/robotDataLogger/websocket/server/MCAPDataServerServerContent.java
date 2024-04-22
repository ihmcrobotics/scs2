package us.ihmc.robotDataLogger.websocket.server;

import io.netty.buffer.ByteBuf;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPRegistrySendBufferBuilder;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.output.MCAPNettyByteBufDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.Attachment;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Compression;
import us.ihmc.scs2.session.mcap.specs.records.DataEnd;
import us.ihmc.scs2.session.mcap.specs.records.Footer;
import us.ihmc.scs2.session.mcap.specs.records.Header;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.MCAPElement;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.Metadata;
import us.ihmc.scs2.session.mcap.specs.records.MetadataMap;
import us.ihmc.scs2.session.mcap.specs.records.MutableAttachment;
import us.ihmc.scs2.session.mcap.specs.records.MutableChunk;
import us.ihmc.scs2.session.mcap.specs.records.MutableMetadata;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.scs2.session.mcap.specs.records.Record;
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
   // TODO Extract out of here
   public static final String MCAP_PROFILE = "us.ihmc.mcap-starter";
   public static final String MCAP_LIBRARY = "version 1.0";

   public static final String MCAP_STARTER = "/mcap_starter.mcap";
   public static final String ROBOT_MODEL_RESOURCES = "/robot_model_resources.zip";
   public static final String ANNOUNCEMENT_METADATA_NAME = "announcement";
   public static final String MODEL_URDF = "model/urdf";
   public static final String MODEL_SDF = "model/sdf";

   private final ByteBuf mcapStarterBuffer;
   private final ByteBuf robotModelResourcesBuffer;

   public MCAPDataServerServerContent(String name,
                                      MCAPBuilder mcapBuilder,
                                      LogModelProvider logModelProvider,
                                      DataServerSettings dataServerSettings,
                                      List<MCAPRegistrySendBufferBuilder> registeredBuffers)
   {
      Attachment modelAttachment = createModel(logModelProvider);

      Record variableChannelsRecord = new MutableRecord(createVariableChannels());
      for (MCAPRegistrySendBufferBuilder buffer : registeredBuffers)
      {
         MutableChunk chunk = variableChannelsRecord.body();
         addVariableChannels(mcapBuilder, buffer.getYoRegistry().collectSubtreeVariables(), chunk);
      }

      List<MCAPElement> mcap = new ArrayList<>();
      mcap.add(Magic.INSTANCE);
      mcap.add(new MutableRecord(new Header(MCAP_PROFILE, MCAP_LIBRARY)));
      mcap.add(new MutableRecord(createAnnouncement(name, dataServerSettings.isLogSession())));
      mcap.add(new MutableRecord(createVariableSchemas(mcapBuilder)));
      mcap.add(variableChannelsRecord);
      if (modelAttachment != null)
         mcap.add(new MutableRecord(modelAttachment));
      MCAPCRC32Helper crc32Helper = new MCAPCRC32Helper();
      mcap.forEach(element -> element.updateCRC(crc32Helper));
      mcap.add(new MutableRecord(new DataEnd(crc32Helper.getValue())));
      mcap.add(new MutableRecord(new Footer()));
      mcap.add(Magic.INSTANCE);
      MCAPNettyByteBufDataOutput output = new MCAPNettyByteBufDataOutput(false);
      for (MCAPElement element : mcap)
      {
         element.write(output);
      }
      mcapStarterBuffer = output.getBuffer();

      // Separate resources as they can be quite large
      Attachment resourcesAttachment = createResources(logModelProvider);

      if (resourcesAttachment != null)
      {
         MCAPNettyByteBufDataOutput resourcesOutput = new MCAPNettyByteBufDataOutput(false);
         resourcesAttachment.write(resourcesOutput);
         robotModelResourcesBuffer = resourcesOutput.getBuffer();
      }
      else
      {
         robotModelResourcesBuffer = null;
      }
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

   private static Chunk createVariableChannels()
   {
      MutableChunk chunk = new MutableChunk();
      chunk.setRecords(new ArrayList<>());
      chunk.setCompression(Compression.NONE);
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
      MutableMetadata metadata = new MutableMetadata();
      metadata.setName(ANNOUNCEMENT_METADATA_NAME);
      MetadataMap metadataMap = new MetadataMap();
      metadataMap.put("name", name);
      metadataMap.put("logSession", Boolean.toString(logSession));
      metadataMap.put("hostName", getHostName());
      metadata.setMetadata(metadataMap);
      return metadata;
   }

   private static String getHostName()
   {
      try
      {
         return InetAddress.getLocalHost().getHostName();
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static Attachment createModel(LogModelProvider logModelProvider)
   {
      if (logModelProvider == null || logModelProvider.getModel() == null || logModelProvider.getModel().length == 0)
         return null;

      MutableAttachment attachment = new MutableAttachment();
      attachment.setName(logModelProvider.getModelName());
      if (logModelProvider.getLoader() == null || logModelProvider.getLoader().getSimpleName().toLowerCase().contains("urdf"))
         attachment.setMediaType(MODEL_URDF);
      else
         attachment.setMediaType(MODEL_SDF);

      attachment.setDataLength(logModelProvider.getModel().length);
      attachment.setData(logModelProvider.getModel());
      attachment.setCRC32(attachment.updateCRC(null).getValue());

      return attachment;
   }

   private static Attachment createResources(LogModelProvider logModelProvider)
   {
      if (logModelProvider == null || logModelProvider.getResourceZip() == null || logModelProvider.getResourceZip().length == 0)
         return null;

      MutableAttachment attachment = new MutableAttachment();
      attachment.setName(logModelProvider.getModelName() + "-resources.zip");
      attachment.setMediaType("application/zip");
      attachment.setDataLength(logModelProvider.getResourceZip().length);
      attachment.setData(logModelProvider.getResourceZip());
      attachment.setCRC32(attachment.updateCRC(null).getValue());

      return attachment;
   }

   public ByteBuf getMCAPStarterBuffer()
   {
      return mcapStarterBuffer.retainedDuplicate();
   }

   public ByteBuf getRobotModelResourcesBuffer()
   {
      return robotModelResourcesBuffer == null ? null : robotModelResourcesBuffer.retainedDuplicate();
   }
}

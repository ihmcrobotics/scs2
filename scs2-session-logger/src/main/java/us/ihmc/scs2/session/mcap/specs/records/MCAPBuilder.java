package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.CDRSerializer;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;

/**
 * The plan for this class is gather the tools for building MCAP data here.
 */
public class MCAPBuilder
{
   public static final String MESSAGE_ENCODING = "cdr";
   private final List<SchemaInfoPackage> schemas = new ArrayList<>();
   private final Map<Class<? extends YoVariable>, SchemaInfoPackage> variableToSchemaMap = new LinkedHashMap<>();

   private final List<ChannelInfoPackage> channels = new ArrayList<>();
   private final Map<YoVariable, ChannelInfoPackage> variableToChannelMap = new LinkedHashMap<>();

   private final CDRSerializer cdrSerializer = new CDRSerializer();
   private Consumer<Channel> newChannelListener;

   public MCAPBuilder()
   {
      for (Class<? extends YoVariable> variableType : Arrays.asList(YoBoolean.class, YoDouble.class, YoLong.class, YoInteger.class, YoEnum.class))
      {
         MutableSchema newSchema = nextSchema(variableType.getSimpleName(), schemas.size(), "ros2msg", loadYoVariableSchemaBytesFromFile(variableType));
         SchemaInfoPackage schemaInfoPackage = new SchemaInfoPackage(newSchema);
         schemas.add(schemaInfoPackage);
         variableToSchemaMap.put(variableType, schemaInfoPackage);
      }
   }

   public void setChannelCreationListener(Consumer<Channel> newChannelListener)
   {

      this.newChannelListener = newChannelListener;
   }

   /**
    * Load the schema file for the given variable type. The schema is stored in memory as a byte array instead of a string for efficiency.
    * <p
    * The file encoding should be UTF-8. The resulting array is prefixed with the length of the schema file as a 4-byte integer in little-endian order.
    * </p>
    *
    * @param variableType the type of the variable.
    * @return the schema file as a byte array.
    */
   private static byte[] loadYoVariableSchemaBytesFromFile(Class<? extends YoVariable> variableType)
   {
      ClassLoader classLoader = MCAPBuilder.class.getClassLoader();
      try
      {
         try (InputStream schemaStream = classLoader.getResourceAsStream("mcap/schema/%sSchema.ros2msg".formatted(variableType.getSimpleName())))
         {
            Objects.requireNonNull(schemaStream, "Could not find schema file for variable: " + variableType.getSimpleName());
            byte[] schemaBytes = schemaStream.readAllBytes();
            ByteBuffer schemaBytesWithPrefixLength = ByteBuffer.allocate(schemaBytes.length + 4).order(ByteOrder.LITTLE_ENDIAN);
            schemaBytesWithPrefixLength.putInt(schemaBytes.length);
            schemaBytesWithPrefixLength.put(schemaBytes);
            return schemaBytesWithPrefixLength.array();
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static MutableSchema nextSchema(String name, int schemaID, String encoding, byte[] data)
   {
      MutableSchema schema = new MutableSchema();
      schema.setId(schemaID);
      schema.setName(name);
      schema.setEncoding(encoding);
      schema.setData(data);
      return schema;
   }

   public Schema getSchema(int id)
   {
      return schemas.get(id).schema;
   }

   public Record getSchemaRecord(int id)
   {
      return schemas.get(id).schemaRecord;
   }

   public Record getVariableSchemaRecord(Class<? extends YoVariable> variableType)
   {
      return variableToSchemaMap.get(variableType).schemaRecord;
   }

   public Record getOrCreateVariableChannelRecord(YoVariable variable)
   {
      return getOrCreateChannel(variable).channelRecord;
   }

   private ChannelInfoPackage getOrCreateChannel(YoVariable variable)
   {
      ChannelInfoPackage channelInfoPackage = variableToChannelMap.get(variable);
      if (channelInfoPackage == null)
      {
         String topic = variable.getFullNameString().replace(YoTools.NAMESPACE_SEPERATOR, '/');
         int schemaID = variableToSchemaMap.get(variable.getClass()).schema.id();
         Channel channel = nextChannel(topic, schemaID, channels.size(), MESSAGE_ENCODING, new MetadataMap());
         channelInfoPackage = new ChannelInfoPackage(channel, variable, variableToSchemaMap.get(variable.getClass()).schema);
         channels.add(channelInfoPackage);
         variableToChannelMap.put(variable, channelInfoPackage);

         if (newChannelListener != null)
            newChannelListener.accept(channel);
      }

      return channelInfoPackage;
   }

   private static MutableChannel nextChannel(String topic, int schemaID, int channelID, String messageEncoding, MetadataMap metadata)
   {
      MutableChannel channel = new MutableChannel();
      channel.setId(channelID);
      channel.setTopic(topic);
      channel.setSchemaId(schemaID);
      channel.setMessageEncoding(messageEncoding);
      channel.setMetadata(metadata);
      return channel;
   }

   public void packVariableMessage(YoVariable variable, MutableMessage message)
   {
      ChannelInfoPackage channel = getOrCreateChannel(variable);
      message.setChannelId(channel.channel.id());

      if (message.messageData() == null)
      {
         message.setMessageData(new byte[12]);
      }
      ByteBuffer messageBuffer = message.messageBuffer();
      messageBuffer.clear();
      messageBuffer.order(ByteOrder.LITTLE_ENDIAN);
      cdrSerializer.initialize(messageBuffer);
      if (variable instanceof YoBoolean yoBoolean)
         cdrSerializer.write_byte((byte) (yoBoolean.getBooleanValue() ? 1 : 0));
      else if (variable instanceof YoDouble yoDouble)
         cdrSerializer.write_float64(yoDouble.getDoubleValue());
      else if (variable instanceof YoLong yoLong)
         cdrSerializer.write_int64(yoLong.getLongValue());
      else if (variable instanceof YoInteger yoInteger)
         cdrSerializer.write_int32(yoInteger.getIntegerValue());
      else if (variable instanceof YoEnum<?> yoEnum)
         cdrSerializer.write_uint8(yoEnum.getOrdinal());
      else
         throw new IllegalArgumentException("Unsupported variable type: " + variable.getClass().getSimpleName());
      messageBuffer.flip();
      message.setDataLength(messageBuffer.remaining());
   }

   private static class SchemaInfoPackage
   {
      private final MutableSchema schema;
      private final Record schemaRecord;

      public SchemaInfoPackage(MutableSchema schema)
      {
         this.schema = schema;
         this.schemaRecord = new MutableRecord(schema);
      }
   }

   private static class ChannelInfoPackage
   {
      private final Channel channel;
      private final YoVariable variable;
      private final Schema schema;

      private final Record channelRecord;

      public ChannelInfoPackage(Channel channel, YoVariable variable, MutableSchema schema)
      {
         this.channel = channel;
         this.variable = variable;
         this.schema = schema;
         this.channelRecord = new MutableRecord(channel);
      }
   }
}

package us.ihmc.scs2.session.mcap.specs.records;

import gnu.trove.map.hash.TIntObjectHashMap;
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
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * The plan for this class is gather the tools for building MCAP data here.
 */
public class MCAPBuilder
{
   private int nextSchemaID = 0;
   private final TIntObjectHashMap<MutableSchema> schemas = new TIntObjectHashMap<>();
   private final Map<Class<? extends YoVariable>, MutableSchema> variableSchemas = new LinkedHashMap<>();

   private final TIntObjectHashMap<Record> schemaRecords = new TIntObjectHashMap<>();
   private final Map<Class<? extends YoVariable>, Record> variableSchemaRecordMap = new LinkedHashMap<>();

   private int nextChannelID = 0;
   private final TIntObjectHashMap<MutableChannel> channels = new TIntObjectHashMap<>();
   private final Map<YoVariable, MutableChannel> variableChannels = new LinkedHashMap<>();

   private final TIntObjectHashMap<Record> channelRecords = new TIntObjectHashMap<>();
   private final Map<YoVariable, Record> variableChannelRecordMap = new LinkedHashMap<>();

   public MCAPBuilder()
   {
      for (Class<? extends YoVariable> variableType : Arrays.asList(YoBoolean.class, YoDouble.class, YoLong.class, YoInteger.class, YoEnum.class))
      {
         MutableSchema newSchema = nextSchema(variableType.getSimpleName(), "ros2msg", loadYoVariableSchemaBytesFromFile(variableType));
         variableSchemas.put(variableType, newSchema);
         variableSchemaRecordMap.put(variableType, schemaRecords.get(newSchema.id()));
      }
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

   private MutableSchema nextSchema(String name, String encoding, byte[] data)
   {
      MutableSchema schema = nextSchema();
      schema.setName(name);
      schema.setEncoding(encoding);
      schema.setData(data);
      return schema;
   }

   private MutableSchema nextSchema()
   {
      MutableSchema schema = new MutableSchema();
      schema.setId(nextSchemaID++);
      schemas.put(schema.id(), schema);
      schemaRecords.put(schema.id(), new MutableRecord(schema));
      return schema;
   }

   public MutableSchema getSchema(int id)
   {
      return schemas.get(id);
   }

   public Record getSchemaRecord(int id)
   {
      return schemaRecords.get(id);
   }

   public Record getVariableSchemaRecord(Class<? extends YoVariable> variableType)
   {
      return variableSchemaRecordMap.get(variableType);
   }

   public Record getOrCreateVariableChannelRecord(YoVariable variable)
   {
      MutableChannel channel = getOrCreateChannel(variable);
      return channelRecords.get(channel.id());
   }

   private MutableChannel getOrCreateChannel(YoVariable variable)
   {
      MutableChannel channel = variableChannels.get(variable);
      if (channel == null)
      {
         String topic = variable.getFullNameString().replace(YoTools.NAMESPACE_SEPERATOR, '/');
         int schemaID = variableSchemas.get(variable.getClass()).id();
         channel = nextChannel(topic, schemaID, "cdr", Collections.emptyList());
         variableChannels.put(variable, channel);
      }

      return channel;
   }

   private MutableChannel nextChannel(String topic, int schemaID, String messageEncoding, List<StringPair> metadata)
   {
      MutableChannel channel = nextChannel();
      channel.setTopic(topic);
      channel.setSchemaId(schemaID);
      channel.setMessageEncoding(messageEncoding);
      channel.setMetadata(metadata);
      return channel;
   }

   private MutableChannel nextChannel()
   {
      MutableChannel channel = new MutableChannel();
      channel.setId(nextChannelID++);
      channels.put(channel.id(), channel);
      channelRecords.put(channel.id(), new MutableRecord(channel));
      return channel;
   }

   public void packVariableMessage(YoVariable variable, MutableMessage message)
   {
      MutableChannel channel = getOrCreateChannel(variable);
      message.setChannelId(channel.id());

      if (message.messageData() == null)
      {
         message.setMessageData(new byte[8]);
      }
      ByteBuffer messageBuffer = message.messageBuffer();
      messageBuffer.clear();
      if (variable instanceof YoBoolean yoBoolean)
         messageBuffer.put((byte) (yoBoolean.getBooleanValue() ? 1 : 0));
      else if (variable instanceof YoDouble yoDouble)
         messageBuffer.putDouble(yoDouble.getDoubleValue());
      else if (variable instanceof YoLong yoLong)
         messageBuffer.putLong(yoLong.getLongValue());
      else if (variable instanceof YoInteger yoInteger)
         messageBuffer.putInt(yoInteger.getIntegerValue());
      else if (variable instanceof YoEnum<?> yoEnum)
         messageBuffer.put((byte) yoEnum.getOrdinal());
      else
         throw new IllegalArgumentException("Unsupported variable type: " + variable.getClass().getSimpleName());
      messageBuffer.flip();
      message.setDataLength(messageBuffer.remaining());
   }
}

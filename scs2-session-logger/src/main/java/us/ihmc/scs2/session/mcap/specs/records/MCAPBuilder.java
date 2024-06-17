package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.euclid.Axis3D;
import us.ihmc.scs2.session.mcap.encoding.CDRSerializer;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * The plan for this class is gather the tools for building MCAP data here.
 */
public class MCAPBuilder
{
   public static final String MESSAGE_ENCODING = "cdr";
   private final List<SchemaInfoPackage> schemas = new ArrayList<>();
   private final Map<YoVariableTypePackage, SchemaInfoPackage> variableToSchemaMap = new LinkedHashMap<>();

   private final List<ChannelInfoPackage> channels = new ArrayList<>();
   private final Map<YoVariable, ChannelInfoPackage> variableToChannelMap = new LinkedHashMap<>();

   private final CDRSerializer cdrSerializer = new CDRSerializer();
   private Consumer<Schema> newSchemaListener;
   private Consumer<Channel> newChannelListener;

   public MCAPBuilder()
   {
      for (Class<? extends YoVariable> variableType : Arrays.asList(YoBoolean.class, YoDouble.class, YoLong.class, YoInteger.class))
      {
         getOrCreateSchema(YoVariableTypePackage.valueOf(variableType));
      }
   }

   public void setSchemaCreationListener(Consumer<Schema> newSchemaListener)
   {
      this.newSchemaListener = newSchemaListener;
   }

   public void setChannelCreationListener(Consumer<Channel> newChannelListener)
   {
      this.newChannelListener = newChannelListener;
   }

   private MutableSchema nextSchema(String name, String schema)
   {
      return createSchema(name, schemas.size(), "ros2msg", schema.getBytes());
   }

   private static MutableSchema createSchema(String name, int schemaID, String encoding, byte[] data)
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

   public List<Record> getAllSchemas()
   {
      return schemas.stream().map(schemaInfoPackage -> schemaInfoPackage.schemaRecord).toList();
   }

   private SchemaInfoPackage getOrCreateSchema(YoVariableTypePackage variableTypePackage)
   {
      SchemaInfoPackage schemaInfoPackage = variableToSchemaMap.get(variableTypePackage);
      if (schemaInfoPackage == null)
      {
         schemaInfoPackage = new SchemaInfoPackage(nextSchema(variableTypePackage.getSchemaName(), variableTypePackage.getSchema()));
         schemas.add(schemaInfoPackage);
         variableToSchemaMap.put(variableTypePackage, schemaInfoPackage);

         if (newSchemaListener != null)
            newSchemaListener.accept(schemaInfoPackage.schema);
      }

      return schemaInfoPackage;
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
         SchemaInfoPackage schemaInfoPackage = getOrCreateSchema(YoVariableTypePackage.valueOf(variable));
         String topic = variable.getFullNameString().replace(YoTools.NAMESPACE_SEPERATOR, '/');
         int schemaID = schemaInfoPackage.schema.id();
         Channel channel = nextChannel(topic, schemaID, channels.size(), MESSAGE_ENCODING, new MetadataMap());
         channelInfoPackage = new ChannelInfoPackage(channel, variable, schemaInfoPackage.schema);
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

   public List<Record> getAllChannels()
   {
      return channels.stream().map(channelInfoPackage -> channelInfoPackage.channelRecord).toList();
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
         cdrSerializer.write_bool(yoBoolean.getValue());
      else if (variable instanceof YoDouble yoDouble)
         cdrSerializer.write_float64(yoDouble.getValue());
      else if (variable instanceof YoLong yoLong)
         cdrSerializer.write_int64(yoLong.getValue());
      else if (variable instanceof YoInteger yoInteger)
         cdrSerializer.write_int32(yoInteger.getValue());
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

   public static Class<? extends YoVariable> getVariableTypeFromSchemaName(String schemaName)
   {
      if (schemaName.equals("YoBoolean"))
         return YoBoolean.class;
      if (schemaName.equals("YoDouble"))
         return YoDouble.class;
      if (schemaName.equals("YoInteger"))
         return YoInteger.class;
      if (schemaName.equals("YoLong"))
         return YoLong.class;
      if (schemaName.startsWith("YoEnum"))
         return YoEnum.class;
      return null;
   }

   private static class YoVariableTypePackage
   {
      private static final YoVariableTypePackage BOOLEAN = new YoVariableTypePackage(YoBoolean.class, null, false);
      private static final YoVariableTypePackage DOUBLE = new YoVariableTypePackage(YoDouble.class, null, false);
      private static final YoVariableTypePackage INTEGER = new YoVariableTypePackage(YoInteger.class, null, false);
      private static final YoVariableTypePackage LONG = new YoVariableTypePackage(YoLong.class, null, false);

      private final Class<? extends YoVariable> variableType;
      private final String[] enumValuesAsString;
      private final boolean allowNull;

      public static YoVariableTypePackage valueOf(YoVariable yoVariable)
      {
         if (yoVariable instanceof YoBoolean)
            return BOOLEAN;
         if (yoVariable instanceof YoDouble)
            return DOUBLE;
         if (yoVariable instanceof YoInteger)
            return INTEGER;
         if (yoVariable instanceof YoLong)
            return LONG;
         if (yoVariable instanceof YoEnum<?> yoEnum)
            return new YoVariableTypePackage(yoEnum.getClass(), yoEnum.getEnumValuesAsString(), yoEnum.isNullAllowed());
         return null;
      }

      public static YoVariableTypePackage valueOf(Class<? extends YoVariable> variableType)
      {
         if (variableType == YoBoolean.class)
            return BOOLEAN;
         if (variableType == YoDouble.class)
            return DOUBLE;
         if (variableType == YoInteger.class)
            return INTEGER;
         if (variableType == YoLong.class)
            return LONG;
         return null;
      }

      public YoVariableTypePackage(Class<? extends YoVariable> variableType, String[] enumValuesAsString, boolean allowNull)
      {
         this.variableType = variableType;
         this.enumValuesAsString = enumValuesAsString;
         this.allowNull = allowNull;
      }

      private String getSchemaName()
      {
         if (enumValuesAsString != null)
            return variableType.getSimpleName() + "-" + Arrays.toString(enumValuesAsString);
         else
            return variableType.getSimpleName();
      }

      private String getSchema()
      {
         if (variableType == YoBoolean.class)
            return "bool value";
         else if (variableType == YoDouble.class)
            return "float64 value";
         else if (variableType == YoInteger.class)
            return "int32 value";
         else if (variableType == YoLong.class)
            return "int64 value";
         else if (enumValuesAsString != null)
            return createYoEnumSchema(enumValuesAsString, allowNull);
         else
            throw new IllegalArgumentException("Unsupported variable type: " + variableType.getSimpleName());
      }

      /**
       * Generate the schema for a given enum type.
       * <p>
       * Example with the {@link Axis3D} enum:
       * <pre>
       *    uint8 SIZE=3
       *    uint8 X=0
       *    uint8 Y=1
       *    uint8 Z=2
       *    bool ALLOW_NULL=false
       *    uint8 value
       * </pre>
       * </p>
       *
       * @param enumType the enum type.
       * @return the schema as a string.
       */
      public static String createYoEnumSchema(Class<? extends Enum<?>> enumType, boolean allowNull)
      {
         if (!enumType.isEnum())
            throw new IllegalArgumentException("The provided class is not an enum: " + enumType.getSimpleName());

         return createYoEnumSchema(Arrays.stream(enumType.getEnumConstants()).map(Enum::name).toArray(String[]::new), allowNull);
      }

      public static String createYoEnumSchema(String[] enumConstantsAsStrings, boolean allowNull)
      {
         StringBuilder schema = new StringBuilder();

         schema.append("uint8 SIZE=").append(enumConstantsAsStrings.length).append("\n");
         for (int i = 0; i < enumConstantsAsStrings.length; i++)
         {
            schema.append("uint8 ").append(enumConstantsAsStrings[i]).append("=").append(i).append("\n");
         }
         schema.append("bool ALLOW_NULL=").append(allowNull).append("\n");
         schema.append("uint8 value\n");
         return schema.toString();
      }
   }
}

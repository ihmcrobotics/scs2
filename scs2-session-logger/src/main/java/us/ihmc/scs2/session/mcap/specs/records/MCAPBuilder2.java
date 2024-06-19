package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.encoding.CDRSerializer;
import us.ihmc.yoVariables.registry.YoRegistry;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * The plan for this class is gather the tools for building MCAP data here.
 * <p>
 * Difference with the {@link MCAPBuilder} is that the schemas are built on a per registry basis.
 * </p>
 */
public class MCAPBuilder2
{
   public static final boolean USE_ROS2MSG = false;
   public static final String SCHEMA_ENCODING = USE_ROS2MSG ? "ros2msg" : "omgidl";
   public static final String MESSAGE_ENCODING = "cdr";

   private final List<RegistryEntry> registryEntries = new ArrayList<>();
   private final Map<YoRegistry, RegistryEntry> registryEntryMap = new HashMap<>();

   private int schemaId = 1; // IDs start at 1: https://mcap.dev/spec#schema-op0x03
   private int channelId = 1; // Just to be consistent with the schema IDs

   public MCAPBuilder2()
   {
   }

   public void addRegistryRecursively(YoRegistry registry)
   {
      addRegistry(registry);

      for (int i = 0; i < registry.getChildren().size(); i++)
      {
         addRegistryRecursively(registry.getChildren().get(i));
      }
   }

   public void addRegistry(YoRegistry registry)
   {
      createSchema(registry);
      createChannel(registry);
   }

   private void createSchema(YoRegistry registry)
   {
      if (registryEntryMap.containsKey(registry))
         return;

      MutableSchema schema = new MutableSchema();
      schema.setId(schemaId++);
      schema.setName(registry.getName());
      schema.setEncoding(SCHEMA_ENCODING);
      if (USE_ROS2MSG)
         schema.setData(createRegistryROS2MSG(registry).getBytes());
      else
         schema.setData(createRegistryOMGIDL(registry).getBytes());
      RegistryEntry registryEntry = new RegistryEntry(registry, schema);
      registryEntries.add(registryEntry);
      registryEntryMap.put(registry, registryEntry);
   }

   private void createChannel(YoRegistry registry)
   {
      RegistryEntry registrySchemaEntry = registryEntryMap.get(registry);

      if (registrySchemaEntry == null)
         throw new IllegalStateException("Schema for registry " + registry.getName() + " has not been created.");

      if (registrySchemaEntry.channel != null)
         return;

      MutableChannel channel = new MutableChannel();
      channel.setId(channelId++);
      channel.setTopic(registry.getNamespace().getName());
      channel.setSchemaId(registrySchemaEntry.schema.id());
      channel.setMessageEncoding(MESSAGE_ENCODING);
      channel.setMetadata(new MetadataMap());
      registrySchemaEntry.setChannel(channel);
   }

   public List<Record> getAllSchemas()
   {
      return registryEntries.stream().map(entry -> entry.schemaRecord).toList();
   }

   public List<Record> getAllChannels()
   {
      return registryEntries.stream().map(entry -> entry.channelRecord).toList();
   }

   private final CDRSerializer cdrSerializer = new CDRSerializer();

   public void packRegistryMessage(YoRegistry registry, MutableMessage message)
   {
      RegistryEntry registryEntry = registryEntryMap.get(registry);

      if (registryEntry == null)
         throw new IllegalStateException("Entry for registry " + registry.getName() + " has not been created.");

      if (registryEntry.channel == null)
         throw new IllegalStateException("Channel for registry " + registry.getName() + " has not been created.");

      message.setChannelId(registryEntry.channel.id());

      if (message.messageData() == null)
         message.setMessageData(new byte[registry.getNumberOfVariables() * 8 + CDRDeserializer.encapsulation_size]);
      ByteBuffer messageBuffer = message.messageBuffer();
      messageBuffer.clear();
      messageBuffer.order(ByteOrder.LITTLE_ENDIAN);
      cdrSerializer.initialize(messageBuffer);
      for (int i = 0; i < registry.getNumberOfVariables(); i++)
      {
         writeVariable(cdrSerializer, registry.getVariable(i));
      }
      messageBuffer.flip();
      message.setDataLength(messageBuffer.remaining());
   }

   static void writeVariable(CDRSerializer cdrSerializer, YoVariable variable)
   {
      if (variable instanceof YoBoolean yoBoolean)
         cdrSerializer.write_bool(yoBoolean.getValue());
      else if (variable instanceof YoDouble yoDouble)
         cdrSerializer.write_float64(yoDouble.getValue());
      else if (variable instanceof YoLong yoLong)
         cdrSerializer.write_int64(yoLong.getValue());
      else if (variable instanceof YoInteger yoInteger)
         cdrSerializer.write_int32(yoInteger.getValue());
      else if (variable instanceof YoEnum<?> yoEnum)
         cdrSerializer.write_int32(yoEnum.getOrdinal());
      else
         throw new IllegalArgumentException("Unsupported variable type: " + variable.getClass().getSimpleName());
   }

   private static String createRegistryROS2MSG(YoRegistry registry)
   {
      StringBuilder builder = new StringBuilder();

      for (int i = 0; i < registry.getNumberOfVariables(); i++)
      {
         YoVariable variable = registry.getVariable(i);

         if (variable instanceof YoEnum<?> yoEnum)
         {
            // Need to switch to upper-case to respect ROS2 message naming conventions
            String enumName = variable.getName().toUpperCase();

            if (yoEnum.isNullAllowed())
               builder.append("byte ").append(enumName).append("_NULL=-1\n");

            for (int j = 0; j < yoEnum.getEnumSize(); j++)
            {
               builder.append("byte ");
               builder.append(enumName)
                      .append("_")
                      .append(yoEnum.getEnumValuesAsString()[j].replace(" ", "_").replaceAll(YoTools.ILLEGAL_CHARACTERS_REGEX, ""));
               builder.append("=").append(j).append("\n");
            }
         }

         builder.append(yoVariableROS2MSGType(variable));
         builder.append(" ");
         builder.append(variable.getName());
         builder.append("\n");
      }
      return builder.toString();
   }

   private static String yoVariableROS2MSGType(YoVariable variable)
   {
      if (variable instanceof YoBoolean)
         return "bool";
      if (variable instanceof YoDouble)
         return "float64";
      if (variable instanceof YoLong)
         return "int64";
      if (variable instanceof YoInteger)
         return "int32";
      if (variable instanceof YoEnum)
         return "byte";
      return null;
   }

   private static String createRegistryOMGIDL(YoRegistry registry)
   {
      StringBuilder builder = new StringBuilder();

      // First we need to define the enums
      for (int i = 0; i < registry.getNumberOfVariables(); i++)
      {
         YoVariable variable = registry.getVariable(i);

         if (variable instanceof YoEnum<?> yoEnum)
         {
            builder.append("enum ");
            builder.append(variable.getName()).append("_enum"); // TODO That'd be nice to have the actual enum name
            builder.append("\n");
            builder.append("{\n");

            for (int j = 0; j < yoEnum.getEnumSize(); j++)
               builder.append("   ").append(yoEnum.getEnumValuesAsString()[j]).append(",\n");

            builder.append("};\n");
         }
      }

      builder.append("\n");
      builder.append("struct ").append(registry.getName()).append("\n").append("{\n");

      for (int i = 0; i < registry.getNumberOfVariables(); i++)
      {
         YoVariable variable = registry.getVariable(i);

         if (variable instanceof YoEnum)
         {
            builder.append(variable.getName()).append("_enum"); // TODO That'd be nice to have the actual enum name
            builder.append(" ");
            builder.append(variable.getName());
            builder.append(";\n");
         }
         else
         {
            builder.append(yoVariableOMGIDLType(variable));
            builder.append(" ");
            builder.append(variable.getName());
            builder.append(";\n");
         }
      }

      builder.append("};\n");

      return builder.toString();
   }

   private static String yoVariableOMGIDLType(YoVariable variable)
   {
      if (variable instanceof YoBoolean)
         return "boolean";
      if (variable instanceof YoDouble)
         return "double";
      if (variable instanceof YoLong)
         return "long long";
      if (variable instanceof YoInteger)
         return "long";
      return null;
   }

   private static class RegistryEntry
   {
      private final YoRegistry registry;

      private final MutableSchema schema;
      private final Record schemaRecord;

      private MutableChannel channel;
      private Record channelRecord;

      public RegistryEntry(YoRegistry registry, MutableSchema schema)
      {
         this.registry = registry;
         this.schema = schema;
         this.schemaRecord = new MutableRecord(schema);
      }

      public void setChannel(MutableChannel channel)
      {
         this.channel = channel;
         this.channelRecord = new MutableRecord(channel);
      }

      public YoRegistry getRegistry()
      {
         return registry;
      }
   }
}


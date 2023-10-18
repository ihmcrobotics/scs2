package us.ihmc.scs2.session.mcap;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.*;
import us.ihmc.scs2.session.mcap.ROS2MessageSchema.ROS2Field;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.util.function.Consumer;

public class YoROS2Message
{
   private final YoRegistry registry;
   private final ROS2MessageSchema schema;
   private final int channelId;
   private final List<Consumer<ByteBuffer>> deserializers = new ArrayList<>();

   public YoROS2Message(int channelId, ROS2MessageSchema schema)
   {
      this(schema.getName(), channelId, schema);
   }

   public YoROS2Message(String name, int channelId, ROS2MessageSchema schema)
   {
      this(schema, channelId, new YoRegistry(name));
   }

   public YoROS2Message(ROS2MessageSchema schema, int channelId, YoRegistry registry)
   {
      this.schema = schema;
      this.registry = registry;
      this.channelId = channelId;

      instantiateSchema(schema, schema.getSubSchemaMap(), registry, deserializers);
   }

   public void readMessage(Mcap.Message message)
   {
      if (message.channelId() != channelId)
         throw new IllegalArgumentException("Expected channel ID: " + channelId + ", but received: " + message.channelId());

      ByteBuffer dataBuffer = ByteBuffer.wrap(message.data());
      dataBuffer.order(ByteOrder.LITTLE_ENDIAN);

      for (Object variable : variablesInOrder)
      {
         if (variable instanceof YoBoolean yoBoolean)
         {
            // TODO Not sure how boolean are serialized, no mention of it in the MCAP doc.
            yoBoolean.set(dataBuffer.get() != 0);
         }
         else if (variable instanceof YoDouble)
         {

         }
      }
   }

   public ROS2MessageSchema getSchema()
   {
      return schema;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   private static YoRegistry instantiateSchema(ROS2MessageSchema schema,
                                               Map<String, ROS2MessageSchema> subSchemaMap,
                                               YoRegistry schemaRegistry,
                                               List<Consumer<ByteBuffer>> deserializers)
   {
      Objects.requireNonNull(schema, "Schema cannot be null. name = " + schemaRegistry.getName());

      for (ROS2Field field : schema.getFields())
      {
         YoVariableType type = typeOf(field.getType());
         String fieldName = field.getName();
         boolean isArray = field.isArray();
         int arrayLength = field.getMaxLength();

         Consumer<ByteBuffer> deserializer = null;
         deserializer = createYoVariable(field, schemaRegistry);

         if (deserializer != null)
            continue;

         if (type != null)
         {
            if (!isArray)
               deserializers.add();
            else
               deserializers.add(createYoVariableArray(fieldName, type, arrayLength, schemaRegistry));
         }
         else if ("string".equals(field.getType()))
         {
            LogTools.warn("YoString not implemented yet. name = " + schemaRegistry.getName() + ", field = " + field);
            deserializers.add(""); // Mark the position for the string to skip it when reading a message.
         }
         else
         {
            ROS2MessageSchema subSchema = subSchemaMap.get(field.getType());
            if (subSchema == null)
               throw new IllegalStateException("Could not find a schema for the type: %s. Might be missing a primitive type.".formatted(field.getType()));
            if (!isArray)
            {
               YoRegistry fieldRegistry = new YoRegistry(fieldName);
               schemaRegistry.addChild(fieldRegistry);
               instantiateSchema(subSchema, subSchemaMap, fieldRegistry, deserializers);
            }
            else
            {
               for (int i = 0; i < arrayLength; i++)
               {
                  YoRegistry fieldRegistry = new YoRegistry(fieldName + "[" + i + "]");
                  schemaRegistry.addChild(fieldRegistry);
                  instantiateSchema(subSchema, subSchemaMap, fieldRegistry, deserializers);
               }
            }
         }
      }
      return schemaRegistry;
   }

   private static Consumer<ByteBuffer> createYoVariable(ROS2Field field, YoRegistry registry)
   {
      String name = field.getName();

      switch (field.getType())
      {
         case "bool":
         {
            YoBoolean yoBoolean = new YoBoolean(name, registry);
            return buffer -> yoBoolean.set(buffer.get() != 0);
         }
         ;
         case "float64":
         {
            YoDouble yoDouble = new YoDouble(name, registry);
            return buffer -> yoDouble.set(buffer.getDouble());
         }
         case "float32":
         {
            YoDouble yoDouble = new YoDouble(name, registry);
            return buffer -> yoDouble.set(buffer.getFloat());
         }
         case "byte":
         {
            YoInteger yoInteger = new YoInteger(name, registry);
            return buffer -> yoInteger.set(buffer.get());
         }
         case "int16", "uint16":
         {
            YoInteger yoInteger = new YoInteger(name, registry);
            return buffer -> yoInteger.set(buffer.getShort());
         }
         case "int32":
         {
            YoInteger yoInteger = new YoInteger(name, registry);
            return buffer -> yoInteger.set(buffer.getInt());
         }
         case "uint32":
         {
            YoLong yoLong = new YoLong(name, registry);
            return buffer -> yoLong.set(buffer.getInt());
         }
         case "int64", "uint64":
         {
            YoLong yoLong = new YoLong(name, registry);
            return buffer -> yoLong.set(buffer.getLong());
         }
         case "string":
         {
            LogTools.warn("YoString not implemented yet. name = " + registry.getName() + ", field = " + field);
            return buffer ->
            {
               int length = buffer.getInt();
               buffer.position(buffer.position() + length);
            };
         }
      }
      ;
   }

   private static YoVariable[] createYoVariableArray(String name, YoVariableType type, int length, YoRegistry registry)
   {
      YoVariable[] out = switch (type)
      {
         case BOOLEAN -> new YoBoolean[length];
         case DOUBLE -> new YoDouble[length];
         case INTEGER -> new YoInteger[length];
         case LONG -> new YoLong[length];
         default -> null;
      };
      if (out == null)
         return null;
      for (int i = 0; i < length; i++)
         out[i] = createYoVariable(name + "[" + i + "]", type, registry);
      return out;
   }

   private static YoVariableType typeOf(String type)
   {
      return switch (type)
      {
         // TODO Consider creating YoFloat, YoUnsignedInteger, YoUnsignedLong, YoString, etc.
         case "bool" -> YoVariableType.BOOLEAN;
         case "float64", "float32" -> YoVariableType.DOUBLE;
         case "byte", "int16", "uint16", "int32" -> YoVariableType.INTEGER;
         case "uint32", "int64", "uint64" -> YoVariableType.LONG;
         //         case "string" -> YoVariableType.STRING; TODO Gonna need to implement YoString
         default -> null;
      };
   }
}

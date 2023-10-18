package us.ihmc.scs2.session.mcap;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.*;
import us.ihmc.scs2.session.mcap.ROS2MessageSchema.ROS2Field;

import java.util.Map;
import java.util.Objects;

public class YoROS2Message
{
   private final YoRegistry registry;
   private final ROS2MessageSchema schema;

   public YoROS2Message(ROS2MessageSchema schema)
   {
      this.schema = schema;
      registry = instantiateSchema(schema.getName(), schema, schema.getSubSchemaMap());
   }

   public ROS2MessageSchema getSchema()
   {
      return schema;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   private YoRegistry instantiateSchema(String schemaName, ROS2MessageSchema schema, Map<String, ROS2MessageSchema> subSchemaMap)
   {
      Objects.requireNonNull(schema, "Schema cannot be null. schemaName = " + schemaName);

      YoRegistry schemaRegistry = new YoRegistry(schemaName);

      for (ROS2Field field : schema.getFields())
      {
         YoVariableType type = typeOf(field.getType());
         String fieldName = field.getName();
         boolean isArray = field.isArray();
         int arrayLength = field.getMaxLength();

         if (type != null)
         {
            if (!isArray)
               createYoVariable(fieldName, type, schemaRegistry);
            else
               createYoVariableArray(fieldName, type, arrayLength, schemaRegistry);
         }
         else if ("string".equals(field.getType()))
         {
            LogTools.warn("YoString not implemented yet. schemaName = " + schemaName + ", field = " + field);
         }
         else
         {
            ROS2MessageSchema subSchema = subSchemaMap.get(field.getType());
            if (subSchema == null)
               throw new IllegalStateException("Could not find a schema for the type: %s. Might be missing a primitive type.".formatted(field.getType()));
            if (!isArray)
            {
               schemaRegistry.addChild(instantiateSchema(fieldName, subSchema, subSchemaMap));
            }
            else
            {
               for (int i = 0; i < arrayLength; i++)
                  schemaRegistry.addChild(instantiateSchema(fieldName + "[" + i + "]", subSchema, subSchemaMap));
            }
         }
      }
      return schemaRegistry;
   }

   private static YoVariable createYoVariable(String name, YoVariableType type, YoRegistry registry)
   {
      return switch (type)
      {
         case BOOLEAN -> new YoBoolean(name, registry);
         case DOUBLE -> new YoDouble(name, registry);
         case INTEGER -> new YoInteger(name, registry);
         case LONG -> new YoLong(name, registry);
         default -> null;
      };
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

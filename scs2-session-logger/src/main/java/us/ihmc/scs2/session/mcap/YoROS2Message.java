package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiFunction;
import java.util.function.Consumer;

public class YoROS2Message extends YoMCAPMessage
{
   private final ROS2Schema schema;
   private final int channelId;
   private final YoRegistry registry;
   private final Consumer<CDRDeserializer> deserializer;

   public static YoROS2Message newMessage(ROS2Schema schema, int channelId, YoRegistry registry)
   {
      return newMessage(schema, channelId, registry, schema.getSubSchemaMap());
   }

   private static YoROS2Message newMessage(ROS2Schema schema, int channelId, YoRegistry messageRegistry, Map<String, ROS2Schema> subSchemaMap)
   {
      Objects.requireNonNull(schema, "Schema cannot be null. name = " + messageRegistry.getName());

      List<Consumer<CDRDeserializer>> deserializers = new ArrayList<>();

      for (MCAPSchemaField field : schema.getFields())
      {
         String fieldName = field.getName();

         if (field.isVector() && field.isArray())
            throw new IllegalArgumentException("Field cannot be both a vector and an array: " + field + ", registry: " + messageRegistry);

         boolean isArrayOrVector = field.isArray() || field.isVector();

         Consumer<CDRDeserializer> deserializer = null;
         if (!isArrayOrVector)
            deserializer = createYoVariable(field, messageRegistry);
         else
            deserializer = createYoVariableArray(field, messageRegistry);

         if (deserializer != null)
         {
            deserializers.add(deserializer);
            continue;
         }
         else if (!field.isComplexType())
         {
            throw new IllegalStateException("Could not deserialize non-complex field of type: %s".formatted(field.getType()));
         }

         ROS2Schema subSchema = subSchemaMap.get(field.getType());

         if (subSchema == null)
            throw new IllegalStateException("Could not find a schema for the type: %s. Might be missing a primitive type.".formatted(field.getType()));

         if (!isArrayOrVector)
         {
            YoRegistry fieldRegistry = new YoRegistry(fieldName);
            messageRegistry.addChild(fieldRegistry);
            YoROS2Message subMessage = newMessage(subSchema, -1, fieldRegistry, subSchemaMap);
            deserializers.add(subMessage.deserializer);
         }
         else
         {
            BiFunction<String, YoRegistry, YoROS2Message> elementBuilder = (name, yoRegistry) ->
            {
               YoROS2Message newElement = newMessage(subSchema, -1, new YoRegistry(name), subSchemaMap);
               messageRegistry.addChild(newElement.getRegistry());
               return newElement;
            };
            createFieldArray(YoROS2Message.class,
                             elementBuilder,
                             YoROS2Message::deserialize,
                             YoROS2Message::clearData,
                             fieldName,
                             field.isArray(),
                             field.getMaxLength(),
                             messageRegistry);
         }
      }

      return new YoROS2Message(schema, channelId, messageRegistry, cdr ->
      {
         for (Consumer<CDRDeserializer> deserializer : deserializers)
            deserializer.accept(cdr);
      });
   }

   private YoROS2Message(ROS2Schema schema, int channelId, YoRegistry registry, Consumer<CDRDeserializer> deserializer)
   {
      super(schema, channelId, registry, deserializer);
      this.schema = schema;
      this.channelId = channelId;
      this.registry = registry;
      this.deserializer = deserializer;
   }
}

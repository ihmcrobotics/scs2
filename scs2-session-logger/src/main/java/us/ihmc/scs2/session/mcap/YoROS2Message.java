package us.ihmc.scs2.session.mcap;

import java.lang.reflect.Array;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.ROS2MessageSchema.ROS2Field;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoROS2Message
{
   private static final Map<String, YoConversionToolbox<?>> conversionMap;

   static
   {
      List<YoConversionToolbox<?>> allConversions = new ArrayList<>();
      allConversions.add(new YoConversionToolbox<>("bool",
                                                   YoBoolean.class,
                                                   (name, registry) -> new YoBoolean(name, registry),
                                                   (variable, buffer) -> variable.set(Byte.toUnsignedInt(buffer.get()) != 0),
                                                   yoBoolean -> yoBoolean.set(false)));
      allConversions.add(new YoConversionToolbox<>("float64",
                                                   YoDouble.class,
                                                   (name, registry) -> new YoDouble(name, registry),
                                                   (variable, buffer) -> variable.set(buffer.getDouble()),
                                                   yoDouble -> yoDouble.set(Double.NaN)));
      allConversions.add(new YoConversionToolbox<>("float32",
                                                   YoDouble.class,
                                                   (name, registry) -> new YoDouble(name, registry),
                                                   (variable, buffer) -> variable.set(buffer.getFloat()),
                                                   yoDouble -> yoDouble.set(Double.NaN)));
      allConversions.add(new YoConversionToolbox<>("byte",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, buffer) -> variable.set(Byte.toUnsignedInt(buffer.get())),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("int16",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, buffer) -> variable.set(buffer.getShort()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("uint16",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, buffer) -> variable.set(Short.toUnsignedInt(buffer.getShort())),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("int32",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, buffer) -> variable.set(buffer.getInt()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("uint32",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, buffer) -> variable.set(Integer.toUnsignedLong(buffer.getInt())),
                                                   yoLong -> yoLong.set(0)));
      allConversions.add(new YoConversionToolbox<>("int64",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, buffer) -> variable.set(buffer.getLong()),
                                                   yoLong -> yoLong.set(0)));
      // TODO uint64 deserializer: Risk of overflow
      allConversions.add(new YoConversionToolbox<>("uint64",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, buffer) -> variable.set(buffer.getLong()),
                                                   yoLong -> yoLong.set(0)));
      // TODO string deserializer: Preserving the BiConsumer signature to remain consistent with the other deserializers. Only skipping string in ByteBuffer for now.
      allConversions.add(new YoConversionToolbox<>("string", null, null, (variable, buffer) ->
      {
         int length = (int) Integer.toUnsignedLong(buffer.getInt());
         buffer.position(buffer.position() + length);
      }, null));
      conversionMap = allConversions.stream().collect(Collectors.toMap(conversion -> conversion.primitiveType, conversion -> conversion));
   }

   private final ROS2MessageSchema schema;
   private final int channelId;
   private final YoRegistry registry;
   private final Consumer<ByteBuffer> deserializer;

   public static YoROS2Message newMessage(int channelId, ROS2MessageSchema schema)
   {
      return newMessage(schema.getName(), channelId, schema);
   }

   public static YoROS2Message newMessage(String name, int channelId, ROS2MessageSchema schema)
   {
      return newMessage(schema, channelId, new YoRegistry(name));
   }

   public static YoROS2Message newMessage(ROS2MessageSchema schema, int channelId, YoRegistry registry)
   {
      return newMessage(schema, channelId, registry, schema.getSubSchemaMap());
   }

   public static YoROS2Message newMessage(ROS2MessageSchema schema, int channelId, YoRegistry messageRegistry, Map<String, ROS2MessageSchema> subSchemaMap)
   {
      Objects.requireNonNull(schema, "Schema cannot be null. name = " + messageRegistry.getName());

      List<Consumer<ByteBuffer>> deserializers = new ArrayList<>();

      for (ROS2Field field : schema.getFields())
      {
         String fieldName = field.getName();
         boolean isArray = field.isArray();

         Consumer<ByteBuffer> deserializer = null;
         deserializer = createYoVariable(field, messageRegistry);

         if (deserializer != null)
         {
            deserializers.add(deserializer);
            continue;
         }

         if (isArray)
         {
            deserializer = createYoVariableArray(field, messageRegistry);

            if (deserializer != null)
            {
               deserializers.add(deserializer);
               continue;
            }
         }

         ROS2MessageSchema subSchema = subSchemaMap.get(field.getType());

         if (subSchema == null)
            throw new IllegalStateException("Could not find a schema for the type: %s. Might be missing a primitive type.".formatted(field.getType()));

         if (!isArray)
         {
            YoRegistry fieldRegistry = new YoRegistry(fieldName);
            messageRegistry.addChild(fieldRegistry);
            YoROS2Message subMessage = newMessage(subSchema, -1, fieldRegistry, subSchemaMap);
            deserializers.add(subMessage.deserializer);
         }
         else
         {
            createFieldArray(YoROS2Message.class, (name, yoRegistry) ->
            {
               YoROS2Message newElement = newMessage(subSchema, -1, new YoRegistry(name), subSchemaMap);
               messageRegistry.addChild(newElement.getRegistry());
               return newElement;
            }, (message, buffer) -> message.deserialize(buffer), (message) -> message.clearData(), fieldName, field.getMaxLength(), messageRegistry);
         }
      }

      return new YoROS2Message(schema, channelId, messageRegistry, buffer ->
      {
         for (Consumer<ByteBuffer> deserializer : deserializers)
            deserializer.accept(buffer);
      });
   }

   private YoROS2Message(ROS2MessageSchema schema, int channelId, YoRegistry registry, Consumer<ByteBuffer> deserializer)
   {
      this.schema = schema;
      this.channelId = channelId;
      this.registry = registry;
      this.deserializer = deserializer;
   }

   public void readMessage(Mcap.Message message)
   {
      if (message.channelId() != channelId)
         throw new IllegalArgumentException("Expected channel ID: " + channelId + ", but received: " + message.channelId());

      ByteBuffer dataBuffer = ByteBuffer.wrap(message.data());
      dataBuffer.order(ByteOrder.LITTLE_ENDIAN);

      if (registry.getName().equals("floating_base_pose"))
         System.out.println();

      try
      {
         deserialize(dataBuffer);
      }
      catch (Exception e)
      {
         LogTools.error("Deserialization failed for message: " + registry.getName());
         throw e;
      }
   }

   private void deserialize(ByteBuffer buffer)
   {
      deserializer.accept(buffer);
   }

   private void clearData()
   {
      deserializer.accept(null);
   }

   public int getChannelId()
   {
      return channelId;
   }

   public ROS2MessageSchema getSchema()
   {
      return schema;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   private static Consumer<ByteBuffer> createYoVariable(ROS2Field field, YoRegistry registry)
   {
      String fieldName = field.getName();
      String fieldType = field.getType();

      YoConversionToolbox conversion = conversionMap.get(fieldType);
      return conversion != null ? conversion.createYoVariable(fieldName, registry) : null;
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private static Consumer<ByteBuffer> createYoVariableArray(ROS2Field field, YoRegistry registry)
   {
      int maxLength = field.getMaxLength();
      String fieldName = field.getName();
      String fieldType = field.getType();

      YoConversionToolbox conversion = conversionMap.get(fieldType);
      if (conversion != null)
         return createFieldArray(conversion.yoType, conversion.yoBuilder, conversion.deserializer, conversion.yoResetter, fieldName, maxLength, registry);
      return null;
   }

   private static <T> Consumer<ByteBuffer> createFieldArray(Class<T> variableType,
                                                            BiFunction<String, YoRegistry, T> elementBuilder,
                                                            BiConsumer<T, ByteBuffer> elementDeserializer,
                                                            Consumer<T> elementResetter,
                                                            String name,
                                                            int length,
                                                            YoRegistry registry)
   {
      T[] array = (T[]) Array.newInstance(variableType, length);
      for (int i = 0; i < length; i++)
         array[i] = elementBuilder.apply(name + "[" + i + "]", registry);
      return buffer ->
      {
         if (buffer == null)
         {
            for (int i = 0; i < length; i++)
               elementResetter.accept(array[i]);
         }
         else
         {
            int arrayLength = (int) Integer.toUnsignedLong(buffer.getInt());
            for (int i = 0; i < arrayLength; i++)
               elementDeserializer.accept(array[i], buffer);
            for (int i = arrayLength; i < length; i++)
               elementResetter.accept(array[i]);
         }
      };
   }

   private static class YoConversionToolbox<T extends YoVariable>
   {
      private final String primitiveType;
      private final Class<T> yoType;
      private final BiFunction<String, YoRegistry, T> yoBuilder;
      private final BiConsumer<T, ByteBuffer> deserializer;
      private final Consumer<T> yoResetter;

      private YoConversionToolbox(String primitiveType,
                                  Class<T> yoType,
                                  BiFunction<String, YoRegistry, T> yoBuilder,
                                  BiConsumer<T, ByteBuffer> deserializer,
                                  Consumer<T> yoResetter)
      {
         this.primitiveType = primitiveType;
         this.yoType = yoType;
         this.yoBuilder = yoBuilder;
         this.deserializer = deserializer;
         this.yoResetter = yoResetter;
      }

      public Consumer<ByteBuffer> createYoVariable(String name, YoRegistry registry)
      {
         if (yoBuilder != null)
         {
            T yoVariable = yoBuilder.apply(name, registry);
            return buffer ->
            {
               if (buffer == null)
               {
                  try
                  {
                     yoResetter.accept(yoVariable);
                  }
                  catch (Exception e)
                  {
                     LogTools.error("Failed to reset variable: " + yoVariable + ", registry: " + registry);
                     throw new RuntimeException(e);
                  }
               }
               else
               {
                  try
                  {
                     deserializer.accept(yoVariable, buffer);
                  }
                  catch (Exception e)
                  {
                     LogTools.error("Failed to deserialize variable: " + yoVariable + ", registry: " + registry);
                     throw new RuntimeException(e);
                  }
               }
            };
         }
         else if (deserializer != null)
         {
            return buffer ->
            {
               try
               {
                  deserializer.accept(null, buffer);
               }
               catch (Exception e)
               {
                  LogTools.error("Failed to deserialize variable: " + name + ", registry: " + registry);
                  throw new RuntimeException(e);
               }
            };
         }
         else
         {
            return null;
         }
      }
   }
}

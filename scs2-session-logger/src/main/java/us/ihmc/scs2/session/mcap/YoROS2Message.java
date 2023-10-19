package us.ihmc.scs2.session.mcap;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import us.ihmc.idl.CDR;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.common.SerializedPayload;
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
                                                   (variable, cdr) -> variable.set(cdr.read_type_7()),
                                                   yoBoolean -> yoBoolean.set(false)));
      allConversions.add(new YoConversionToolbox<>("float64",
                                                   YoDouble.class,
                                                   (name, registry) -> new YoDouble(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_6()),
                                                   yoDouble -> yoDouble.set(Double.NaN)));
      allConversions.add(new YoConversionToolbox<>("float32",
                                                   YoDouble.class,
                                                   (name, registry) -> new YoDouble(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_5()),
                                                   yoDouble -> yoDouble.set(Double.NaN)));
      allConversions.add(new YoConversionToolbox<>("byte",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_9()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("int16",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_1()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("uint16",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_3()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("int32",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_2()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("uint32",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_4()),
                                                   yoLong -> yoLong.set(0)));
      allConversions.add(new YoConversionToolbox<>("int64",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_11()),
                                                   yoLong -> yoLong.set(0)));
      // TODO uint64 deserializer: Risk of overflow
      allConversions.add(new YoConversionToolbox<>("uint64",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_type_12()),
                                                   yoLong -> yoLong.set(0)));
      // TODO string deserializer: Preserving the BiConsumer signature to remain consistent with the other deserializers. Only skipping string in CDR for now.
      allConversions.add(new YoConversionToolbox<>("string", null, null, new BiConsumer<YoVariable, CDR>()
      {
         private final StringBuilder sb = new StringBuilder();

         @Override
         public void accept(YoVariable variable, CDR cdr)
         {
            cdr.read_type_d(sb);
         }
      }, null));
      conversionMap = allConversions.stream().collect(Collectors.toMap(conversion -> conversion.primitiveType, conversion -> conversion));
   }

   private final ROS2MessageSchema schema;
   private final int channelId;
   private final YoRegistry registry;
   private final Consumer<CDR> deserializer;

   /**
    * Used to deserialize a message data.
    */
   private final CDR cdr = new CDR();

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

      List<Consumer<CDR>> deserializers = new ArrayList<>();

      for (ROS2Field field : schema.getFields())
      {
         String fieldName = field.getName();
         boolean isArray = field.isArray();

         Consumer<CDR> deserializer = null;
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
            }, (message, cdr) -> message.deserialize(cdr), (message) -> message.clearData(), fieldName, field.getMaxLength(), messageRegistry);
         }
      }

      return new YoROS2Message(schema, channelId, messageRegistry, cdr ->
      {
         for (Consumer<CDR> deserializer : deserializers)
            deserializer.accept(cdr);
      });
   }

   private YoROS2Message(ROS2MessageSchema schema, int channelId, YoRegistry registry, Consumer<CDR> deserializer)
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

      SerializedPayload payload = new SerializedPayload(message.data().length);
      payload.getData().put(message.data());
      payload.getData().position(0);
      payload.getData().limit(payload.getData().capacity());
      cdr.deserialize(payload);

      try
      {
         deserialize(cdr);
         cdr.finishDeserialize();
      }
      catch (Exception e)
      {
         LogTools.error("Deserialization failed for message: " + registry.getName() + ", schema ID: " + schema.getId() + ", schema name: " + schema.getName()
                        + ", message data: " + Arrays.toString(message.data()));
         throw e;
      }
   }

   private void deserialize(CDR cdr)
   {
      deserializer.accept(cdr);
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

   @SuppressWarnings({"rawtypes", "unchecked"})
   private static Consumer<CDR> createYoVariable(ROS2Field field, YoRegistry registry)
   {
      String fieldName = field.getName();
      String fieldType = field.getType();

      YoConversionToolbox conversion = conversionMap.get(fieldType);
      return conversion != null ? conversion.createYoVariable(fieldName, registry) : null;
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private static Consumer<CDR> createYoVariableArray(ROS2Field field, YoRegistry registry)
   {
      int maxLength = field.getMaxLength();
      String fieldName = field.getName();
      String fieldType = field.getType();

      YoConversionToolbox conversion = conversionMap.get(fieldType);
      if (conversion != null)
         return createFieldArray(conversion.yoType, conversion.yoBuilder, conversion.deserializer, conversion.yoResetter, fieldName, maxLength, registry);
      return null;
   }

   private static <T> Consumer<CDR> createFieldArray(Class<T> variableType,
                                                     BiFunction<String, YoRegistry, T> elementBuilder,
                                                     BiConsumer<T, CDR> elementDeserializer,
                                                     Consumer<T> elementResetter,
                                                     String name,
                                                     int length,
                                                     YoRegistry registry)
   {
      @SuppressWarnings("unchecked")
      T[] array = (T[]) Array.newInstance(variableType, length);
      for (int i = 0; i < length; i++)
         array[i] = elementBuilder.apply(name + "[" + i + "]", registry);
      return cdr ->
      {
         if (cdr == null)
         {
            for (int i = 0; i < length; i++)
               elementResetter.accept(array[i]);
         }
         else
         {
            int arrayLength = (int) cdr.read_type_4();
            for (int i = 0; i < arrayLength; i++)
               elementDeserializer.accept(array[i], cdr);
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
      private final BiConsumer<T, CDR> deserializer;
      private final Consumer<T> yoResetter;

      private YoConversionToolbox(String primitiveType,
                                  Class<T> yoType,
                                  BiFunction<String, YoRegistry, T> yoBuilder,
                                  BiConsumer<T, CDR> deserializer,
                                  Consumer<T> yoResetter)
      {
         this.primitiveType = primitiveType;
         this.yoType = yoType;
         this.yoBuilder = yoBuilder;
         this.deserializer = deserializer;
         this.yoResetter = yoResetter;
      }

      public Consumer<CDR> createYoVariable(String name, YoRegistry registry)
      {
         if (yoBuilder != null)
         {
            T yoVariable = yoBuilder.apply(name, registry);
            return cdr ->
            {
               if (cdr == null)
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
                     deserializer.accept(yoVariable, cdr);
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
            return cdr ->
            {
               try
               {
                  deserializer.accept(null, cdr);
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

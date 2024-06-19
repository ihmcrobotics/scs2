package us.ihmc.scs2.session.mcap;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public final class YoMCAPMessage
{
   private final MCAPSchema schema;
   private final int channelId;
   private final YoRegistry registry;
   private final Consumer<CDRDeserializer> deserializer;
   /**
    * Used to deserialize a message data.
    */
   private final CDRDeserializer cdr = new CDRDeserializer();

   public static YoMCAPMessage newMessage(MCAPSchema schema, int channelId, YoRegistry registry)
   {
      return newMessage(schema, channelId, registry, schema.getSubSchemaMap());
   }

   private static YoMCAPMessage newMessage(MCAPSchema schema, int channelId, YoRegistry messageRegistry, Map<String, MCAPSchema> subSchemaMap)
   {
      Objects.requireNonNull(schema, "Schema cannot be null. name = " + messageRegistry.getName());

      List<Consumer<CDRDeserializer>> deserializers = new ArrayList<>();

      for (MCAPSchemaField field : schema.getFields())
      {
         String fieldName = field.getName();

         if (field.isVector() && field.isArray())
            throw new IllegalArgumentException("Field cannot be both a vector and an array: " + field + ", registry: " + messageRegistry);

         if (field.getDefaultValue() != null)
            continue; // Ignoring static fields for now. We'll process them further down when they happen to declare an enum.

         boolean isArrayOrVector = field.isArray() || field.isVector();

         Consumer<CDRDeserializer> deserializer;
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

         MCAPSchema subSchema = subSchemaMap.get(field.getType());

         if (subSchema == null)
            throw new IllegalStateException("Could not find a schema for the type: %s. Might be missing a primitive type.".formatted(field.getType()));

         if (!isArrayOrVector)
         {
            if (subSchema.isEnum())
            {
               deserializers.add(Objects.requireNonNull(createYoEnum(field, subSchema, messageRegistry),
                                                        "Failed to create enum for field: " + field + ", registry: " + messageRegistry));
            }
            else
            {
               YoRegistry fieldRegistry = new YoRegistry(fieldName);
               messageRegistry.addChild(fieldRegistry);
               YoMCAPMessage subMessage = newMessage(subSchema, -1, fieldRegistry, subSchemaMap);
               deserializers.add(subMessage.deserializer);
            }
         }
         else
         {
            if (subSchema.isEnum())
            {
               deserializers.add(createYoEnumArray(field, subSchema, messageRegistry));
            }
            else
            {
               BiFunction<String, YoRegistry, YoMCAPMessage> elementBuilder = (name, yoRegistry) ->
               {
                  YoMCAPMessage newElement = newMessage(subSchema, -1, new YoRegistry(name), subSchemaMap);
                  messageRegistry.addChild(newElement.getRegistry());
                  return newElement;
               };
               deserializers.add(createFieldArray(YoMCAPMessage.class,
                                                  elementBuilder,
                                                  YoMCAPMessage::deserialize,
                                                  YoMCAPMessage::clearData,
                                                  fieldName,
                                                  field.isArray(),
                                                  field.getMaxLength(),
                                                  messageRegistry));
            }
         }
      }

      return new YoMCAPMessage(schema, channelId, messageRegistry, cdr ->
      {
         for (Consumer<CDRDeserializer> deserializer : deserializers)
            deserializer.accept(cdr);
      });
   }

   private YoMCAPMessage(MCAPSchema schema, int channelId, YoRegistry registry, Consumer<CDRDeserializer> deserializer)
   {
      this.schema = schema;
      this.channelId = channelId;
      this.registry = registry;
      this.deserializer = deserializer;
   }

   public MCAPSchema getSchema()
   {
      return schema;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public int getChannelId()
   {
      return channelId;
   }

   public void readMessage(Message message)
   {
      if (message.channelId() != channelId)
         throw new IllegalArgumentException("Expected channel ID: " + channelId + ", but received: " + message.channelId());

      cdr.initialize(message.messageBuffer(), 0, message.dataLength());

      try
      {
         deserialize(cdr);
      }
      catch (Exception e)
      {
         LogTools.error("Deserialization failed for message: " + registry.getName() + ", schema ID: " + schema.getId() + ", schema name: " + schema.getName());
         throw e;
      }
      finally
      {
         cdr.finalize(true);
      }
   }

   private void deserialize(CDRDeserializer cdr)
   {
      deserializer.accept(cdr);
   }

   private void clearData()
   {
      deserializer.accept(null);
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   private static Consumer<CDRDeserializer> createYoVariable(MCAPSchemaField field, YoRegistry registry)
   {
      String fieldName = field.getName();
      String fieldType = field.getType();

      YoConversionToolbox conversion = conversionMap.get(fieldType);
      return conversion != null ? conversion.createYoVariable(fieldName, registry) : null;
   }

   private static Consumer<CDRDeserializer> createYoEnum(MCAPSchemaField field, MCAPSchema enumSchema, YoRegistry registry)
   {
      String fieldName = field.getName();

      return createYoEnum(fieldName, enumSchema, registry);
   }

   private static Consumer<CDRDeserializer> createYoEnum(String fieldName, MCAPSchema enumSchema, YoRegistry registry)
   {
      if (!enumSchema.isEnum())
         throw new IllegalArgumentException("Schema is not an enum: " + enumSchema + ", registry: " + registry);

      return createYoEnumConversionToolbox(enumSchema).createYoVariable(fieldName, registry);
   }

   /**
    * Creates an array of {@code YoVariable}s which can be used to parse a ROS2 field that is either an
    * array or a vector.
    *
    * @param field    the ROS2 field to instantiate into a {@code YoVariable} array.
    * @param registry the registry in which the {@code YoVariable}s are to be added.
    * @return the parsing function.
    */
   @SuppressWarnings({"unchecked", "rawtypes"})
   private static Consumer<CDRDeserializer> createYoVariableArray(MCAPSchemaField field, YoRegistry registry)
   {
      int maxLength = field.getMaxLength();
      String fieldName = field.getName();
      String fieldType = field.getType();

      if (field.isVector() == field.isArray())
         throw new IllegalArgumentException("Field is neither a vector nor an array: " + field + ", registry: " + registry);

      boolean isFixedSize = field.isArray();

      YoConversionToolbox conversion = conversionMap.get(fieldType);
      if (conversion != null)
         return createFieldArray(conversion.yoType(),
                                 conversion.yoBuilder(),
                                 conversion.deserializer(),
                                 conversion.yoResetter(),
                                 fieldName,
                                 isFixedSize,
                                 maxLength,
                                 registry);
      return null;
   }

   @SuppressWarnings("rawtypes")
   private static Consumer<CDRDeserializer> createYoEnumArray(MCAPSchemaField field, MCAPSchema enumSchema, YoRegistry registry)
   {
      int maxLength = field.getMaxLength();
      String fieldName = field.getName();

      if (field.isVector() == field.isArray())
         throw new IllegalArgumentException("Field is neither a vector nor an array: " + field + ", registry: " + registry);

      boolean isFixedSize = field.isArray();

      YoConversionToolbox<YoEnum> conversion = createYoEnumConversionToolbox(enumSchema);
      return createFieldArray(conversion.yoType(),
                              conversion.yoBuilder(),
                              conversion.deserializer(),
                              conversion.yoResetter(),
                              fieldName,
                              isFixedSize,
                              maxLength,
                              registry);
   }

   /**
    * Creates an array of {@code YoVariable}s which can be used to parse a ROS2 field that is either an
    * array or a vector.
    *
    * @param <T>                 the type of the {@code YoVariable}.
    * @param variableType        the type of the {@code YoVariable} to be created.
    * @param elementBuilder      the function used to create a new {@code YoVariable}.
    * @param elementDeserializer the function used to deserialize a ROS2 message and update the
    *                            {@code YoVariable}.
    * @param elementResetter     the function used to reset a {@code YoVariable}.
    * @param name                the base name of the {@code YoVariable}.
    * @param isFixedSize         whether the array is fixed size or not.
    * @param length              the length of the array.
    * @param registry            the registry in which the {@code YoVariable}s are to be added.
    * @return the parsing function.
    */
   @SuppressWarnings("unchecked")
   private static <T> Consumer<CDRDeserializer> createFieldArray(Class<T> variableType,
                                                                 BiFunction<String, YoRegistry, T> elementBuilder,
                                                                 BiConsumer<T, CDRDeserializer> elementDeserializer,
                                                                 Consumer<T> elementResetter,
                                                                 String name,
                                                                 boolean isFixedSize,
                                                                 int length,
                                                                 YoRegistry registry)
   {
      T[] array;
      if (variableType == null)
      { // Probably dealing with String type
         array = null;
      }
      else
      {
         array = (T[]) Array.newInstance(variableType, length);

         for (int i = 0; i < length; i++)
            array[i] = elementBuilder.apply(name + "[" + i + "]", registry);
      }

      return cdr ->
      {
         if (cdr == null)
         {
            if (array != null && elementResetter != null)
            {
               for (int i = 0; i < length; i++)
                  elementResetter.accept(array[i]);
            }
         }
         else
         {
            if (isFixedSize)
            {
               cdr.read_array((elementIndex, des) -> elementDeserializer.accept(array == null ? null : array[elementIndex], des), length);
            }
            else
            {
               int size = cdr.read_sequence((elementIndex, des) ->
                                            {
                                               if (elementIndex < length)
                                                  elementDeserializer.accept(array == null ? null : array[elementIndex], des);
                                            });
               if (size > length)
                  LogTools.warn("Received array of size: " + size + ", but expected size: " + length + ", registry: " + registry);
               if (array != null && elementResetter != null)
               {
                  for (int i = size; i < length; i++)
                     elementResetter.accept(array[i]);
               }
            }
         }
      };
   }

   public static final Map<String, YoConversionToolbox<?>> conversionMap;

   @SuppressWarnings("rawtypes")
   private static YoConversionToolbox<YoEnum> createYoEnumConversionToolbox(MCAPSchema enumSchema)
   {
      return new YoConversionToolbox<>("enum",
                                       YoEnum.class,
                                       (name, registry) -> new YoEnum<>(name, "", registry, true, enumSchema.getEnumConstants()),
                                       (v, cdr1) -> v.set(cdr1.read_int32()),
                                       // TODO Not sure if this is the right way to read an enum
                                       v -> v.set(YoEnum.NULL_VALUE));
   }

   static
   {
      List<YoConversionToolbox<?>> allConversions = new ArrayList<>();
      Consumer<YoBoolean> yoBooleanResetter = v -> v.set(false);
      Consumer<YoDouble> yoDoubleResetter = v -> v.set(Double.NaN);
      Consumer<YoInteger> yoIntegerResetter = v -> v.set(0);
      Consumer<YoLong> yoLongResetter = v -> v.set(0);

      allConversions.add(new YoConversionToolbox<>("bool", YoBoolean.class, YoBoolean::new, (v, cdr) -> v.set(cdr.read_bool()), yoBooleanResetter));
      allConversions.add(new YoConversionToolbox<>("boolean", YoBoolean.class, YoBoolean::new, (v, cdr) -> v.set(cdr.read_bool()), yoBooleanResetter));
      allConversions.add(new YoConversionToolbox<>("float64", YoDouble.class, YoDouble::new, (v, cdr) -> v.set(cdr.read_float64()), yoDoubleResetter));
      allConversions.add(new YoConversionToolbox<>("double", YoDouble.class, YoDouble::new, (v, cdr) -> v.set(cdr.read_float64()), yoDoubleResetter));
      allConversions.add(new YoConversionToolbox<>("float32", YoDouble.class, YoDouble::new, (v, cdr) -> v.set(cdr.read_float32()), yoDoubleResetter));
      allConversions.add(new YoConversionToolbox<>("float", YoDouble.class, YoDouble::new, (v, cdr) -> v.set(cdr.read_float32()), yoDoubleResetter));
      allConversions.add(new YoConversionToolbox<>("byte", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_byte()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("char", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_byte()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("octet", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_byte()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("int8", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_int8()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("uint8", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_uint8()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("int16", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_int16()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("short", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_int16()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("uint16", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_uint16()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("unsignedshort", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_uint16()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("int32", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_int32()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("long", YoInteger.class, YoInteger::new, (v, cdr) -> v.set(cdr.read_int32()), yoIntegerResetter));
      allConversions.add(new YoConversionToolbox<>("uint32", YoLong.class, YoLong::new, (v, cdr) -> v.set(cdr.read_uint32()), yoLongResetter));
      allConversions.add(new YoConversionToolbox<>("unsignedlong", YoLong.class, YoLong::new, (v, cdr) -> v.set(cdr.read_uint32()), yoLongResetter));
      allConversions.add(new YoConversionToolbox<>("int64", YoLong.class, YoLong::new, (v, cdr) -> v.set(cdr.read_int64()), yoLongResetter));
      allConversions.add(new YoConversionToolbox<>("longlong", YoLong.class, YoLong::new, (v, cdr) -> v.set(cdr.read_int64()), yoLongResetter));
      // TODO uint64 deserializer: Risk of overflow
      allConversions.add(new YoConversionToolbox<>("uint64", YoLong.class, YoLong::new, (v, cdr) -> v.set(cdr.read_uint64()), yoLongResetter));
      allConversions.add(new YoConversionToolbox<>("unsignedlonglong", YoLong.class, YoLong::new, (v, cdr) -> v.set(cdr.read_uint64()), yoLongResetter));
      // TODO string deserializer: Preserving the BiConsumer signature to remain consistent with the other deserializers. Only skipping string in CDR for now.
      allConversions.add(new YoConversionToolbox<>("string", null, null, (v, cdr) -> cdr.read_string(), null));
      conversionMap = allConversions.stream().collect(Collectors.toMap(YoConversionToolbox::primitiveType, conversion -> conversion));
   }

   public record YoConversionToolbox<T extends YoVariable>(String primitiveType, Class<T> yoType, BiFunction<String, YoRegistry, T> yoBuilder,
                                                           BiConsumer<T, CDRDeserializer> deserializer, Consumer<T> yoResetter)
   {
      public Consumer<CDRDeserializer> createYoVariable(String name, YoRegistry registry)
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
               if (cdr == null)
               {
                  if (yoResetter == null)
                     return;
                  try
                  {
                     yoResetter.accept(null);
                  }
                  catch (Exception e)
                  {
                     LogTools.error("Failed to reset variable: " + name + ", registry: " + registry);
                     throw new RuntimeException(e);
                  }
               }

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

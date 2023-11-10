package us.ihmc.scs2.session.mcap;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.OMGIDLSchema.OMGIDLField;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.*;

import java.lang.reflect.Array;
import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class YoOMGIDLMessage implements YoMCAPMessage
{
   private static final Map<String, YoConversionToolbox<?>> conversionMap;

   static
   {
      List<YoConversionToolbox<?>> allConversions = new ArrayList<>();
      allConversions.add(new YoConversionToolbox<>("bool",
                                                   YoBoolean.class,
                                                   (name, registry) -> new YoBoolean(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_bool()),
                                                   yoBoolean -> yoBoolean.set(false)));
      allConversions.add(new YoConversionToolbox<>("boolean",
                                                   YoBoolean.class,
                                                   (name, registry) -> new YoBoolean(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_bool()),
                                                   yoBoolean -> yoBoolean.set(false)));
      allConversions.add(new YoConversionToolbox<>("float64",
                                                   YoDouble.class,
                                                   (name, registry) -> new YoDouble(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_float64()),
                                                   yoDouble -> yoDouble.set(Double.NaN)));
      allConversions.add(new YoConversionToolbox<>("double",
                                                   YoDouble.class,
                                                   (name, registry) -> new YoDouble(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_float64()),
                                                   yoDouble -> yoDouble.set(Double.NaN)));
      allConversions.add(new YoConversionToolbox<>("float32",
                                                   YoDouble.class,
                                                   (name, registry) -> new YoDouble(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_float32()),
                                                   yoDouble -> yoDouble.set(Double.NaN)));
      allConversions.add(new YoConversionToolbox<>("float",
                                                   YoDouble.class,
                                                   (name, registry) -> new YoDouble(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_float32()),
                                                   yoDouble -> yoDouble.set(Double.NaN)));
      allConversions.add(new YoConversionToolbox<>("byte",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_byte()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("char",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_byte()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("octet",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_byte()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("int8",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_int8()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("uint8",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_uint8()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("int16",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_int16()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("short",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_int16()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("uint16",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_uint16()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("unsignedshort",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_uint16()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("int32",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_int32()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("long",
                                                   YoInteger.class,
                                                   (name, registry) -> new YoInteger(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_int32()),
                                                   yoInteger -> yoInteger.set(0)));
      allConversions.add(new YoConversionToolbox<>("uint32",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_uint32()),
                                                   yoLong -> yoLong.set(0)));
      allConversions.add(new YoConversionToolbox<>("unsignedlong",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_uint32()),
                                                   yoLong -> yoLong.set(0)));
      allConversions.add(new YoConversionToolbox<>("int64",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_int64()),
                                                   yoLong -> yoLong.set(0)));
      allConversions.add(new YoConversionToolbox<>("longlong",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_int64()),
                                                   yoLong -> yoLong.set(0)));
      // TODO uint64 deserializer: Risk of overflow
      allConversions.add(new YoConversionToolbox<>("uint64",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_uint64()),
                                                   yoLong -> yoLong.set(0)));
      allConversions.add(new YoConversionToolbox<>("unsignedlonglong",
                                                   YoLong.class,
                                                   (name, registry) -> new YoLong(name, registry),
                                                   (variable, cdr) -> variable.set(cdr.read_uint64()),
                                                   yoLong -> yoLong.set(0)));
      // TODO string deserializer: Preserving the BiConsumer signature to remain consistent with the other deserializers. Only skipping string in CDR for now.
      allConversions.add(new YoConversionToolbox<>("string", null, null, (variable, cdr) -> cdr.read_string(), null));
      conversionMap = allConversions.stream().collect(Collectors.toMap(conversion -> conversion.primitiveType, conversion -> conversion));
   }

   private final OMGIDLSchema schema;
   private final int channelId;
   private final YoRegistry registry;
   private final Consumer<CDRDeserializer> deserializer;

   /**
    * Used to deserialize a message data.
    */
   private final CDRDeserializer cdr = new CDRDeserializer();

   public YoOMGIDLMessage newMessage(int channelId, MCAPSchema schema)
   {
      return newMessage(channelId, (OMGIDLSchema) schema);
   }

   public static YoOMGIDLMessage newMessage(int channelId, OMGIDLSchema schema)
   {
      return newMessage(schema.getName(), channelId, schema);
   }

   public static YoOMGIDLMessage newMessage(String name, int channelId, OMGIDLSchema schema)
   {
      return newMessage(schema, channelId, new YoRegistry(name));
   }

   public static YoOMGIDLMessage newMessage(OMGIDLSchema schema, int channelId, YoRegistry registry)
   {
      return newMessage(schema, channelId, registry, schema.getSubSchemaMap());
   }

   public static YoOMGIDLMessage newMessage(OMGIDLSchema schema, int channelId, YoRegistry messageRegistry, Map<String, OMGIDLSchema> subSchemaMap)
   {
      Objects.requireNonNull(schema, "Schema cannot be null. name = " + messageRegistry.getName());

      List<Consumer<CDRDeserializer>> deserializers = new ArrayList<>();

      for (OMGIDLField field : schema.getFields())
      {
         String fieldName = field.getName();

         if (field.isVector() && field.isArray())
            throw new IllegalArgumentException("Field cannot be both a vector and an array: " + field + ", registry: " + messageRegistry);

         boolean isArrayOrVector = field.isArray() || field.isVector();

         Consumer<CDRDeserializer> deserializer = null;
         if (!isArrayOrVector)
         {
            deserializer = createYoVariable(field, messageRegistry);
         }
         else
         {
            // We need to check if it's an array/vector of other structs
            // TODO: (AM) nested sequences not supported yet

            //TODO: (AM) replace the regex split with something like field.getBaseType()
            if (field.getType().contains("sequence") && subSchemaMap.containsKey((field.getType().split("[<,>]")[1])))
            {
               // TODO: (AM) array/vector of structs, handle recursively
               throw new IllegalStateException("Can't handle sequences of type %s yet".formatted(field.getType().split("[<,>]")[1]));
            }
            else
            {
               // array/vector of primitives
               deserializer = createYoVariableArray(field, messageRegistry);
            }
         }

         if (deserializer != null)
         {
            deserializers.add(deserializer);
            continue;
         }
         if (deserializer == null && !field.isComplexType())
            throw new IllegalStateException("Could not deserialize non-complex field of type: %s".formatted(field.getType()));

         OMGIDLSchema subSchema = null;
         if (field.getType().equals("struct"))
         {
            subSchema = subSchemaMap.get(field.getName());
         }
         else
         {
            subSchema = subSchemaMap.get(field.getType());
         }

         if (subSchema == null)
            throw new IllegalStateException("Could not find a schema for the type: %s. Might be missing a primitive type.".formatted(field.getType()));

         if (!isArrayOrVector)
         {
            YoRegistry fieldRegistry = new YoRegistry(fieldName);
            messageRegistry.addChild(fieldRegistry);
            YoOMGIDLMessage subMessage = newMessage(subSchema, -1, fieldRegistry, subSchemaMap);
            deserializers.add(subMessage.deserializer);
         }
         else
         {
            // TODO: (AM) finalSubSchema exists because the schema object being passed to a lambda function needs to be "effectively final"
            OMGIDLSchema finalSubSchema = subSchema;
            BiFunction<String, YoRegistry, YoOMGIDLMessage> elementBuilder = (name, yoRegistry) ->
            {
               YoOMGIDLMessage newElement = newMessage(finalSubSchema, -1, new YoRegistry(name), subSchemaMap);
               messageRegistry.addChild(newElement.getRegistry());
               return newElement;
            };
            createFieldArray(YoOMGIDLMessage.class,
                             elementBuilder,
                             YoOMGIDLMessage::deserialize,
                             YoOMGIDLMessage::clearData,
                             fieldName,
                             field.isArray(),
                             field.getMaxLength(),
                             messageRegistry);
         }
      }

      return new YoOMGIDLMessage(schema, channelId, messageRegistry, cdr ->
      {
         for (Consumer<CDRDeserializer> deserializer : deserializers)
            deserializer.accept(cdr);
      });
   }

   private YoOMGIDLMessage(OMGIDLSchema schema, int channelId, YoRegistry registry, Consumer<CDRDeserializer> deserializer)
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

      cdr.initialize(message.messageBuffer(), message.offsetData(), message.lengthData());

      try
      {
         deserialize(cdr);
      }
      catch (Exception e)
      {
         LogTools.error("Deserialization failed for message: " + registry.getName() + ", schema ID: " + schema.getId() + ", schema name: " + schema.getName()
                        + ", message data: " + Arrays.toString(message.data()));
         throw e;
      }
      finally
      {
         cdr.finalize(false);
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

   public int getChannelId()
   {
      return channelId;
   }

   public OMGIDLSchema getSchema()
   {
      return schema;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   private static Consumer<CDRDeserializer> createYoVariable(OMGIDLField field, YoRegistry registry)
   {
      String fieldName = field.getName();
      String fieldType = field.getType();

      YoConversionToolbox conversion = conversionMap.get(fieldType);
      return conversion != null ? conversion.createYoVariable(fieldName, registry) : null;
   }

   /**
    * Creates an array of {@code YoVariable}s which can be used to parse a ROS2 field that is either an array or a vector.
    *
    * @param field    the ROS2 field to instantiate into a {@code YoVariable} array.
    * @param registry the registry in which the {@code YoVariable}s are to be added.
    * @return the parsing function.
    */
   @SuppressWarnings({"unchecked", "rawtypes"})
   private static Consumer<CDRDeserializer> createYoVariableArray(OMGIDLField field, YoRegistry registry)
   {
      int maxLength = field.getMaxLength();
      String fieldName = field.getName();
      //TODO: (AM) we manually get the primitive type from the sequence type string, find a better way to get a field's base type
      String fieldBaseType = field.getType();
      if (fieldBaseType.contains("sequence"))
      {
         fieldBaseType = fieldBaseType.split("[<,>]")[1];
      }

      if (field.isVector() == field.isArray())
         throw new IllegalArgumentException("Field is neither a vector nor an array: " + field + ", registry: " + registry);

      boolean isFixedSize = field.isArray();

      YoConversionToolbox conversion = conversionMap.get(fieldBaseType);
      if (conversion != null)
         return createFieldArray(conversion.yoType,
                                 conversion.yoBuilder,
                                 conversion.deserializer,
                                 conversion.yoResetter,
                                 fieldName,
                                 isFixedSize,
                                 maxLength,
                                 registry);
      return null;
   }

   /**
    * Creates an array of {@code YoVariable}s which can be used to parse a ROS2 field that is either an array or a vector.
    *
    * @param variableType        the type of the {@code YoVariable} to be created.
    * @param elementBuilder      the function used to create a new {@code YoVariable}.
    * @param elementDeserializer the function used to deserialize a ROS2 message and update the {@code YoVariable}.
    * @param elementResetter     the function used to reset a {@code YoVariable}.
    * @param name                the base name of the {@code YoVariable}.
    * @param isFixedSize         whether the array is fixed size or not.
    * @param length              the length of the array.
    * @param registry            the registry in which the {@code YoVariable}s are to be added.
    * @param <T>                 the type of the {@code YoVariable}.
    * @return the parsing function.
    */
   private static <T> Consumer<CDRDeserializer> createFieldArray(Class<T> variableType,
                                                                 BiFunction<String, YoRegistry, T> elementBuilder,
                                                                 BiConsumer<T, CDRDeserializer> elementDeserializer,
                                                                 Consumer<T> elementResetter,
                                                                 String name,
                                                                 boolean isFixedSize,
                                                                 int length,
                                                                 YoRegistry registry)
   {
      @SuppressWarnings("unchecked") T[] array = (T[]) Array.newInstance(variableType, length);
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
            if (isFixedSize)
            {
               cdr.read_array((elementIndex, des) -> elementDeserializer.accept(array[elementIndex], des), length);
            }
            else
            {
               int size = cdr.read_sequence((elementIndex, des) -> elementDeserializer.accept(array[elementIndex], des));
               for (int i = size; i < length; i++)
                  elementResetter.accept(array[i]);
            }
         }
      };
   }

   private static class YoConversionToolbox<T extends YoVariable>
   {
      private final String primitiveType;
      private final Class<T> yoType;
      private final BiFunction<String, YoRegistry, T> yoBuilder;
      private final BiConsumer<T, CDRDeserializer> deserializer;
      private final Consumer<T> yoResetter;

      private YoConversionToolbox(String primitiveType,
                                  Class<T> yoType,
                                  BiFunction<String, YoRegistry, T> yoBuilder,
                                  BiConsumer<T, CDRDeserializer> deserializer,
                                  Consumer<T> yoResetter)
      {
         this.primitiveType = primitiveType;
         this.yoType = yoType;
         this.yoBuilder = yoBuilder;
         this.deserializer = deserializer;
         this.yoResetter = yoResetter;
      }

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

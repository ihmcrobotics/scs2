package us.ihmc.scs2.session.mcap;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.OMGIDLSchema.OMGIDLSchemaField;
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

      for (OMGIDLSchemaField field : schema.getFields())
      {
         String fieldName = field.getName();

         if (field.isVector() && field.isArray())
            throw new IllegalArgumentException("Field cannot be both a vector and an array: " + field + ", registry: " + messageRegistry);

         boolean isArrayOrVector = field.isArray() || field.isVector();

         Consumer<CDRDeserializer> deserializer = null;
         if (!isArrayOrVector)
         {
            deserializer = createYoVariable(schema, field, messageRegistry);
         }
         else
         {
            // TODO: (AM) check if nested sequences work properly
            deserializer = createYoVariableArray(schema, field, messageRegistry);
         }

         if (deserializer != null)
         {
            deserializers.add(deserializer);
            continue;
         }
         else if (!field.isComplexType())
         {
            throw new IllegalStateException("Could not deserialize non-complex field of type: %s".formatted(field.getType()));
         }

         OMGIDLSchema subSchema = null;
         if (field.getType().equals("struct"))
         {
            subSchema = subSchemaMap.get(field.getName());
         }
         else if (field.getType().contains("sequence"))
         {
            subSchema = subSchemaMap.get(field.getType().split("[<,>]")[1]);
         }
         else
         {
            subSchema = subSchemaMap.get(field.getType());
         }

         if (subSchema == null)
            throw new IllegalStateException("Could not find a schema for the type: %s. Might be missing a primitive type.".formatted(field.getType()));

         for (Map.Entry<String, OMGIDLSchema> entry : subSchemaMap.entrySet())
         {
            if (!entry.getKey().equals(fieldName))
               subSchema.getSubSchemaMap().put(entry.getKey(), entry.getValue());
         }

         if (!isArrayOrVector)
         {
            //TODO: (AM) This will include struct definitions and instantiations, handle them properly
            YoRegistry fieldRegistry = new YoRegistry(fieldName.replaceAll(":", "-"));
            messageRegistry.addChild(fieldRegistry);
            YoOMGIDLMessage subMessage = newMessage(subSchema, -1, fieldRegistry, subSchemaMap);
            deserializers.add(subMessage.deserializer);
         }
         else
         {
            // TODO: (AM) will we ever even get here?
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

   public void readMessage(MCAP.Message message)
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
   private static Consumer<CDRDeserializer> createYoVariable(OMGIDLSchema schema, OMGIDLSchemaField field, YoRegistry registry)
   {
      //      List<OMGIDLSchemaField> flatFields = schema.flattenField(field).stream().filter(f -> !f.isComplexType()).toList();
      String fieldName = field.getName();
      String fieldType = field.getType();

      YoConversionToolbox conversion = conversionMap.get(fieldType);
      return conversion != null ? conversion.createYoVariable(fieldName, registry) : null;
   }

   /**
    * Creates an array of {@code YoVariable}s which can be used to parse an OMGIDL field that is either an array or a vector.
    *
    * @param schema
    * @param field    the ROS2 field to instantiate into a {@code YoVariable} array.
    * @param registry the registry in which the {@code YoVariable}s are to be added.
    * @return the parsing function.
    */
   @SuppressWarnings({"unchecked", "rawtypes"})
   private static Consumer<CDRDeserializer> createYoVariableArray(OMGIDLSchema schema, OMGIDLSchemaField field, YoRegistry registry)
   {
      List<OMGIDLSchemaField> flatFields = schema.flattenField(field).stream().filter(f -> !f.isComplexType()).toList();
      Map<String, YoVariable[]> fieldArrayMap = new HashMap<>();
      Map<String, YoConversionToolbox> fieldConversionMap = new HashMap<>();

      int maxLength = field.getMaxLength();

      YoRegistry[] subRegistryArray = new YoRegistry[maxLength];//(YoRegistry[]) Array.newInstance(YoRegistry.class, maxLength);

      // create subregs for the field array
      for (int i = 0; i < maxLength; i++)
      {
         subRegistryArray[i] = new YoRegistry(field.getName() + "[" + i + "]");
         registry.addChild(subRegistryArray[i]);
      }

      for (OMGIDLSchemaField flatField : flatFields)
      {
         //TODO: (AM) No string support for now
         if (flatField.getType().equals("string"))
            continue;
         // Build every YoVariable in this field
         YoConversionToolbox conversion = conversionMap.get(flatField.getType());
         if (conversion == null)
            //TODO: (AM) having only simple types in flatfield prevents entering this branch
            return null;
         fieldArrayMap.put(flatField.getName(), (YoVariable[]) Array.newInstance(conversion.yoType, maxLength));
         fieldConversionMap.put(flatField.getName(), conversion);
         String[] flatFieldHierarchy = flatField.getName().split("-");

         for (int i = 0; i < maxLength; i++)
         {
            YoRegistry subFieldRegistry = subRegistryArray[i];
            // traverse from the second to the second last element to add all sub-registries
            for (int j = 1; j < flatFieldHierarchy.length - 1; j++)
            {
               if (subFieldRegistry.getChild(flatFieldHierarchy[j]) == null)
                  subFieldRegistry.addChild(new YoRegistry(flatFieldHierarchy[j]));
               subFieldRegistry = subFieldRegistry.getChild(flatFieldHierarchy[j]);
            }

            fieldArrayMap.get(flatField.getName())[i] = (YoVariable) fieldConversionMap.get(flatField.getName()).yoBuilder.apply(flatFieldHierarchy[
                                                                                                                                       flatFieldHierarchy.length
                                                                                                                                       - 1], subFieldRegistry);
         }
      }

      return cdr ->
      {

         if (cdr == null)
         {
            for (OMGIDLSchemaField flatField : flatFields)
            {
               //TODO: (AM) No string support for now
               if (flatField.getType().equals("string"))
                  continue;

               for (int i = 0; i < maxLength; i++)
               {
                  fieldConversionMap.get(flatField.getName()).yoResetter.accept(fieldArrayMap.get(flatField.getName())[i]);
               }
            }
         }
         else
         {
            if (field.isArray())
            {
               cdr.read_array((elementIndex, deserializer) ->
                              {
                                 for (OMGIDLSchemaField flatField : flatFields)
                                 {
                                    //TODO: (AM) No string support for now
                                    if (flatField.getType().equals("string"))
                                       continue;
                                    fieldConversionMap.get(flatField.getName()).deserializer.accept(fieldArrayMap.get(flatField.getName())[elementIndex],
                                                                                                    deserializer);
                                 }
                              }, maxLength);
            }
            else if (field.isVector())
            {
               int size = cdr.read_sequence((elementIndex, deserializer) ->
                                            {
                                               for (OMGIDLSchemaField flatField : flatFields)
                                               {
                                                  //TODO: (AM) No string support for now
                                                  if (flatField.getType().equals("string"))
                                                     continue;

                                                  fieldConversionMap.get(flatField.getName()).deserializer.accept(fieldArrayMap.get(flatField.getName())[elementIndex],
                                                                                                                  deserializer);
                                               }
                                            });
               // Reset remaining elements
               for (OMGIDLSchemaField flatField : flatFields)
               {
                  if (flatField.getType().equals("string"))
                     continue;

                  for (int i = size; i < maxLength; i++)
                     fieldConversionMap.get(flatField.getName()).yoResetter.accept(fieldArrayMap.get(flatField.getName())[i]);
               }
            }
         }
      };
   }

   /**
    * Creates an array of {@code YoVariable}s which can be used to parse an OMGIDL field that is either an array or a vector.
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

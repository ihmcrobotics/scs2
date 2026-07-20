package us.ihmc.scs2.session.mcap;

import com.google.protobuf.Descriptors.Descriptor;
import com.google.protobuf.Descriptors.EnumValueDescriptor;
import com.google.protobuf.Descriptors.FieldDescriptor;
import com.google.protobuf.Descriptors.FieldDescriptor.JavaType;
import com.google.protobuf.DynamicMessage;
import com.google.protobuf.InvalidProtocolBufferException;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;

/**
 * Counterpart to {@link YoMCAPMessage} for MCAP channels whose message encoding is {@code protobuf}.
 * <p>
 * Protobuf's wire format is a self-describing tag/length-value encoding, decoded via reflection
 * ({@link DynamicMessage} + {@link Descriptor}), not a sequential cursor read in schema-declared order like CDR. This
 * class therefore builds its own {@code YoVariable} bindings directly off a {@link Descriptor} and shares no decode
 * machinery with {@link YoMCAPMessage}/{@code CDRDeserializer}.
 * </p>
 */
public final class YoMCAPProtobufMessage implements MCAPMessageDecoder
{
   public static final int UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH = ProtobufSchemaParser.UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH;

   private final MCAPProtobufSchema schema;
   private final Descriptor descriptor;
   private final int channelId;
   private final YoRegistry registry;
   private final Consumer<com.google.protobuf.Message> deserializer;

   public static YoMCAPProtobufMessage newMessage(MCAPProtobufSchema schema, int channelId, YoRegistry registry)
   {
      return newMessage(schema, channelId, registry, null);
   }

   /**
    * @param sampleMessageData raw bytes of one message recorded on this channel, or {@code null} if none is
    *                          available. Protobuf {@code map} fields are wire-compatible with {@code repeated
    *                          MapEntry{key; value;}}, so their key set can't be recovered from the descriptor alone
    *                          the way every other field can - a sample message is the only way to discover which
    *                          keys to build {@code YoVariable}s for. Without one, map fields are skipped (logged),
    *                          same as before this parameter existed.
    */
   public static YoMCAPProtobufMessage newMessage(MCAPProtobufSchema schema, int channelId, YoRegistry registry, byte[] sampleMessageData)
   {
      Descriptor descriptor = schema.getDescriptor();

      com.google.protobuf.Message sample = null;
      if (sampleMessageData != null)
      {
         try
         {
            sample = DynamicMessage.parseFrom(descriptor, sampleMessageData);
         }
         catch (InvalidProtocolBufferException e)
         {
            LogTools.warn("Failed to parse sample message for channel: " + channelId + ", schema: " + descriptor.getFullName()
                          + "; protobuf map fields will not be decoded.");
         }
      }

      Deque<String> ancestorTypes = new ArrayDeque<>();
      ancestorTypes.push(descriptor.getFullName());
      Consumer<com.google.protobuf.Message> deserializer = buildStructDeserializer(descriptor, registry, ancestorTypes, sample);
      return new YoMCAPProtobufMessage(schema, descriptor, channelId, registry, deserializer);
   }

   private YoMCAPProtobufMessage(MCAPProtobufSchema schema, Descriptor descriptor, int channelId, YoRegistry registry,
                                 Consumer<com.google.protobuf.Message> deserializer)
   {
      this.schema = schema;
      this.descriptor = descriptor;
      this.channelId = channelId;
      this.registry = registry;
      this.deserializer = deserializer;
   }

   public Descriptor getDescriptor()
   {
      return descriptor;
   }

   @Override
   public MCAPSchema getSchema()
   {
      return schema;
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public int getChannelId()
   {
      return channelId;
   }

   @Override
   public void readMessage(Message message)
   {
      if (message.channelId() != channelId)
         throw new IllegalArgumentException("Expected channel ID: " + channelId + ", but received: " + message.channelId());

      com.google.protobuf.Message dynamicMessage;
      try
      {
         dynamicMessage = DynamicMessage.parseFrom(descriptor, message.messageData());
      }
      catch (InvalidProtocolBufferException e)
      {
         LogTools.error("Failed to parse protobuf message for channel: " + channelId + ", schema: " + descriptor.getFullName());
         throw new RuntimeException(e);
      }

      try
      {
         deserializer.accept(dynamicMessage);
      }
      catch (Exception e)
      {
         LogTools.error("Deserialization failed for message: " + registry.getName() + ", schema: " + descriptor.getFullName());
         throw e;
      }
   }

   /**
    * Builds a deserializer that, given the {@code com.google.protobuf.Message} instance for this struct level (or
    * {@code null} if this struct instance is absent, e.g. an unset optional sub-message or an array slot beyond the
    * actual repeated field count), updates every {@code YoVariable} built for {@code descriptor}'s fields.
    */
   private static Consumer<com.google.protobuf.Message> buildStructDeserializer(Descriptor descriptor, YoRegistry registry, Deque<String> ancestorTypes,
                                                                                 com.google.protobuf.Message sample)
   {
      List<Consumer<com.google.protobuf.Message>> fieldDeserializers = new ArrayList<>();

      for (FieldDescriptor fieldDescriptor : descriptor.getFields())
      {
         Consumer<com.google.protobuf.Message> fieldDeserializer = buildFieldDeserializer(fieldDescriptor, registry, ancestorTypes, sample);
         if (fieldDeserializer != null)
            fieldDeserializers.add(fieldDeserializer);
      }

      return message ->
      {
         for (Consumer<com.google.protobuf.Message> fieldDeserializer : fieldDeserializers)
            fieldDeserializer.accept(message);
      };
   }

   private static Consumer<com.google.protobuf.Message> buildFieldDeserializer(FieldDescriptor fieldDescriptor, YoRegistry registry,
                                                                                Deque<String> ancestorTypes, com.google.protobuf.Message sample)
   {
      if (fieldDescriptor.isMapField())
         return buildMapFieldDeserializer(fieldDescriptor, registry, ancestorTypes, sample);

      String fieldName = fieldDescriptor.getName();

      if (fieldDescriptor.isRepeated())
         return buildRepeatedFieldDeserializer(fieldDescriptor, fieldName, registry, ancestorTypes);

      return switch (fieldDescriptor.getJavaType())
      {
         case MESSAGE -> buildMessageFieldDeserializer(fieldDescriptor, fieldName, registry, ancestorTypes, sample);
         case ENUM -> buildEnumFieldDeserializer(fieldDescriptor, fieldName, registry);
         case STRING, BYTE_STRING -> null; // No YoVariable equivalent, same limitation as YoMCAPMessage's CDR "string" handling.
         default -> buildScalarFieldDeserializer(fieldDescriptor, fieldName, registry);
      };
   }

   private static Consumer<com.google.protobuf.Message> buildMessageFieldDeserializer(FieldDescriptor fieldDescriptor, String fieldName,
                                                                                       YoRegistry registry, Deque<String> ancestorTypes,
                                                                                       com.google.protobuf.Message sample)
   {
      Descriptor messageType = fieldDescriptor.getMessageType();
      if (ancestorTypes.contains(messageType.getFullName()))
      {
         LogTools.warn("Recursive protobuf field '" + fieldDescriptor.getFullName() + "' of type " + messageType.getFullName()
                       + " is not supported and will not be decoded.");
         return null;
      }

      YoRegistry fieldRegistry = new YoRegistry(fieldName);
      registry.addChild(fieldRegistry);

      com.google.protobuf.Message subSample = sample != null && sample.hasField(fieldDescriptor) ? (com.google.protobuf.Message) sample.getField(
            fieldDescriptor) : null;

      ancestorTypes.push(messageType.getFullName());
      Consumer<com.google.protobuf.Message> subDeserializer = buildStructDeserializer(messageType, fieldRegistry, ancestorTypes, subSample);
      ancestorTypes.pop();

      return message ->
      {
         com.google.protobuf.Message subMessage = message != null && message.hasField(fieldDescriptor) ?
               (com.google.protobuf.Message) message.getField(fieldDescriptor) : null;
         subDeserializer.accept(subMessage);
      };
   }

   /**
    * Builds a deserializer for a {@code map<string, Message>} field. Protobuf maps are wire-compatible with
    * {@code repeated MapEntry{key; value;}}, so, unlike every other field shape here, the set of {@code YoVariable}s
    * to build can't be determined from the descriptor alone - it depends on which keys actually show up in the data.
    * {@code sample} (one real message recorded on this channel, see {@link #newMessage(MCAPProtobufSchema, int,
    * YoRegistry, byte[])}) is used to discover that key set once, up front; the resulting per-key {@code YoVariable}s
    * are then live-updated on every subsequent message the same way as any other field. A key that shows up later but
    * wasn't in the sample is silently ignored, same trade-off {@code repeated} fields already make with their fixed
    * {@code maxLength}. Map value types other than {@code Message}, or a field with no sample data to resolve keys
    * from, fall back to the pre-existing skip+warning behavior.
    */
   private static Consumer<com.google.protobuf.Message> buildMapFieldDeserializer(FieldDescriptor fieldDescriptor, YoRegistry registry,
                                                                                   Deque<String> ancestorTypes, com.google.protobuf.Message sample)
   {
      String fieldName = fieldDescriptor.getName();
      FieldDescriptor keyField = fieldDescriptor.getMessageType().findFieldByName("key");
      FieldDescriptor valueField = fieldDescriptor.getMessageType().findFieldByName("value");

      if (keyField.getJavaType() != JavaType.STRING || valueField.getJavaType() != JavaType.MESSAGE)
      {
         LogTools.warn("Protobuf map fields are only supported for map<string, message> shapes, field '" + fieldDescriptor.getFullName()
                       + "' will not be decoded.");
         return null;
      }

      int sampleCount = sample == null ? 0 : sample.getRepeatedFieldCount(fieldDescriptor);
      if (sampleCount == 0)
      {
         LogTools.warn("No sample data available to resolve keys for protobuf map field '" + fieldDescriptor.getFullName() + "', it will not be decoded.");
         return null;
      }

      Descriptor valueType = valueField.getMessageType();
      if (ancestorTypes.contains(valueType.getFullName()))
      {
         LogTools.warn("Recursive protobuf map field '" + fieldDescriptor.getFullName() + "' of type " + valueType.getFullName()
                       + " is not supported and will not be decoded.");
         return null;
      }

      YoRegistry mapRegistry = new YoRegistry(fieldName);
      registry.addChild(mapRegistry);

      List<String> keys = new ArrayList<>();
      Map<String, Consumer<com.google.protobuf.Message>> entryDeserializers = new LinkedHashMap<>();

      ancestorTypes.push(valueType.getFullName());
      for (int i = 0; i < sampleCount; i++)
      {
         com.google.protobuf.Message entry = (com.google.protobuf.Message) sample.getRepeatedField(fieldDescriptor, i);
         String key = (String) entry.getField(keyField);
         if (entryDeserializers.containsKey(key))
            continue;

         com.google.protobuf.Message valueSample = (com.google.protobuf.Message) entry.getField(valueField);
         YoRegistry entryRegistry = new YoRegistry(key);
         mapRegistry.addChild(entryRegistry);
         keys.add(key);
         entryDeserializers.put(key, buildStructDeserializer(valueType, entryRegistry, ancestorTypes, valueSample));
      }
      ancestorTypes.pop();

      return message ->
      {
         Map<String, com.google.protobuf.Message> liveEntries = new HashMap<>();
         if (message != null)
         {
            int count = message.getRepeatedFieldCount(fieldDescriptor);
            for (int i = 0; i < count; i++)
            {
               com.google.protobuf.Message entry = (com.google.protobuf.Message) message.getRepeatedField(fieldDescriptor, i);
               liveEntries.put((String) entry.getField(keyField), (com.google.protobuf.Message) entry.getField(valueField));
            }
         }

         for (String key : keys)
            entryDeserializers.get(key).accept(liveEntries.get(key));
      };
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   private static Consumer<com.google.protobuf.Message> buildEnumFieldDeserializer(FieldDescriptor fieldDescriptor, String fieldName, YoRegistry registry)
   {
      String[] names = fieldDescriptor.getEnumType().getValues().stream().map(EnumValueDescriptor::getName).toArray(String[]::new);
      YoEnum yoEnum = new YoEnum(fieldName, "", registry, true, names);

      return message ->
      {
         if (message == null)
         {
            yoEnum.set(YoEnum.NULL_VALUE);
            return;
         }
         EnumValueDescriptor value = (EnumValueDescriptor) message.getField(fieldDescriptor);
         yoEnum.set(value.getIndex());
      };
   }

   private static Consumer<com.google.protobuf.Message> buildScalarFieldDeserializer(FieldDescriptor fieldDescriptor, String fieldName, YoRegistry registry)
   {
      return switch (fieldDescriptor.getType())
      {
         case UINT32, FIXED32 ->
         {
            YoLong v = new YoLong(fieldName, registry);
            yield message -> v.set(message == null ? 0L : Integer.toUnsignedLong((Integer) message.getField(fieldDescriptor)));
         }
         case INT32, SINT32, SFIXED32 ->
         {
            YoInteger v = new YoInteger(fieldName, registry);
            yield message -> v.set(message == null ? 0 : (Integer) message.getField(fieldDescriptor));
         }
         // TODO uint64/fixed64 deserializer: risk of overflow for values beyond Long.MAX_VALUE, same known limitation as YoMCAPMessage's CDR uint64 path.
         case UINT64, FIXED64, INT64, SINT64, SFIXED64 ->
         {
            YoLong v = new YoLong(fieldName, registry);
            yield message -> v.set(message == null ? 0L : (Long) message.getField(fieldDescriptor));
         }
         case FLOAT ->
         {
            YoDouble v = new YoDouble(fieldName, registry);
            yield message -> v.set(message == null ? Double.NaN : (Float) message.getField(fieldDescriptor));
         }
         case DOUBLE ->
         {
            YoDouble v = new YoDouble(fieldName, registry);
            yield message -> v.set(message == null ? Double.NaN : (Double) message.getField(fieldDescriptor));
         }
         case BOOL ->
         {
            YoBoolean v = new YoBoolean(fieldName, registry);
            yield message -> v.set(message != null && (Boolean) message.getField(fieldDescriptor));
         }
         default -> null; // STRING/BYTES/ENUM/MESSAGE/GROUP are routed to their own builders before reaching here.
      };
   }

   private static Consumer<com.google.protobuf.Message> buildRepeatedFieldDeserializer(FieldDescriptor fieldDescriptor, String fieldName,
                                                                                        YoRegistry registry, Deque<String> ancestorTypes)
   {
      int maxLength = UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH;

      if (fieldDescriptor.getJavaType() == JavaType.STRING || fieldDescriptor.getJavaType() == JavaType.BYTE_STRING)
         return null;

      LogTools.warn(
            "Unbounded arrays are not supported for type " + fieldDescriptor.getFullName() + ", limiting max length to " + maxLength);

      if (fieldDescriptor.getJavaType() == JavaType.MESSAGE)
         return buildRepeatedMessageFieldDeserializer(fieldDescriptor, fieldName, registry, ancestorTypes, maxLength);
      else if (fieldDescriptor.getJavaType() == JavaType.ENUM)
         return buildRepeatedEnumFieldDeserializer(fieldDescriptor, fieldName, registry, maxLength);
      else
         return buildRepeatedScalarFieldDeserializer(fieldDescriptor, fieldName, registry, maxLength);
   }

   private static Consumer<com.google.protobuf.Message> buildRepeatedMessageFieldDeserializer(FieldDescriptor fieldDescriptor, String fieldName,
                                                                                               YoRegistry registry, Deque<String> ancestorTypes,
                                                                                               int maxLength)
   {
      Descriptor messageType = fieldDescriptor.getMessageType();
      if (ancestorTypes.contains(messageType.getFullName()))
      {
         LogTools.warn("Recursive protobuf field '" + fieldDescriptor.getFullName() + "' of type " + messageType.getFullName()
                       + " is not supported and will not be decoded.");
         return null;
      }

      List<Consumer<com.google.protobuf.Message>> elementDeserializers = new ArrayList<>(maxLength);
      ancestorTypes.push(messageType.getFullName());
      for (int i = 0; i < maxLength; i++)
      {
         YoRegistry elementRegistry = new YoRegistry(fieldName + "[" + i + "]");
         registry.addChild(elementRegistry);
         // No sample propagated to array elements: map fields nested inside a repeated message field are a rare
         // enough shape that they can fall back to the existing skip+warning behavior.
         elementDeserializers.add(buildStructDeserializer(messageType, elementRegistry, ancestorTypes, null));
      }
      ancestorTypes.pop();

      return message ->
      {
         int count = message == null ? 0 : message.getRepeatedFieldCount(fieldDescriptor);
         if (count > maxLength)
            LogTools.warn("Received array of size: " + count + ", but expected size: " + maxLength + ", registry: " + registry);
         for (int i = 0; i < maxLength; i++)
         {
            com.google.protobuf.Message element = i < count ? (com.google.protobuf.Message) message.getRepeatedField(fieldDescriptor, i) : null;
            elementDeserializers.get(i).accept(element);
         }
      };
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   private static Consumer<com.google.protobuf.Message> buildRepeatedEnumFieldDeserializer(FieldDescriptor fieldDescriptor, String fieldName,
                                                                                            YoRegistry registry, int maxLength)
   {
      String[] names = fieldDescriptor.getEnumType().getValues().stream().map(EnumValueDescriptor::getName).toArray(String[]::new);
      YoEnum[] array = new YoEnum[maxLength];
      for (int i = 0; i < maxLength; i++)
         array[i] = new YoEnum(fieldName + "[" + i + "]", "", registry, true, names);

      return message ->
      {
         int count = message == null ? 0 : message.getRepeatedFieldCount(fieldDescriptor);
         if (count > maxLength)
            LogTools.warn("Received array of size: " + count + ", but expected size: " + maxLength + ", registry: " + registry);
         for (int i = 0; i < maxLength; i++)
         {
            if (i < count)
               array[i].set(((EnumValueDescriptor) message.getRepeatedField(fieldDescriptor, i)).getIndex());
            else
               array[i].set(YoEnum.NULL_VALUE);
         }
      };
   }

   private static Consumer<com.google.protobuf.Message> buildRepeatedScalarFieldDeserializer(FieldDescriptor fieldDescriptor, String fieldName,
                                                                                              YoRegistry registry, int maxLength)
   {
      return switch (fieldDescriptor.getType())
      {
         case UINT32, FIXED32 -> repeatedOf(YoLong::new,
                                            fieldName,
                                            registry,
                                            maxLength,
                                            fieldDescriptor,
                                            (v, o) -> v.set(Integer.toUnsignedLong((Integer) o)),
                                            v -> v.set(0));
         case INT32, SINT32, SFIXED32 -> repeatedOf(YoInteger::new, fieldName, registry, maxLength, fieldDescriptor, (v, o) -> v.set((Integer) o), v -> v.set(0));
         case UINT64, FIXED64, INT64, SINT64, SFIXED64 -> repeatedOf(YoLong::new,
                                                                     fieldName,
                                                                     registry,
                                                                     maxLength,
                                                                     fieldDescriptor,
                                                                     (v, o) -> v.set((Long) o),
                                                                     v -> v.set(0));
         case FLOAT -> repeatedOf(YoDouble::new,
                                  fieldName,
                                  registry,
                                  maxLength,
                                  fieldDescriptor,
                                  (v, o) -> v.set((Float) o),
                                  v -> v.set(Double.NaN));
         case DOUBLE -> repeatedOf(YoDouble::new,
                                   fieldName,
                                   registry,
                                   maxLength,
                                   fieldDescriptor,
                                   (v, o) -> v.set((Double) o),
                                   v -> v.set(Double.NaN));
         case BOOL -> repeatedOf(YoBoolean::new, fieldName, registry, maxLength, fieldDescriptor, (v, o) -> v.set((Boolean) o), v -> v.set(false));
         default -> null;
      };
   }

   @SuppressWarnings("unchecked")
   private static <T extends YoVariable> Consumer<com.google.protobuf.Message> repeatedOf(BiFunction<String, YoRegistry, T> builder, String fieldName,
                                                                                           YoRegistry registry, int maxLength,
                                                                                           FieldDescriptor fieldDescriptor, BiConsumer<T, Object> setter,
                                                                                           Consumer<T> resetter)
   {
      T[] array = (T[]) new YoVariable[maxLength];
      for (int i = 0; i < maxLength; i++)
         array[i] = builder.apply(fieldName + "[" + i + "]", registry);

      return message ->
      {
         int count = message == null ? 0 : message.getRepeatedFieldCount(fieldDescriptor);
         if (count > maxLength)
            LogTools.warn("Received array of size: " + count + ", but expected size: " + maxLength + ", registry: " + registry);
         for (int i = 0; i < maxLength; i++)
         {
            if (i < count)
               setter.accept(array[i], message.getRepeatedField(fieldDescriptor, i));
            else
               resetter.accept(array[i]);
         }
      };
   }
}

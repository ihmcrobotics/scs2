package us.ihmc.scs2.session.mcap;

import com.google.protobuf.Descriptors.Descriptor;
import com.google.protobuf.Descriptors.FieldDescriptor;
import com.google.protobuf.DynamicMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.specs.records.MutableMessage;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class YoMCAPProtobufMessageTest
{
   private static final int CHANNEL_ID = 7;

   @Test
   public void testDecodeAndReset()
   {
      MCAPProtobufSchema schema = ProtobufSchemaParser.loadSchema(ProtobufTestSchemas.ROOT_TYPE_NAME, 1, ProtobufTestSchemas.fileDescriptorSetBytes());
      Descriptor descriptor = schema.getDescriptor();
      YoRegistry registry = new YoRegistry("root");
      YoMCAPProtobufMessage yoMessage = YoMCAPProtobufMessage.newMessage(schema, CHANNEL_ID, registry);

      DynamicMessage populated = buildPopulatedRoot(descriptor);
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, populated.toByteArray()));

      assertEquals(1.5, ((YoDouble) registry.findVariable("d")).getValue());
      assertEquals(2.5f, ((YoDouble) registry.findVariable("f")).getValue(), 1e-6);
      assertEquals(-3, ((YoInteger) registry.findVariable("i32")).getValue());
      assertEquals(4L, ((YoLong) registry.findVariable("i64")).getValue());
      // uint32 widened into a YoLong so values beyond Integer.MAX_VALUE are representable.
      assertEquals(4_000_000_000L, ((YoLong) registry.findVariable("u32")).getValue());
      assertEquals(6L, ((YoLong) registry.findVariable("u64")).getValue());
      assertTrue(((YoBoolean) registry.findVariable("b")).getValue());
      // "s" (string) has no YoVariable equivalent, same as YoMCAPMessage's CDR string handling.
      assertNull(registry.findVariable("s"));

      // Status.ERROR = 5 declared 3rd (index 2): YoEnum ordinal is the declaration index, not the proto number.
      assertEquals(2, ((YoEnum<?>) registry.findVariable("status")).getOrdinal());

      YoRegistry innerRegistry = registry.findRegistry("inner");
      assertEquals(3.25, ((YoDouble) innerRegistry.findVariable("value")).getValue());

      assertEquals(10, ((YoInteger) registry.findVariable("numbers[0]")).getValue());
      assertEquals(20, ((YoInteger) registry.findVariable("numbers[1]")).getValue());
      assertEquals(30, ((YoInteger) registry.findVariable("numbers[2]")).getValue());
      // Slots beyond the actual repeated-field count are reset to default.
      assertEquals(0, ((YoInteger) registry.findVariable("numbers[3]")).getValue());

      YoRegistry nodeRegistry = registry.findRegistry("node");
      assertEquals(99, ((YoInteger) nodeRegistry.findVariable("id")).getValue());
      // Node.next : Node is self-referential; the recursion guard cuts it off, so no "next" sub-registry exists.
      assertNull(nodeRegistry.findRegistry("next"));

      // "tags" (map<string, int32>) is present in the wire bytes but has no YoVariable representation.
      assertNull(registry.findVariable("key"));

      // Re-decoding an empty message resets everything back to default: top-level scalars get proto3's own
      // zero-value default (getField never returns null), while fields inside an *absent* sub-message (like
      // "inner", unset here) go through the explicit reset path.
      DynamicMessage empty = DynamicMessage.newBuilder(descriptor).build();
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, empty.toByteArray()));
      assertEquals(0.0, ((YoDouble) registry.findVariable("d")).getValue());
      assertEquals(0, ((YoInteger) registry.findVariable("i32")).getValue());
      assertEquals(0, ((YoEnum<?>) registry.findVariable("status")).getOrdinal());
      assertTrue(Double.isNaN(((YoDouble) innerRegistry.findVariable("value")).getValue()));
      assertEquals(0, ((YoInteger) registry.findVariable("numbers[0]")).getValue());

      // "named_inners" (map<string, Inner>) has no sample data with this 3-arg newMessage() overload, so - like
      // "tags" above - it's present in the wire bytes but has no YoVariable representation.
      assertNull(registry.findRegistry("named_inners"));
   }

   @Test
   public void testMapFieldWithMessageValueDecodesWhenSampleProvided()
   {
      MCAPProtobufSchema schema = ProtobufSchemaParser.loadSchema(ProtobufTestSchemas.ROOT_TYPE_NAME, 1, ProtobufTestSchemas.fileDescriptorSetBytes());
      Descriptor descriptor = schema.getDescriptor();
      YoRegistry registry = new YoRegistry("root");

      DynamicMessage sample = buildNamedInners(descriptor, entry("a", 1.0), entry("b", 2.0));
      YoMCAPProtobufMessage yoMessage = YoMCAPProtobufMessage.newMessage(schema, CHANNEL_ID, registry, sample.toByteArray());

      // The sample is only used to discover which keys to build YoVariables for at construction time; it still
      // needs to be read like any other message for its values to actually populate those YoVariables.
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, sample.toByteArray()));

      YoRegistry namedInners = registry.findRegistry("named_inners");
      assertEquals(1.0, ((YoDouble) namedInners.getChild("a").findVariable("value")).getValue());
      assertEquals(2.0, ((YoDouble) namedInners.getChild("b").findVariable("value")).getValue());

      // A later message dropping "b" and introducing an unseen key "c": "b" resets to the absent-submessage
      // default, and "c" - not among the keys discovered from the sample - is silently ignored (same trade-off
      // "repeated" fields already make with their fixed maxLength).
      DynamicMessage next = buildNamedInners(descriptor, entry("a", 9.0), entry("c", 3.0));
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, next.toByteArray()));

      assertEquals(9.0, ((YoDouble) namedInners.getChild("a").findVariable("value")).getValue());
      assertTrue(Double.isNaN(((YoDouble) namedInners.getChild("b").findVariable("value")).getValue()));
      assertNull(namedInners.getChild("c"));
   }

   /**
    * "repeated Message" fields (e.g. {@code repeated Inner inners}) don't get one YoVariable per slot - each slot is
    * a whole sub-{@code YoRegistry} (built the same way a single nested-message field is, via
    * {@code buildStructDeserializer}), pre-allocated up to the 255-slot cap so a fixed set of YoVariables exists
    * regardless of how many elements any given message actually carries.
    */
   @Test
   public void testRepeatedMessageFieldPopulatesSubRegistriesAndResetsUnusedSlots()
   {
      MCAPProtobufSchema schema = ProtobufSchemaParser.loadSchema(ProtobufTestSchemas.ROOT_TYPE_NAME, 1, ProtobufTestSchemas.fileDescriptorSetBytes());
      Descriptor descriptor = schema.getDescriptor();
      YoRegistry registry = new YoRegistry("root");
      YoMCAPProtobufMessage yoMessage = YoMCAPProtobufMessage.newMessage(schema, CHANNEL_ID, registry);

      FieldDescriptor innersField = field(descriptor, "inners");
      Descriptor innerType = innersField.getMessageType();
      DynamicMessage.Builder root = DynamicMessage.newBuilder(descriptor);
      root.addRepeatedField(innersField, DynamicMessage.newBuilder(innerType).setField(field(innerType, "value"), 1.1).build());
      root.addRepeatedField(innersField, DynamicMessage.newBuilder(innerType).setField(field(innerType, "value"), 2.2).build());
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, root.build().toByteArray()));

      assertEquals(1.1, ((YoDouble) registry.findRegistry("inners[0]").findVariable("value")).getValue());
      assertEquals(2.2, ((YoDouble) registry.findRegistry("inners[1]").findVariable("value")).getValue());
      // Slot 2 exists (pre-allocated) but its backing sub-message is absent, so it goes through the same
      // absent-submessage reset path a single unset message field does (see "inner" -> NaN in testDecodeAndReset).
      assertTrue(Double.isNaN(((YoDouble) registry.findRegistry("inners[2]").findVariable("value")).getValue()));
   }

   /**
    * "repeated Enum" fields get one {@code YoEnum} per slot, same shape as a repeated scalar but using
    * {@code YoEnum.NULL_VALUE} (not a type-specific zero/NaN/false) as the "slot beyond the actual count" reset
    * value - enums have no zero-cost default the way a proto3 scalar does.
    */
   @Test
   public void testRepeatedEnumFieldTracksLiveCountAndResetsToNullValue()
   {
      MCAPProtobufSchema schema = ProtobufSchemaParser.loadSchema(ProtobufTestSchemas.ROOT_TYPE_NAME, 1, ProtobufTestSchemas.fileDescriptorSetBytes());
      Descriptor descriptor = schema.getDescriptor();
      YoRegistry registry = new YoRegistry("root");
      YoMCAPProtobufMessage yoMessage = YoMCAPProtobufMessage.newMessage(schema, CHANNEL_ID, registry);

      FieldDescriptor statusesField = field(descriptor, "statuses");
      DynamicMessage.Builder root = DynamicMessage.newBuilder(descriptor);
      root.addRepeatedField(statusesField, statusesField.getEnumType().findValueByName("OK"));
      root.addRepeatedField(statusesField, statusesField.getEnumType().findValueByName("ERROR"));
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, root.build().toByteArray()));

      assertEquals(1, ((YoEnum<?>) registry.findVariable("statuses[0]")).getOrdinal()); // OK is declared 2nd (index 1).
      assertEquals(2, ((YoEnum<?>) registry.findVariable("statuses[1]")).getOrdinal()); // ERROR is declared 3rd (index 2).
      assertEquals(YoEnum.NULL_VALUE, ((YoEnum<?>) registry.findVariable("statuses[2]")).getOrdinal());
   }

   /**
    * Every repeated-scalar type resets an unused slot to a different "default", matching what the equivalent
    * top-level scalar field resets to (see testDecodeAndReset): float/double -> NaN (proto3's own zero for a scalar
    * would be misleadingly indistinguishable from "actually zero"), bool -> false, and unsigned ints -> 0 but only
    * after being widened into a YoLong the same way the single "u32" field is - all three exercise a distinct
    * branch of repeatedOf's setter/resetter pair, none of which "numbers" (repeated int32, reset to plain 0) covers.
    */
   @Test
   public void testRepeatedScalarFieldTypesUseTypeAppropriateDefaults()
   {
      MCAPProtobufSchema schema = ProtobufSchemaParser.loadSchema(ProtobufTestSchemas.ROOT_TYPE_NAME, 1, ProtobufTestSchemas.fileDescriptorSetBytes());
      Descriptor descriptor = schema.getDescriptor();
      YoRegistry registry = new YoRegistry("root");
      YoMCAPProtobufMessage yoMessage = YoMCAPProtobufMessage.newMessage(schema, CHANNEL_ID, registry);

      FieldDescriptor doublesField = field(descriptor, "doubles");
      FieldDescriptor flagsField = field(descriptor, "flags");
      FieldDescriptor manyU32Field = field(descriptor, "many_u32");
      DynamicMessage.Builder root = DynamicMessage.newBuilder(descriptor);
      root.addRepeatedField(doublesField, 4.5);
      root.addRepeatedField(flagsField, true);
      root.addRepeatedField(manyU32Field, (int) 4_000_000_000L); // wraps to a negative int, widened back on decode.
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, root.build().toByteArray()));

      assertEquals(4.5, ((YoDouble) registry.findVariable("doubles[0]")).getValue());
      assertTrue(Double.isNaN(((YoDouble) registry.findVariable("doubles[1]")).getValue()));

      assertTrue(((YoBoolean) registry.findVariable("flags[0]")).getValue());
      assertFalse(((YoBoolean) registry.findVariable("flags[1]")).getValue());

      assertEquals(4_000_000_000L, ((YoLong) registry.findVariable("many_u32[0]")).getValue());
      assertEquals(0L, ((YoLong) registry.findVariable("many_u32[1]")).getValue());
   }

   /**
    * The 255-slot cap ({@code UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH}) is a hard pre-allocation limit, not a validated
    * bound - a message carrying more elements than that must not throw (a single overlong channel message would
    * otherwise take down the whole session), it should just warn and leave the overflow elements undecoded.
    */
   @Test
   public void testRepeatedFieldCountBeyondMaxLengthIsTruncatedNotThrown()
   {
      MCAPProtobufSchema schema = ProtobufSchemaParser.loadSchema(ProtobufTestSchemas.ROOT_TYPE_NAME, 1, ProtobufTestSchemas.fileDescriptorSetBytes());
      Descriptor descriptor = schema.getDescriptor();
      YoRegistry registry = new YoRegistry("root");
      YoMCAPProtobufMessage yoMessage = YoMCAPProtobufMessage.newMessage(schema, CHANNEL_ID, registry);

      FieldDescriptor numbersField = field(descriptor, "numbers");
      DynamicMessage.Builder root = DynamicMessage.newBuilder(descriptor);
      int elementCount = ProtobufSchemaParser.UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH + 45; // comfortably past the 255 cap.
      for (int i = 0; i < elementCount; i++)
         root.addRepeatedField(numbersField, i);

      // Must not throw despite the message carrying more elements than were pre-allocated.
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, root.build().toByteArray()));

      int lastSlot = ProtobufSchemaParser.UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH - 1;
      assertEquals(0, ((YoInteger) registry.findVariable("numbers[0]")).getValue());
      assertEquals(lastSlot, ((YoInteger) registry.findVariable("numbers[" + lastSlot + "]")).getValue());
      // No slot was ever allocated beyond the cap, regardless of how many elements the wire message actually had.
      assertNull(registry.findVariable("numbers[" + ProtobufSchemaParser.UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH + "]"));
   }

   /**
    * {@code Node.children : repeated Node} is a separate recursion-guard check from {@code Node.next : Node}
    * (plain message field) above it - {@code buildRepeatedMessageFieldDeserializer} never delegates to
    * {@code buildMessageFieldDeserializer}, so it re-checks {@code ancestorTypes} itself. Without that duplicate
    * check, a self-referential *repeated* field would recurse into building 255 sub-registries per level, forever.
    */
   @Test
   public void testRepeatedSelfReferentialMessageFieldIsSkippedByRecursionGuard()
   {
      MCAPProtobufSchema schema = ProtobufSchemaParser.loadSchema(ProtobufTestSchemas.ROOT_TYPE_NAME, 1, ProtobufTestSchemas.fileDescriptorSetBytes());
      Descriptor descriptor = schema.getDescriptor();
      YoRegistry registry = new YoRegistry("root");
      YoMCAPProtobufMessage yoMessage = YoMCAPProtobufMessage.newMessage(schema, CHANNEL_ID, registry);

      FieldDescriptor nodeField = field(descriptor, "node");
      Descriptor nodeType = nodeField.getMessageType();
      FieldDescriptor childrenField = field(nodeType, "children");
      DynamicMessage child = DynamicMessage.newBuilder(nodeType).setField(field(nodeType, "id"), 1).build();
      DynamicMessage node = DynamicMessage.newBuilder(nodeType)
                                          .setField(field(nodeType, "id"), 99)
                                          .addRepeatedField(childrenField, child)
                                          .build();
      DynamicMessage.Builder root = DynamicMessage.newBuilder(descriptor).setField(nodeField, node);
      yoMessage.readMessage(new MutableMessage(CHANNEL_ID, root.build().toByteArray()));

      YoRegistry nodeRegistry = registry.findRegistry("node");
      assertEquals(99, ((YoInteger) nodeRegistry.findVariable("id")).getValue());
      // The guard drops the field entirely - no "children[0]" sub-registry was ever created to populate.
      assertNull(nodeRegistry.findRegistry("children[0]"));
   }

   private record NamedInner(String key, double value)
   {
   }

   private static NamedInner entry(String key, double value)
   {
      return new NamedInner(key, value);
   }

   private static DynamicMessage buildNamedInners(Descriptor descriptor, NamedInner... entries)
   {
      DynamicMessage.Builder root = DynamicMessage.newBuilder(descriptor);
      FieldDescriptor namedInnersField = field(descriptor, "named_inners");
      Descriptor entryType = namedInnersField.getMessageType();
      Descriptor innerType = field(entryType, "value").getMessageType();

      for (NamedInner namedInner : entries)
      {
         DynamicMessage inner = DynamicMessage.newBuilder(innerType).setField(field(innerType, "value"), namedInner.value()).build();
         DynamicMessage mapEntry = DynamicMessage.newBuilder(entryType)
                                                 .setField(field(entryType, "key"), namedInner.key())
                                                 .setField(field(entryType, "value"), inner)
                                                 .build();
         root.addRepeatedField(namedInnersField, mapEntry);
      }

      return root.build();
   }

   private static DynamicMessage buildPopulatedRoot(Descriptor descriptor)
   {
      DynamicMessage.Builder root = DynamicMessage.newBuilder(descriptor);
      root.setField(field(descriptor, "d"), 1.5);
      root.setField(field(descriptor, "f"), 2.5f);
      root.setField(field(descriptor, "i32"), -3);
      root.setField(field(descriptor, "i64"), 4L);
      root.setField(field(descriptor, "u32"), (int) 4_000_000_000L); // wraps to a negative int, widened back on decode.
      root.setField(field(descriptor, "u64"), 6L);
      root.setField(field(descriptor, "b"), true);
      root.setField(field(descriptor, "s"), "ignored");

      FieldDescriptor statusField = field(descriptor, "status");
      root.setField(statusField, statusField.getEnumType().findValueByName("ERROR"));

      FieldDescriptor innerField = field(descriptor, "inner");
      DynamicMessage inner = DynamicMessage.newBuilder(innerField.getMessageType()).setField(field(innerField.getMessageType(), "value"), 3.25).build();
      root.setField(innerField, inner);

      FieldDescriptor numbersField = field(descriptor, "numbers");
      root.addRepeatedField(numbersField, 10);
      root.addRepeatedField(numbersField, 20);
      root.addRepeatedField(numbersField, 30);

      FieldDescriptor nodeField = field(descriptor, "node");
      DynamicMessage node = DynamicMessage.newBuilder(nodeField.getMessageType()).setField(field(nodeField.getMessageType(), "id"), 99).build();
      root.setField(nodeField, node);

      FieldDescriptor tagsField = field(descriptor, "tags");
      Descriptor tagsEntryType = tagsField.getMessageType();
      DynamicMessage tagEntry = DynamicMessage.newBuilder(tagsEntryType)
                                              .setField(field(tagsEntryType, "key"), "greeting")
                                              .setField(field(tagsEntryType, "value"), 1)
                                              .build();
      root.addRepeatedField(tagsField, tagEntry);

      return root.build();
   }

   private static FieldDescriptor field(Descriptor descriptor, String name)
   {
      FieldDescriptor field = descriptor.findFieldByName(name);
      if (field == null)
         throw new IllegalArgumentException("No such field: " + name + " in " + descriptor.getFullName());
      return field;
   }
}

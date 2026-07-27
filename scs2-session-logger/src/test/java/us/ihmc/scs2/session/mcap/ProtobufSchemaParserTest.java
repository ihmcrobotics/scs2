package us.ihmc.scs2.session.mcap;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ProtobufSchemaParserTest
{
   /**
    * Exercises {@link ProtobufSchemaParser#loadSchema} against the hand-built {@link ProtobufTestSchemas#ROOT_TYPE_NAME}
    * descriptor set, covering every field/message shape it's meant to handle: scalars (incl. unsigned widening),
    * a nested message, an enum with non-contiguous values, a repeated scalar, a self-referential message, and map
    * fields (expected to be dropped at this layer - see {@link YoMCAPProtobufMessageTest} for how maps are actually
    * decoded downstream in {@link YoMCAPProtobufMessage}).
    */
   @Test
   public void testLoadSchema()
   {
      MCAPProtobufSchema schema = ProtobufSchemaParser.loadSchema(ProtobufTestSchemas.ROOT_TYPE_NAME, 42, ProtobufTestSchemas.fileDescriptorSetBytes());

      assertEquals(ProtobufTestSchemas.ROOT_TYPE_NAME, schema.getName());
      assertEquals(42, schema.getId());
      assertEquals(ProtobufTestSchemas.ROOT_TYPE_NAME, schema.getDescriptor().getFullName());

      // "tags"/"named_inners" (the two map fields) are dropped: dynamic keys don't fit the static YoVariable
      // allocation model. Every other field (repeated or not) survives - 19 declared fields minus those 2 maps.
      List<MCAPSchemaField> fields = schema.getFields();
      assertEquals(17, fields.size());
      assertNull(findField(fields, "tags"));

      // Protobuf "repeated" fields have no static bound (unlike a ROS2 fixed-size array), so they're modeled as a
      // "vector" with a capped default max length rather than "array" - see UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH's
      // javadoc for why the two are distinguished at all.
      MCAPSchemaField numbers = findField(fields, "numbers");
      assertTrue(numbers.isVector());
      assertFalse(numbers.isArray());
      assertEquals(ProtobufSchemaParser.UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH, numbers.getMaxLength());

      // Type name keeps the unsigned/signed distinction ("uint32"/"uint64" vs "int32"/"int64") even though both
      // pairs are Java-represented the same size at this layer - YoMCAPProtobufMessage relies on that string to
      // decide whether to widen the decoded value into a YoLong (see its "uint32 widened" test case).
      MCAPSchemaField u32 = findField(fields, "u32");
      assertEquals("uint32", u32.getType());
      MCAPSchemaField u64 = findField(fields, "u64");
      assertEquals("uint64", u64.getType());

      // Both an enum and a nested message set isComplexType=true - it's the "resolve via subSchemaMap instead of a
      // YoVariable" flag, not an enum/message distinction. getEnumType()/getMessageType() diverge underneath, but
      // both funnel into the same flat map (see the subSchemaMap check below).
      MCAPSchemaField status = findField(fields, "status");
      assertTrue(status.isComplexType());
      assertEquals("test_proto.Status", status.getType());

      MCAPSchemaField inner = findField(fields, "inner");
      assertTrue(inner.isComplexType());
      assertEquals("test_proto.Inner", inner.getType());

      // Sub-schema map is flat (same convention ROS2SchemaParser/OMGIDLSchemaParser use), keyed by full proto name.
      Map<String, MCAPSchema> subSchemaMap = schema.getSubSchemaMap();
      assertTrue(subSchemaMap.containsKey("test_proto.Inner"));
      assertTrue(subSchemaMap.containsKey("test_proto.Status"));
      assertTrue(subSchemaMap.containsKey("test_proto.Node"));

      MCAPSchema statusSchema = subSchemaMap.get("test_proto.Status");
      assertTrue(statusSchema.isEnum());
      assertEquals(List.of("UNKNOWN", "OK", "ERROR"), List.of(statusSchema.getEnumConstants()));
      assertEquals(0L, statusSchema.getEnumValues()[0]);
      assertEquals(1L, statusSchema.getEnumValues()[1]);
      assertEquals(5L, statusSchema.getEnumValues()[2]); // non-contiguous value, preserved as-is.

      // Node is self-referential (Node.next : Node). Resolving via the flat map (instead of eagerly recursing)
      // is what lets this terminate instead of stack-overflowing while the schema is being built.
      MCAPSchema nodeSchema = subSchemaMap.get("test_proto.Node");
      MCAPSchemaField next = findField(nodeSchema.getFields(), "next");
      assertTrue(next.isComplexType());
      assertEquals("test_proto.Node", next.getType());
      assertTrue(subSchemaMap.containsKey(next.getType()));
   }

   private static MCAPSchemaField findField(List<MCAPSchemaField> fields, String name)
   {
      return fields.stream().filter(f -> f.getName().equals(name)).findFirst().orElse(null);
   }
}

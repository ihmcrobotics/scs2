package us.ihmc.scs2.session.mcap;

import com.google.protobuf.DescriptorProtos.DescriptorProto;
import com.google.protobuf.DescriptorProtos.EnumDescriptorProto;
import com.google.protobuf.DescriptorProtos.EnumValueDescriptorProto;
import com.google.protobuf.DescriptorProtos.FieldDescriptorProto;
import com.google.protobuf.DescriptorProtos.FieldDescriptorProto.Label;
import com.google.protobuf.DescriptorProtos.FieldDescriptorProto.Type;
import com.google.protobuf.DescriptorProtos.FileDescriptorProto;
import com.google.protobuf.DescriptorProtos.FileDescriptorSet;
import com.google.protobuf.DescriptorProtos.MessageOptions;

/**
 * Test-support fixture, not a test itself - no {@code @Test} methods here. {@link ProtobufSchemaParserTest} and
 * {@link YoMCAPProtobufMessageTest} both call {@link #fileDescriptorSetBytes()} to get the same schema bytes to test
 * against, so a field added here (with a name referenced from those tests) is immediately visible to both.
 * <p>
 * Hand-builds a small {@code FileDescriptorSet} (no {@code protoc}/build-time codegen needed) covering the
 * situations {@link ProtobufSchemaParser}/{@link YoMCAPProtobufMessage} need to handle: scalar fields (including
 * unsigned 32-bit widening), a nested message, an enum with non-contiguous values, a repeated scalar field, a map
 * field (expected to be skipped), and a self-referential message (expected to be cut off by the recursion guard).
 * <p>
 * Also covers every other "repeated" shape {@code YoMCAPProtobufMessage} builds a distinct deserializer for -
 * repeated message ({@code inners}), repeated enum ({@code statuses}), repeated types with a non-zero reset default
 * ({@code doubles} resets to NaN, {@code flags} to false), repeated unsigned widening ({@code many_u32}), and a
 * repeated field whose element type is self-referential ({@code Node.children}, a separate recursion-guard code
 * path from the plain {@code Node.next} field above it).
 * </p>
 * <p>
 * Equivalent to:
 * </p>
 * <pre>
 * syntax = "proto3";
 * package test_proto;
 *
 * enum Status { UNKNOWN = 0; OK = 1; ERROR = 5; }
 *
 * message Inner { double value = 1; }
 *
 * message Node
 * {
 *    int32 id = 1;
 *    Node next = 2; // self-referential
 *    repeated Node children = 3; // self-referential, repeated
 * }
 *
 * message Root
 * {
 *    double d = 1;
 *    float f = 2;
 *    int32 i32 = 3;
 *    int64 i64 = 4;
 *    uint32 u32 = 5;
 *    uint64 u64 = 6;
 *    bool b = 7;
 *    string s = 8;
 *    Status status = 9;
 *    Inner inner = 10;
 *    repeated int32 numbers = 11;
 *    Node node = 12;
 *    map&lt;string, int32&gt; tags = 13;
 *    map&lt;string, Inner&gt; named_inners = 14;
 *    repeated Inner inners = 15;
 *    repeated Status statuses = 16;
 *    repeated double doubles = 17;
 *    repeated bool flags = 18;
 *    repeated uint32 many_u32 = 19;
 * }
 * </pre>
 */
final class ProtobufTestSchemas
{
   static final String ROOT_TYPE_NAME = "test_proto.Root";

   private ProtobufTestSchemas()
   {
   }

   static byte[] fileDescriptorSetBytes()
   {
      return FileDescriptorSet.newBuilder().addFile(buildFile()).build().toByteArray();
   }

   private static FileDescriptorProto buildFile()
   {
      FileDescriptorProto.Builder file = FileDescriptorProto.newBuilder();
      file.setName("test.proto");
      file.setPackage("test_proto");
      file.setSyntax("proto3");

      EnumDescriptorProto.Builder status = EnumDescriptorProto.newBuilder();
      status.setName("Status");
      status.addValue(EnumValueDescriptorProto.newBuilder().setName("UNKNOWN").setNumber(0));
      status.addValue(EnumValueDescriptorProto.newBuilder().setName("OK").setNumber(1));
      status.addValue(EnumValueDescriptorProto.newBuilder().setName("ERROR").setNumber(5));
      file.addEnumType(status);

      DescriptorProto.Builder inner = DescriptorProto.newBuilder();
      inner.setName("Inner");
      inner.addField(scalarField("value", 1, Type.TYPE_DOUBLE, Label.LABEL_OPTIONAL));
      file.addMessageType(inner);

      DescriptorProto.Builder node = DescriptorProto.newBuilder();
      node.setName("Node");
      node.addField(scalarField("id", 1, Type.TYPE_INT32, Label.LABEL_OPTIONAL));
      node.addField(messageField("next", 2, ".test_proto.Node", Label.LABEL_OPTIONAL));
      // Repeated counterpart to "next": exercises buildRepeatedMessageFieldDeserializer's own recursion guard,
      // which is a separate check from the plain-message-field guard "next" above already exercises.
      node.addField(messageField("children", 3, ".test_proto.Node", Label.LABEL_REPEATED));
      file.addMessageType(node);

      DescriptorProto.Builder tagsEntry = DescriptorProto.newBuilder();
      tagsEntry.setName("TagsEntry");
      tagsEntry.addField(scalarField("key", 1, Type.TYPE_STRING, Label.LABEL_OPTIONAL));
      tagsEntry.addField(scalarField("value", 2, Type.TYPE_INT32, Label.LABEL_OPTIONAL));
      tagsEntry.setOptions(MessageOptions.newBuilder().setMapEntry(true).build());

      DescriptorProto.Builder namedInnersEntry = DescriptorProto.newBuilder();
      namedInnersEntry.setName("NamedInnersEntry");
      namedInnersEntry.addField(scalarField("key", 1, Type.TYPE_STRING, Label.LABEL_OPTIONAL));
      namedInnersEntry.addField(messageField("value", 2, ".test_proto.Inner", Label.LABEL_OPTIONAL));
      namedInnersEntry.setOptions(MessageOptions.newBuilder().setMapEntry(true).build());

      DescriptorProto.Builder root = DescriptorProto.newBuilder();
      root.setName("Root");
      root.addField(scalarField("d", 1, Type.TYPE_DOUBLE, Label.LABEL_OPTIONAL));
      root.addField(scalarField("f", 2, Type.TYPE_FLOAT, Label.LABEL_OPTIONAL));
      root.addField(scalarField("i32", 3, Type.TYPE_INT32, Label.LABEL_OPTIONAL));
      root.addField(scalarField("i64", 4, Type.TYPE_INT64, Label.LABEL_OPTIONAL));
      root.addField(scalarField("u32", 5, Type.TYPE_UINT32, Label.LABEL_OPTIONAL));
      root.addField(scalarField("u64", 6, Type.TYPE_UINT64, Label.LABEL_OPTIONAL));
      root.addField(scalarField("b", 7, Type.TYPE_BOOL, Label.LABEL_OPTIONAL));
      root.addField(scalarField("s", 8, Type.TYPE_STRING, Label.LABEL_OPTIONAL));
      root.addField(enumField("status", 9, ".test_proto.Status", Label.LABEL_OPTIONAL));
      root.addField(messageField("inner", 10, ".test_proto.Inner", Label.LABEL_OPTIONAL));
      root.addField(scalarField("numbers", 11, Type.TYPE_INT32, Label.LABEL_REPEATED));
      root.addField(messageField("node", 12, ".test_proto.Node", Label.LABEL_OPTIONAL));
      root.addField(messageField("tags", 13, ".test_proto.Root.TagsEntry", Label.LABEL_REPEATED));
      root.addField(messageField("named_inners", 14, ".test_proto.Root.NamedInnersEntry", Label.LABEL_REPEATED));
      // The remaining "repeated" shapes not otherwise covered above: repeated message, repeated enum, repeated
      // scalars whose unset-slot default isn't 0 (double -> NaN, bool -> false), and repeated unsigned widening.
      root.addField(messageField("inners", 15, ".test_proto.Inner", Label.LABEL_REPEATED));
      root.addField(enumField("statuses", 16, ".test_proto.Status", Label.LABEL_REPEATED));
      root.addField(scalarField("doubles", 17, Type.TYPE_DOUBLE, Label.LABEL_REPEATED));
      root.addField(scalarField("flags", 18, Type.TYPE_BOOL, Label.LABEL_REPEATED));
      root.addField(scalarField("many_u32", 19, Type.TYPE_UINT32, Label.LABEL_REPEATED));
      root.addNestedType(tagsEntry);
      root.addNestedType(namedInnersEntry);
      file.addMessageType(root);

      return file.build();
   }

   private static FieldDescriptorProto scalarField(String name, int number, Type type, Label label)
   {
      return FieldDescriptorProto.newBuilder().setName(name).setNumber(number).setType(type).setLabel(label).build();
   }

   private static FieldDescriptorProto messageField(String name, int number, String typeName, Label label)
   {
      return FieldDescriptorProto.newBuilder().setName(name).setNumber(number).setType(Type.TYPE_MESSAGE).setTypeName(typeName).setLabel(label).build();
   }

   private static FieldDescriptorProto enumField(String name, int number, String typeName, Label label)
   {
      return FieldDescriptorProto.newBuilder().setName(name).setNumber(number).setType(Type.TYPE_ENUM).setTypeName(typeName).setLabel(label).build();
   }
}

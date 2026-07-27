package us.ihmc.scs2.session.mcap;

import com.google.protobuf.DescriptorProtos.FileDescriptorProto;
import com.google.protobuf.DescriptorProtos.FileDescriptorSet;
import com.google.protobuf.Descriptors.Descriptor;
import com.google.protobuf.Descriptors.DescriptorValidationException;
import com.google.protobuf.Descriptors.EnumDescriptor;
import com.google.protobuf.Descriptors.EnumValueDescriptor;
import com.google.protobuf.Descriptors.FieldDescriptor;
import com.google.protobuf.Descriptors.FileDescriptor;
import com.google.protobuf.InvalidProtocolBufferException;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.specs.records.Schema;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Class used to represent a Java interpreter of a MCAP schema which encoding is "protobuf".
 * <p>
 * For this encoding, {@link Schema#data()} is a serialized {@code google.protobuf.FileDescriptorSet} containing the
 * target message's {@code FileDescriptorProto} plus all of its transitive dependencies (this is the MCAP protobuf
 * schema convention, see <a href="https://mcap.dev/spec/registry#protobuf">the MCAP spec</a>).
 * </p>
 * <p>
 * Unlike {@link ROS2SchemaParser}/{@link OMGIDLSchemaParser}, the {@link MCAPSchema} graph built here is used only
 * for schema bookkeeping/debug display; actual message decoding is done by {@link YoMCAPProtobufMessage} working
 * directly off the {@code com.google.protobuf.Descriptors.Descriptor} carried by the returned
 * {@link MCAPProtobufSchema}, since protobuf's wire format (tag/length-value, decoded via reflection) has nothing in
 * common with the CDR cursor {@link us.ihmc.scs2.session.mcap.encoding.CDRDeserializer} reads.
 * </p>
 */
public class ProtobufSchemaParser
{
   /**
    * Protobuf {@code repeated} fields have no static bound, same situation as ROS2's unbounded arrays.
    *
    * @see ROS2SchemaParser#UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH
    */
   public static final int UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH = ROS2SchemaParser.UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH;

   public static MCAPProtobufSchema loadSchema(Schema mcapSchema)
   {
      return loadSchema(mcapSchema.name(), mcapSchema.id(), mcapSchema.data().array());
   }

   /**
    * Loads a schema from the given data.
    *
    * @param name the name of the schema, i.e. the fully-qualified name of the target message type.
    * @param id   the ID of the schema.
    * @param data the data of the schema, expected to be a serialized {@code google.protobuf.FileDescriptorSet}.
    * @return the loaded schema.
    */
   public static MCAPProtobufSchema loadSchema(String name, int id, byte[] data)
   {
      FileDescriptorSet fileDescriptorSet;
      try
      {
         fileDescriptorSet = FileDescriptorSet.parseFrom(data);
      }
      catch (InvalidProtocolBufferException e)
      {
         throw new RuntimeException("Failed to parse FileDescriptorSet for schema: " + name, e);
      }

      Map<String, FileDescriptorProto> protoByFileName = new HashMap<>();
      for (FileDescriptorProto proto : fileDescriptorSet.getFileList())
         protoByFileName.put(proto.getName(), proto);

      Map<String, FileDescriptor> builtFiles = new HashMap<>();
      for (String fileName : protoByFileName.keySet())
         buildFileDescriptor(fileName, protoByFileName, builtFiles);

      Descriptor targetDescriptor = null;
      for (FileDescriptor fileDescriptor : builtFiles.values())
      {
         targetDescriptor = findMessageType(fileDescriptor.getMessageTypes(), name);
         if (targetDescriptor != null)
            break;
      }

      if (targetDescriptor == null)
         throw new IllegalArgumentException("Could not find message type: " + name + " in schema's FileDescriptorSet");

      Map<String, MCAPSchema> subSchemaMap = new LinkedHashMap<>();
      collectMessageSchema(targetDescriptor, subSchemaMap);
      List<MCAPSchemaField> rootFields = subSchemaMap.get(targetDescriptor.getFullName()).getFields();

      return new MCAPProtobufSchema(name, id, rootFields, subSchemaMap, targetDescriptor);
   }

   /**
    * Builds the {@link FileDescriptor} for {@code fileName}, first recursively building its dependencies (protobuf-java
    * requires a file's dependencies to already be built before it can be built itself). Memoizes into {@code built}.
    */
   private static FileDescriptor buildFileDescriptor(String fileName, Map<String, FileDescriptorProto> protoByFileName, Map<String, FileDescriptor> built)
   {
      FileDescriptor existing = built.get(fileName);
      if (existing != null)
         return existing;

      FileDescriptorProto proto = protoByFileName.get(fileName);
      if (proto == null)
         throw new IllegalStateException("Missing FileDescriptorProto for dependency: " + fileName);

      FileDescriptor[] dependencies = new FileDescriptor[proto.getDependencyCount()];
      for (int i = 0; i < dependencies.length; i++)
         dependencies[i] = buildFileDescriptor(proto.getDependency(i), protoByFileName, built);

      FileDescriptor fileDescriptor;
      try
      {
         fileDescriptor = FileDescriptor.buildFrom(proto, dependencies);
      }
      catch (DescriptorValidationException e)
      {
         throw new RuntimeException("Failed to build FileDescriptor: " + fileName, e);
      }

      built.put(fileName, fileDescriptor);
      return fileDescriptor;
   }

   private static Descriptor findMessageType(List<Descriptor> candidates, String fullName)
   {
      for (Descriptor candidate : candidates)
      {
         if (candidate.getFullName().equals(fullName))
            return candidate;
         Descriptor nested = findMessageType(candidate.getNestedTypes(), fullName);
         if (nested != null)
            return nested;
      }
      return null;
   }

   /**
    * Populates {@code subSchemaMap} (flat, keyed by protobuf full type name) with an entry for {@code descriptor} and
    * every message/enum type reachable from its fields, mirroring the flat sub-schema convention
    * {@link ROS2SchemaParser}/{@link OMGIDLSchemaParser} use so the rest of the pipeline needs no changes.
    * <p>
    * The entry for {@code descriptor} is inserted into the map <em>before</em> its fields are walked, so a
    * self-referential message (legal in protobuf, unlike ROS2 .msg) resolves back to the in-progress entry instead of
    * recursing forever.
    * </p>
    */
   private static void collectMessageSchema(Descriptor descriptor, Map<String, MCAPSchema> subSchemaMap)
   {
      String fullName = descriptor.getFullName();
      if (subSchemaMap.containsKey(fullName))
         return;

      List<MCAPSchemaField> fields = new ArrayList<>();
      subSchemaMap.put(fullName, new MCAPSchema(fullName, -1, fields, null));

      for (FieldDescriptor fieldDescriptor : descriptor.getFields())
      {
         MCAPSchemaField field = buildField(fieldDescriptor, subSchemaMap);
         if (field != null)
            fields.add(field);
      }
   }

   private static MCAPSchema collectEnumSchema(EnumDescriptor enumDescriptor, Map<String, MCAPSchema> subSchemaMap)
   {
      String fullName = enumDescriptor.getFullName();
      MCAPSchema existing = subSchemaMap.get(fullName);
      if (existing != null)
         return existing;

      List<EnumValueDescriptor> values = enumDescriptor.getValues();
      String[] enumConstants = values.stream().map(EnumValueDescriptor::getName).toArray(String[]::new);
      long[] enumValues = values.stream().mapToLong(EnumValueDescriptor::getNumber).toArray();
      MCAPSchema enumSchema = new MCAPSchema(fullName, -1, enumConstants, enumValues);
      subSchemaMap.put(fullName, enumSchema);
      return enumSchema;
   }

   private static MCAPSchemaField buildField(FieldDescriptor fieldDescriptor, Map<String, MCAPSchema> subSchemaMap)
   {
      if (fieldDescriptor.isMapField())
      {
         LogTools.warn("Protobuf map fields are not supported, field '" + fieldDescriptor.getFullName() + "' will not be decoded.");
         return null;
      }

      MCAPSchemaField field = new MCAPSchemaField();
      field.setName(fieldDescriptor.getName());

      switch (fieldDescriptor.getJavaType())
      {
         case MESSAGE ->
         {
            Descriptor messageType = fieldDescriptor.getMessageType();
            collectMessageSchema(messageType, subSchemaMap);
            field.setType(messageType.getFullName());
            field.setComplexType(true);
         }
         case ENUM ->
         {
            collectEnumSchema(fieldDescriptor.getEnumType(), subSchemaMap);
            field.setType(fieldDescriptor.getEnumType().getFullName());
            field.setComplexType(true);
         }
         default -> field.setType(scalarTypeName(fieldDescriptor));
      }

      if (fieldDescriptor.isRepeated())
      {
         field.setArray(false);
         field.setVector(true);
         field.setMaxLength(UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH);
         field.setComplexType(true);
         LogTools.warn("Unbounded arrays are not supported for type " + field.getType() + ", limiting max length to " + UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH);
      }
      else
      {
         field.setArray(false);
         field.setVector(false);
         field.setMaxLength(-1);
      }

      return field;
   }

   private static String scalarTypeName(FieldDescriptor fieldDescriptor)
   {
      return switch (fieldDescriptor.getType())
      {
         case INT32, SINT32, SFIXED32 -> "int32";
         case UINT32, FIXED32 -> "uint32";
         case INT64, SINT64, SFIXED64 -> "int64";
         case UINT64, FIXED64 -> "uint64";
         case FLOAT -> "float";
         case DOUBLE -> "double";
         case BOOL -> "bool";
         case STRING -> "string";
         case BYTES -> "bytes";
         default -> fieldDescriptor.getType().name().toLowerCase();
      };
   }
}

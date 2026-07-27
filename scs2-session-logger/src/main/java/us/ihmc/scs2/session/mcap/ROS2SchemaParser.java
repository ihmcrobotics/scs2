package us.ihmc.scs2.session.mcap;

import us.ihmc.jros2.parser.MsgContext;
import us.ihmc.jros2.parser.field.InterfaceField;
import us.ihmc.jros2.parser.field.InterfaceFieldParsingException;
import us.ihmc.jros2.parser.msgdeps.MsgDepsContext;
import us.ihmc.jros2.parser.msgdeps.MsgDepsParser;
import us.ihmc.jros2.parser.util.BuiltinTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.specs.records.Schema;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Class used to represent a Java interpreter of a MCAP schema which encoding is "ros2msg".
 * This schema resembles much of ROS2 messages.
 * <p>
 * The field-level tokenizing (splitting the bundled schema into its dependency blocks and parsing each field line)
 * is delegated to {@code jros2-parser} ({@link MsgDepsParser}/{@link InterfaceField}), which implements the same
 * bundled "ros2msg" convention (see <a href="https://mcap.dev/spec/registry#ros2msg">the MCAP spec</a>). This class
 * is responsible for converting that into the {@link MCAPSchema} graph the rest of the MCAP pipeline
 * ({@link us.ihmc.scs2.session.mcap.encoding.CDRDeserializer} and friends) walks at runtime, which jros2 has no
 * equivalent of since it's built to feed Java source-code generation.
 * </p>
 */
public class ROS2SchemaParser
{
   /**
    * ROS2 allows declaring unbounded arrays, e.g. "float64[]", which have no max length in the schema.
    * Since the max length is used to statically allocate the backing {@code YoVariable}s, unbounded arrays are treated as bounded with this default length.
    */
   public static final int UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH = 255;

   /**
    * Loads a schema from the given {@link Schema}.
    *
    * @param mcapSchema the schema to load.
    * @return the loaded schema.
    */
   public static MCAPSchema loadSchema(Schema mcapSchema)
   {
      return loadSchema(mcapSchema.name(), mcapSchema.id(), mcapSchema.data().array());
   }

   /**
    * Loads a schema from the given data.
    *
    * @param name the name of the schema.
    * @param id   the ID of the schema.
    * @param data the data of the schema, expected to be a {@link String} using UTF-8 encoding.
    * @return the loaded schema.
    */
   public static MCAPSchema loadSchema(String name, int id, byte[] data)
   {
      String schemasBundledString = new String(data, StandardCharsets.UTF_8);

      MsgDepsContext msgDepsContext;
      try
      {
         msgDepsContext = MsgDepsParser.parseMsgDeps(schemasBundledString, toPackageResourceName(name));
      }
      catch (InterfaceFieldParsingException e)
      {
         throw new RuntimeException("Failed to parse ros2msg schema for " + name, e);
      }

      List<MCAPSchemaField> fields = new ArrayList<>();
      List<MCAPSchemaField> constants = new ArrayList<>();
      splitFieldsAndConstants(msgDepsContext.getFields().values(), fields, constants);

      LinkedHashMap<String, MCAPSchema> subSchemaMap = new LinkedHashMap<>();
      // Package name of the schema each sub-schema was declared in, keyed the same as subSchemaMap. Needed to
      // resolve bare (same-package) type references declared within that sub-schema.
      Map<String, String> subSchemaPackageNames = new LinkedHashMap<>();

      for (Map.Entry<String, MsgContext> dependency : msgDepsContext.getDependencies().entrySet())
      {
         String subName = dependency.getKey();
         MsgContext dependencyContext = dependency.getValue();

         List<MCAPSchemaField> subFields = new ArrayList<>();
         List<MCAPSchemaField> subConstants = new ArrayList<>();
         splitFieldsAndConstants(dependencyContext.getFields().values(), subFields, subConstants);

         MCAPSchema subSchema = new MCAPSchema(subName, -1, subFields, null);
         subSchema.getStaticFields().addAll(subConstants);
         registerEnumFields(subFields, subConstants);
         subSchemaMap.put(subName, subSchema);
         subSchemaPackageNames.put(subName, dependencyContext.getPackageName());
      }

      // Update the fields to indicate whether they are complex types or not, and register enum fields.
      // Short type names (no '/') are resolved to their fully-qualified key in the sub-schema map.
      registerEnumFields(fields, constants);
      for (MCAPSchemaField field : fields)
      {
         field.setType(resolveTypeName(field.getType(), msgDepsContext.getPackageName(), subSchemaMap));
         if (subSchemaMap.containsKey(field.getType()))
            field.setComplexType(true);
      }
      for (Map.Entry<String, MCAPSchema> subSchemaEntry : subSchemaMap.entrySet())
      {
         String enclosingPackageName = subSchemaPackageNames.get(subSchemaEntry.getKey());
         for (MCAPSchemaField subField : subSchemaEntry.getValue().getFields())
         {
            subField.setType(resolveTypeName(subField.getType(), enclosingPackageName, subSchemaMap));
            if (subSchemaMap.containsKey(subField.getType()))
               subField.setComplexType(true);
         }
      }

      MCAPSchema result = new MCAPSchema(name, id, fields, subSchemaMap);
      result.getStaticFields().addAll(constants);
      return result;
   }

   /**
    * jros2's {@link MsgDepsParser} requires a "package/Resource" name (exactly one '/'), but MCAP schema names use
    * the ROS2 "package/msg/Resource" convention. Strips the middle segment(s) to fit jros2's expectation.
    */
   private static String toPackageResourceName(String schemaName)
   {
      String[] parts = schemaName.split("/");
      if (parts.length == 2)
         return schemaName;
      if (parts.length > 2)
         return parts[0] + "/" + parts[parts.length - 1];
      return "unknown/" + schemaName;
   }

   /**
    * Splits a collection of jros2 fields into data fields and constant definitions, mirroring the previous
    * hand-rolled distinction between a field declaration and a constant declaration ("TYPE NAME = VALUE").
    */
   private static void splitFieldsAndConstants(Collection<InterfaceField> interfaceFields, List<MCAPSchemaField> fields, List<MCAPSchemaField> constants)
   {
      for (InterfaceField interfaceField : interfaceFields)
      {
         if (interfaceField.getConstantValue() != null)
            constants.add(toConstantField(interfaceField));
         else
            fields.add(toDataField(interfaceField));
      }
   }

   private static MCAPSchemaField toConstantField(InterfaceField interfaceField)
   {
      MCAPSchemaField field = new MCAPSchemaField();
      field.setType(interfaceField.getType());
      field.setName(interfaceField.getName());
      field.setDefaultValue(interfaceField.getConstantValue());
      return field;
   }

   private static MCAPSchemaField toDataField(InterfaceField interfaceField)
   {
      MCAPSchemaField field = new MCAPSchemaField();
      field.setType(interfaceField.getType());
      field.setName(interfaceField.getName());

      if (interfaceField.isArray())
      {
         if (interfaceField.isUnbounded())
         {
            field.setArray(false);
            field.setVector(true);
            field.setMaxLength(UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH);
            LogTools.warn("Unbounded arrays are not supported for type " + interfaceField.getType() + ", limiting max length to "
                          + UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH);
         }
         else if (interfaceField.isUpperBounded())
         {
            field.setArray(false);
            field.setVector(true);
            field.setMaxLength(interfaceField.getLength());
         }
         else
         {
            field.setArray(true);
            field.setVector(false);
            field.setMaxLength(interfaceField.getLength());
         }
         // Set unconditionally for any bracketed (array or vector) field, even if the element type is primitive.
         field.setComplexType(true);
      }
      else
      {
         field.setArray(false);
         field.setVector(false);
         field.setMaxLength(-1);
      }
      return field;
   }

   /**
    * For each data field whose integer type matches a group of constants declared in the same schema,
    * creates a synthetic enum {@link MCAPSchema} and attaches it to the field so the log viewer can
    * display the named constant instead of a raw integer.
    */
   private static void registerEnumFields(List<MCAPSchemaField> fields, List<MCAPSchemaField> constants)
   {
      if (constants.isEmpty())
         return;

      // Group integer-type constants by their primitive type.
      Map<String, List<MCAPSchemaField>> constantsByType = constants.stream()
                                                                    .filter(c -> isIntegerType(c.getType()))
                                                                    .collect(Collectors.groupingBy(MCAPSchemaField::getType,
                                                                                                   LinkedHashMap::new,
                                                                                                   Collectors.toList()));
      if (constantsByType.isEmpty())
         return;

      for (MCAPSchemaField field : fields)
      {
         List<MCAPSchemaField> matchingConstants = constantsByType.get(field.getType());
         if (matchingConstants == null || matchingConstants.isEmpty())
            continue;

         // Sort by integer value so YoEnum ordinals are assigned in ascending value order.
         List<MCAPSchemaField> sorted = new ArrayList<>(matchingConstants);
         sorted.sort(Comparator.comparingLong(c -> parseConstantValue(c.getDefaultValue())));

         String[] enumConstantNames = sorted.stream().map(MCAPSchemaField::getName).toArray(String[]::new);
         long[] enumValues = sorted.stream().mapToLong(c -> parseConstantValue(c.getDefaultValue())).toArray();

         field.setEnumSchema(new MCAPSchema(field.getType(), -1, enumConstantNames, enumValues));
      }
   }

   private static boolean isIntegerType(String type)
   {
      return switch (type)
      {
         case "bool", "float32", "float64", "string", "wstring" -> false;
         default -> BuiltinTools.isBuiltinType(type);
      };
   }

   private static long parseConstantValue(String value)
   {
      if (value == null)
         return 0;
      try
      {
         return Long.parseLong(value.trim());
      }
      catch (NumberFormatException e)
      {
         return 0;
      }
   }

   /**
    * Resolves a short type name (no '/') to its fully-qualified key in the sub-schema map using the ROS2 .msg
    * convention that a bare type name refers to a message in the same package as the enclosing schema, i.e. the
    * package jros2 already resolved for that schema (see {@code InterfaceContext.getPackageName()}). Returns the
    * original name unchanged if it already contains '/' or the qualified name isn't a known sub-schema (e.g. it's
    * a builtin primitive).
    */
   private static String resolveTypeName(String type, String enclosingPackageName, Map<String, MCAPSchema> subSchemaMap)
   {
      if (type.contains("/") || type.contains("::"))
         return type;
      String qualifiedType = enclosingPackageName + "/" + type;
      return subSchemaMap.containsKey(qualifiedType) ? qualifiedType : type;
   }
}

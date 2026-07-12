package us.ihmc.scs2.session.mcap;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.specs.records.Schema;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Class used to represent a Java interpreter of a MCAP schema which encoding is "ros2msg".
 * This schema resembles much of ROS2 messages.
 */
public class ROS2SchemaParser
{
   public static final String SUB_SCHEMA_SEPARATOR_REGEX = "\n(=+)\n";
   public static final String SUB_SCHEMA_PREFIX = "MSG: ";
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
      String schemasBundledString = new String(data);
      schemasBundledString = schemasBundledString.replaceAll("\r\n", "\n"); // To handle varying declaration of a new line.
      String[] schemasStrings = schemasBundledString.split(SUB_SCHEMA_SEPARATOR_REGEX);

      List<MCAPSchemaField> fields = new ArrayList<>();
      List<MCAPSchemaField> constants = new ArrayList<>();
      parseFieldsAndConstants(schemasStrings[0], fields, constants);

      LinkedHashMap<String, MCAPSchema> subSchemaMap = new LinkedHashMap<>();

      for (int i = 1; i < schemasStrings.length; i++)
      {
         String schemaString = schemasStrings[i];

         int firstNewLineCharacter = schemaString.indexOf("\n");
         String firstLine = schemaString.substring(0, firstNewLineCharacter);
         String subName = firstLine.replace(SUB_SCHEMA_PREFIX, "").trim();

         List<MCAPSchemaField> subFields = new ArrayList<>();
         List<MCAPSchemaField> subConstants = new ArrayList<>();
         parseFieldsAndConstants(schemaString.substring(firstNewLineCharacter + 1), subFields, subConstants);

         MCAPSchema subSchema = new MCAPSchema(subName, -1, subFields, null);
         subSchema.getStaticFields().addAll(subConstants);
         registerEnumFields(subFields, subConstants);
         subSchemaMap.put(subName, subSchema);
      }

      // Update the fields to indicate whether they are complex types or not, and register enum fields.
      // Short type names (no '/') are resolved to their fully-qualified key in the sub-schema map.
      registerEnumFields(fields, constants);
      for (MCAPSchemaField field : fields)
      {
         field.setType(resolveTypeName(field.getType(), subSchemaMap));
         if (subSchemaMap.containsKey(field.getType()))
            field.setComplexType(true);
      }
      for (MCAPSchema subSchema : subSchemaMap.values())
      {
         for (MCAPSchemaField subField : subSchema.getFields())
         {
            subField.setType(resolveTypeName(subField.getType(), subSchemaMap));
            if (subSchemaMap.containsKey(subField.getType()))
               subField.setComplexType(true);
         }
      }

      MCAPSchema result = new MCAPSchema(name, id, fields, subSchemaMap);
      result.getStaticFields().addAll(constants);
      return result;
   }

   /**
    * Splits the lines of a schema block into data fields and constant definitions.
    */
   private static void parseFieldsAndConstants(String schemaBlock, List<MCAPSchemaField> fields, List<MCAPSchemaField> constants)
   {
      schemaBlock.lines()
                 .filter(line -> !line.isBlank() && !line.trim().startsWith("#"))
                 .forEach(line ->
                          {
                             if (isConstantDefinition(line))
                                constants.add(parseConstantField(line));
                             else
                                fields.add(parseMCAPSchemaField(line));
                          });
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

   private static boolean isConstantDefinition(String line)
   {
      int commentIndex = line.indexOf('#');
      String effectiveLine = commentIndex >= 0 ? line.substring(0, commentIndex).trim() : line;
      int firstSpace = effectiveLine.indexOf(' ');
      if (firstSpace < 0)
         return false;
      // ROS2 msg field declarations ("TYPE NAME", optionally with array brackets) never contain '=';
      // only constant declarations ("TYPE NAME=VALUE" or "TYPE NAME = VALUE") do, so presence of '=' after
      // the type is sufficient. Previously this also required '=' to appear before any space, which broke
      // on the "NAME = VALUE" spacing style (e.g. "uint8 UNKNOWN = 0"): the space between NAME and '='
      // came first, so those constants were misclassified as regular fields, corrupting CDR deserialization
      // for every message using the "<Type>Enum <field>_foxglove_enum" pattern.
      String remaining = effectiveLine.substring(firstSpace + 1).trim();
      return remaining.indexOf('=') >= 0;
   }

   private static MCAPSchemaField parseConstantField(String line)
   {
      int commentIndex = line.indexOf('#');
      if (commentIndex >= 0)
         line = line.substring(0, commentIndex).trim();
      int firstSpace = line.indexOf(' ');
      String type = line.substring(0, firstSpace).trim();
      String remaining = line.substring(firstSpace + 1).trim();
      int equalsIndex = remaining.indexOf('=');
      String constantName = remaining.substring(0, equalsIndex).trim();
      String constantValue = remaining.substring(equalsIndex + 1).trim();

      MCAPSchemaField field = new MCAPSchemaField();
      field.setType(type);
      field.setName(constantName);
      field.setDefaultValue(constantValue);
      return field;
   }

   private static boolean isIntegerType(String type)
   {
      return switch (type)
      {
         case "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "byte", "char" -> true;
         default -> false;
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
    * Resolves a short type name (no '/') to its fully-qualified key in the sub-schema map by matching
    * the last path segment. Returns the original name unchanged if it already contains '/' or no match
    * is found.
    */
   private static String resolveTypeName(String type, Map<String, MCAPSchema> subSchemaMap)
   {
      if (subSchemaMap.containsKey(type))
         return type;
      if (type.contains("/") || type.contains("::"))
         return type;
      for (String key : subSchemaMap.keySet())
      {
         int lastSlash = key.lastIndexOf('/');
         String shortName = lastSlash >= 0 ? key.substring(lastSlash + 1) : key;
         if (shortName.equals(type))
            return key;
      }
      return type;
   }

   public static MCAPSchemaField parseMCAPSchemaField(String line)
   {
      int inlineCommentIndex = line.indexOf('#');
      if (inlineCommentIndex >= 0)
         line = line.substring(0, inlineCommentIndex).trim();
      MCAPSchemaField field = new MCAPSchemaField();
      field.setType(line.substring(0, line.indexOf(' ')).trim());
      field.setName(line.substring(line.indexOf(' ') + 1).trim().split("\\s+")[0]);

      int lBracketIndex = field.getType().indexOf('[');
      int rBracketIndex = field.getType().indexOf(']');

      if (lBracketIndex < rBracketIndex)
      {
         String maxLengthStr = field.getType().substring(lBracketIndex + 1, rBracketIndex);
         if (maxLengthStr.isEmpty())
         {
            // Unbounded array, e.g. "float64[]": no max length declared in the schema.
            field.setArray(false);
            field.setVector(true);
            maxLengthStr = Integer.toString(UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH);
            LogTools.warn("Unbounded arrays are not supported for type " + field.getType() + ", limiting max length to "
                          + UNBOUNDED_ARRAY_DEFAULT_MAX_LENGTH);
         }
         else if (maxLengthStr.startsWith("<="))
         {
            field.setArray(false);
            field.setVector(true);
            maxLengthStr = maxLengthStr.substring(2);
         }
         else
         {
            field.setArray(true);
            field.setVector(false);
         }
         field.setComplexType(true);
         try
         {
            field.setMaxLength(Integer.parseInt(maxLengthStr));
         }
         catch (NumberFormatException e)
         {
            // The length is probably defined as a maximum length "array[<=54]"
            maxLengthStr = maxLengthStr.replace("<=", "");
            field.setMaxLength(Integer.parseInt(maxLengthStr));
         }
         field.setType(field.getType().substring(0, lBracketIndex));
      }
      else
      {
         field.setArray(false);
         field.setVector(false);
         field.setMaxLength(-1);
      }
      return field;
   }
}

package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Class used to represent a Java interpreter of a MCAP schema which encoding is "ros2msg".
 * This schema resembles much of ROS2 messages.
 */
public class ROS2SchemaParser
{
   public static final String SUB_SCHEMA_SEPARATOR_REGEX = "\n(=+)\n";
   public static final String SUB_SCHEMA_PREFIX = "MSG: fastdds/";

   /**
    * Loads a schema from the given {@link MCAP.Schema}.
    *
    * @param mcapSchema the schema to load.
    * @return the loaded schema.
    */
   public static MCAPSchema loadSchema(MCAP.Schema mcapSchema)
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

      List<MCAPSchemaField> fields = schemasStrings[0].lines().map(ROS2SchemaParser::parseMCAPSchemaField).collect(Collectors.toList());

      LinkedHashMap<String, MCAPSchema> subSchemaMap = new LinkedHashMap<>();

      for (int i = 1; i < schemasStrings.length; i++)
      {
         String schemaString = schemasStrings[i];

         int firstNewLineCharacter = schemaString.indexOf("\n");
         String firstLine = schemaString.substring(0, firstNewLineCharacter);
         String subName = firstLine.replace(SUB_SCHEMA_PREFIX, "").trim();
         List<MCAPSchemaField> subFields = schemaString.substring(firstNewLineCharacter + 1)
                                                       .lines()
                                                       .map(ROS2SchemaParser::parseMCAPSchemaField)
                                                       .collect(Collectors.toList());
         subSchemaMap.put(subName, new MCAPSchema(subName, -1, subFields, null));
      }

      // Update the fields to indicate whether they are complex types or not.
      for (MCAPSchemaField field : fields)
      {
         if (subSchemaMap.containsKey(field.getType()))
         {
            field.setComplexType(true);
         }

         for (MCAPSchema subSchema : subSchemaMap.values())
         {
            for (MCAPSchemaField subField : subSchema.getFields())
            {
               if (subSchemaMap.containsKey(subField.getType()))
               {
                  subField.setComplexType(true);
               }
            }
         }
      }

      return new MCAPSchema(name, id, fields, subSchemaMap);
   }

   public static MCAPSchemaField parseMCAPSchemaField(String line)
   {
      MCAPSchemaField field = new MCAPSchemaField();
      field.setType(line.substring(0, line.indexOf(' ')).trim());
      field.setName(line.substring(line.indexOf(' ') + 1).trim());

      int lBracketIndex = field.getType().indexOf('[');
      int rBracketIndex = field.getType().indexOf(']');

      if (lBracketIndex < rBracketIndex)
      {
         String maxLengthStr = field.getType().substring(lBracketIndex + 1, rBracketIndex);
         if (maxLengthStr.startsWith("<="))
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

package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.specs.records.Schema;

import java.util.ArrayList;
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
      List<MCAPSchemaField> fields = new ArrayList<>();

      fields.add(parseMCAPSchemaField(""));

      LinkedHashMap<String, MCAPSchema> subSchemaMap = new LinkedHashMap<>();

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
      System.out.println("SCHEMA::");
      System.out.println(line);

      MCAPSchemaField field = new MCAPSchemaField();
      field.setArray(false);
      field.setVector(false);
      field.setName("data");
      field.setType("float32");

      return field;
   }
}

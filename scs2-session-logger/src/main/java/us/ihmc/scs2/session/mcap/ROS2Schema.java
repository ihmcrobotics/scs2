package us.ihmc.scs2.session.mcap;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

import java.util.*;
import java.util.stream.Collectors;

import static us.ihmc.scs2.session.mcap.MCAPSchema.indent;
import static us.ihmc.scs2.session.mcap.MCAPSchema.indentString;

/**
 * Class used to represent a Java interpreter of a MCAP schema which encoding is "ros2msg".
 * This schema resembles much of ROS2 messages.
 */
public class ROS2Schema implements MCAPSchema
{
   public static final String SUB_SCHEMA_SEPARATOR_REGEX = "\n(=+)\n";
   public static final String SUB_SCHEMA_PREFIX = "MSG: fastdds/";
   private int id;
   private String name;
   private List<MCAPSchemaField> fields;
   private boolean isSchemaFlat;
   private Map<String, ROS2Schema> subSchemaMap;

   /**
    * Loads a schema from the given {@link MCAP.Schema}.
    *
    * @param mcapSchema the schema to load.
    * @return the loaded schema.
    */
   public static ROS2Schema loadSchema(MCAP.Schema mcapSchema)
   {
      ROS2Schema schema = loadSchema(mcapSchema.name(), mcapSchema.id(), mcapSchema.data());
      mcapSchema.unloadData();
      return schema;
   }

   /**
    * Loads a schema from the given data.
    *
    * @param name the name of the schema.
    * @param id   the ID of the schema.
    * @param data the data of the schema, expected to be a {@link String} using UTF-8 encoding.
    * @return the loaded schema.
    */
   public static ROS2Schema loadSchema(String name, int id, byte[] data)
   {
      ROS2Schema schema = new ROS2Schema();
      schema.name = name;
      schema.id = id;

      String schemasBundledString = new String(data);
      schemasBundledString = schemasBundledString.replaceAll("\r\n", "\n"); // To handle varying declaration of a new line.
      String[] schemasStrings = schemasBundledString.split(SUB_SCHEMA_SEPARATOR_REGEX);

      schema.fields = schemasStrings[0].lines().map(ROS2Schema::parseMCAPSchemaField).collect(Collectors.toList());

      schema.subSchemaMap = new LinkedHashMap<>();
      for (int i = 1; i < schemasStrings.length; i++)
      {
         String schemaString = schemasStrings[i];

         ROS2Schema subSchema = new ROS2Schema();

         int firstNewLineCharacter = schemaString.indexOf("\n");
         String firstLine = schemaString.substring(0, firstNewLineCharacter);
         subSchema.name = firstLine.replace(SUB_SCHEMA_PREFIX, "").trim();
         subSchema.fields = schemaString.substring(firstNewLineCharacter + 1).lines().map(ROS2Schema::parseMCAPSchemaField).collect(Collectors.toList());
         schema.subSchemaMap.put(subSchema.name, subSchema);
      }
      schema.isSchemaFlat = schema.subSchemaMap.isEmpty();

      // Update the fields to indicate whether they are complex types or not.
      for (MCAPSchemaField field : schema.fields)
      {
         if (schema.subSchemaMap.containsKey(field.getType()))
         {
            field.setComplexType(true);
         }

         for (ROS2Schema subSchema : schema.subSchemaMap.values())
         {
            for (MCAPSchemaField subField : subSchema.fields)
            {
               if (schema.subSchemaMap.containsKey(subField.getType()))
               {
                  subField.setComplexType(true);
               }
            }
         }
      }

      return schema;
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

   @Override
   public int getId()
   {
      return id;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public boolean isSchemaFlat()
   {
      return isSchemaFlat;
   }

   @Override
   public List<MCAPSchemaField> getFields()
   {
      return fields;
   }

   public Map<String, ROS2Schema> getSubSchemaMap()
   {
      return subSchemaMap;
   }

   /**
    * Returns a schema equivalent to this one but with all the complex types flattened, i.e. all the fields that are arrays or sub-schemas are expanded into
    * multiple fields.
    *
    * @return the flattened schema.
    */
   @Override
   public ROS2Schema flattenSchema()
   {
      ROS2Schema flatSchema = new ROS2Schema();
      flatSchema.id = id;
      flatSchema.name = name;
      flatSchema.isSchemaFlat = true;
      flatSchema.fields = new ArrayList<>();

      for (MCAPSchemaField field : fields)
      {
         flatSchema.fields.addAll(flattenField(field));
      }

      return flatSchema;
   }

   private List<MCAPSchemaField> flattenField(MCAPSchemaField field)
   {
      MCAPSchemaField flatField = field.clone();

      if (!field.isComplexType())
      {
         return Collections.singletonList(flatField);
      }

      List<MCAPSchemaField> flatFields = new ArrayList<>();
      flatFields.add(flatField);

      if (flatField.isArray())
      {
         for (int i = 0; i < flatField.getMaxLength(); i++)
         {
            MCAPSchemaField subField = new MCAPSchemaField();
            subField.setParent(flatField);
            subField.setType(flatField.getType());
            subField.setName(flatField.getName() + "[" + i + "]");
            subField.setArray(false);
            subField.setVector(false);
            subField.setMaxLength(-1);
            flatFields.add(subField);
         }
      }
      else
      {
         ROS2Schema subSchema = subSchemaMap.get(flatField.getType());
         if (subSchema != null)
         {
            for (MCAPSchemaField subField : subSchema.fields)
            {
               subField.setParent(flatField);
               subField.setName(flatField.getName() + "." + subField.getName());
               flatFields.add(subField);
            }
         }
      }
      return flatFields;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   public String toString(int indent)
   {
      String out = getClass().getSimpleName() + ":";
      out += "\n\t-name=" + name;
      if (fields != null)
         out += "\n\t-fields=\n" + EuclidCoreIOTools.getCollectionString("\n", fields, f -> f.toString(indent + 2));
      if (subSchemaMap != null)
         out += "\n\t-subSchemaMap=\n" + indentString(indent + 2) + EuclidCoreIOTools.getCollectionString("\n" + indentString(indent + 2),
                                                                                                          subSchemaMap.entrySet(),
                                                                                                          e -> e.getKey() + "->\n" + e.getValue()
                                                                                                                                      .toString(indent + 3)
                                                                                                                                      .replace("^(\t*)", ""));
      return indent(out, indent);
   }

   public static String mcapROS2MessageToString(MCAP.Message message, ROS2Schema schema)
   {
      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(message.messageBuffer(), message.offsetData(), message.lengthData());

      String output = mcapROS2MessageToString(cdr, schema, 0);

      cdr.finalize(true);
      return output;
   }

   private static String mcapROS2MessageToString(CDRDeserializer cdr, ROS2Schema schema, int indent)
   {
      StringBuilder out = new StringBuilder(schema.getName() + ":");
      for (MCAPSchemaField field : schema.fields)
      {
         String fieldToString = mcapROS2MessageFieldToString(cdr, field, schema, indent + 1);
         if (fieldToString != null)
            out.append(fieldToString);
      }
      return out.toString();
   }

   private static String mcapROS2MessageFieldToString(CDRDeserializer cdr, MCAPSchemaField field, ROS2Schema schema, int indent)
   {
      if (schema == null && field.isComplexType())
      { // Dealing with a flat schema, skip this field.
         return null;
      }

      StringBuilder out = new StringBuilder("\n" + indentString(indent) + field.getName() + ": ");

      if (field.isArray())
      {
         out.append("[");
         for (int i = 0; i < field.getMaxLength(); i++)
         {
            out.append("\n").append(indentString(indent + 1)).append(i).append(": ");
            out.append(mcapROS2MessageFieldToString(cdr, field, schema, indent + 2));
         }
         out.append("\n").append(indentString(indent)).append("]");
      }
      else
      {
         String fieldValue = null;
         try
         {
            fieldValue = cdr.readTypeAsString(CDRDeserializer.Type.parseType(field.getType()), field.getMaxLength());
         }
         catch (IllegalArgumentException e)
         {
            // Ignore
         }

         if (fieldValue == null)
         {
            ROS2Schema subSchema = schema.getSubSchemaMap() == null ? null : schema.getSubSchemaMap().get(field.getType());
            if (subSchema != null)
            {
               fieldValue = "\n" + indentString(indent + 1) + mcapROS2MessageToString(cdr, subSchema, indent + 1);
            }
            else if (!schema.isSchemaFlat())
            {
               fieldValue = "Unknown type: " + field.getType();
            }
         }
         out.append(fieldValue);
      }

      return out.toString();
   }
}

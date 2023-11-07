package us.ihmc.scs2.session.mcap;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

import java.util.*;
import java.util.stream.Collectors;

public class ROS2MessageSchema
{
   private int id;
   private String name;
   private List<ROS2Field> fields;
   private boolean isSchemaFlat;
   private Map<String, ROS2MessageSchema> subSchemaMap;

   public static ROS2MessageSchema loadSchema(Mcap.Schema mcapSchema)
   {
      ROS2MessageSchema schema = loadSchema(mcapSchema.name(), mcapSchema.id(), mcapSchema.data());
      mcapSchema.unloadData();
      return schema;
   }

   public static ROS2MessageSchema loadSchema(String name, int id, byte[] data)
   {
      ROS2MessageSchema schema = new ROS2MessageSchema();
      schema.name = name;
      schema.id = id;

      String schemasBundledString = new String(data);
      schemasBundledString = schemasBundledString.replaceAll("\r\n", "\n"); // To handle varying declaration of a new line.
      String[] schemasStrings = schemasBundledString.split("\n(=+)\n");

      schema.fields = schemasStrings[0].lines().map(ROS2Field::fromLine).collect(Collectors.toList());

      schema.subSchemaMap = new LinkedHashMap<>();
      for (int i = 1; i < schemasStrings.length; i++)
      {
         String schemaString = schemasStrings[i];

         ROS2MessageSchema subSchema = new ROS2MessageSchema();

         int firstNewLineCharacter = schemaString.indexOf("\n");
         String firstLine = schemaString.substring(0, firstNewLineCharacter);
         subSchema.name = firstLine.replace("MSG: fastdds/", "").trim();
         subSchema.fields = schemaString.substring(firstNewLineCharacter + 1).lines().map(ROS2Field::fromLine).collect(Collectors.toList());
         schema.subSchemaMap.put(subSchema.name, subSchema);
      }
      schema.isSchemaFlat = schema.subSchemaMap.isEmpty();

      // Update the fields to indicate whether they are complex types or not.
      for (ROS2Field field : schema.fields)
      {
         if (schema.subSchemaMap.containsKey(field.getType()))
         {
            field.isComplexType = true;
         }

         for (ROS2MessageSchema subSchema : schema.subSchemaMap.values())
         {
            for (ROS2Field subField : subSchema.fields)
            {
               if (schema.subSchemaMap.containsKey(subField.getType()))
               {
                  subField.isComplexType = true;
               }
            }
         }
      }

      return schema;
   }

   public int getId()
   {
      return id;
   }

   public String getName()
   {
      return name;
   }

   public boolean isSchemaFlat()
   {
      return isSchemaFlat;
   }

   public List<ROS2Field> getFields()
   {
      return fields;
   }

   public Map<String, ROS2MessageSchema> getSubSchemaMap()
   {
      return subSchemaMap;
   }

   /**
    * Returns a schema equivalent to this one but with all the complex types flattened, i.e. all the fields that are arrays or sub-schemas are expanded into
    * multiple fields.
    *
    * @return the flattened schema.
    */
   public ROS2MessageSchema flattenSchema()
   {
      ROS2MessageSchema flatSchema = new ROS2MessageSchema();
      flatSchema.id = id;
      flatSchema.name = name;
      flatSchema.isSchemaFlat = true;
      flatSchema.fields = new ArrayList<>();

      for (ROS2Field field : fields)
      {
         flatSchema.fields.addAll(flattenField(field));
      }

      return flatSchema;
   }

   private List<ROS2Field> flattenField(ROS2Field field)
   {
      ROS2Field flatField = field.clone();

      if (!field.isComplexType)
      {
         return Collections.singletonList(flatField);
      }

      List<ROS2Field> flatFields = new ArrayList<>();
      flatFields.add(flatField);

      if (flatField.isArray())
      {
         for (int i = 0; i < flatField.maxLength; i++)
         {
            ROS2Field subField = new ROS2Field();
            subField.parent = flatField;
            subField.type = flatField.type;
            subField.name = flatField.name + "[" + i + "]";
            subField.isArray = false;
            subField.maxLength = -1;
            flatFields.add(subField);
         }
      }
      else
      {
         ROS2MessageSchema subSchema = subSchemaMap.get(flatField.getType());
         if (subSchema != null)
         {
            for (ROS2Field subField : subSchema.fields)
            {
               subField.parent = flatField;
               subField.name = flatField.name + "." + subField.name;
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

   public static class ROS2Field
   {
      /**
       * The parent is used when flattening the schema.
       */
      private ROS2Field parent;
      private String type;
      private String name;
      private boolean isArray;
      private int maxLength;
      /**
       * This is {@code  true} whenever this field is for an array or a sub-schema.
       */
      private boolean isComplexType;

      public static ROS2Field fromLine(String line)
      {
         ROS2Field field = new ROS2Field();
         field.type = line.substring(0, line.indexOf(' ')).trim();
         field.name = line.substring(line.indexOf(' ') + 1).trim();

         int lBracketIndex = field.type.indexOf('[');
         int rBracketIndex = field.type.indexOf(']');

         if (lBracketIndex < rBracketIndex)
         {
            field.isArray = true;
            field.isComplexType = true;
            String maxLengthStr = field.type.substring(lBracketIndex + 1, rBracketIndex);
            try
            {
               field.maxLength = Integer.parseInt(maxLengthStr);
            }
            catch (NumberFormatException e)
            {
               // The length is probably defined as a maximum length "array[<=54]"
               maxLengthStr = maxLengthStr.replace("<=", "");
               field.maxLength = Integer.parseInt(maxLengthStr);
            }
            field.type = field.type.substring(0, lBracketIndex);
         }
         else
         {
            field.isArray = false;
            field.maxLength = -1;
         }
         return field;
      }

      public ROS2Field clone()
      {
         ROS2Field clone = new ROS2Field();
         clone.parent = parent;
         clone.type = type;
         clone.name = name;
         clone.isArray = isArray;
         clone.maxLength = maxLength;
         clone.isComplexType = isComplexType;
         return clone;
      }

      public ROS2Field getParent()
      {
         return parent;
      }

      public String getType()
      {
         return type;
      }

      public void setType(String type)
      {
         this.type = type;
      }

      public String getName()
      {
         return name;
      }

      public void setName(String name)
      {
         this.name = name;
      }

      public boolean isArray()
      {
         return isArray;
      }

      public void setArray(boolean isArray)
      {
         this.isArray = isArray;
      }

      public int getMaxLength()
      {
         return maxLength;
      }

      public void setMaxLength(int maxLength)
      {
         this.maxLength = maxLength;
      }

      /**
       * This is {@code  true} whenever this field is for an array or a sub-schema.
       * <p>
       * If this field is belongs to a flat schema, the subsequent fields will be the elements of the array or the fields of the sub-schema, such that this
       * field can be skipped when deseriliazing a message.
       * </p>
       *
       * @return {@code true} if this field is for an array or a sub-schema, {@code false} otherwise.
       */
      public boolean isComplexType()
      {
         return isComplexType;
      }

      @Override
      public String toString()
      {
         return toString(0);
      }

      public String toString(int indent)
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-type=" + type;
         out += "\n\t-name=" + name;
         out += "\n\t-isArray=" + isArray;
         if (isArray)
            out += "\n\t-maxLength=" + maxLength;
         return indent(out, indent);
      }
   }

   private static String indent(String stringToIndent, int indent)
   {
      if (indent <= 0)
         return stringToIndent;
      String indentStr = indentString(indent);
      return indentStr + stringToIndent.replace("\n", "\n" + indentStr);
   }

   private static String indentString(int indent)
   {
      return "\t".repeat(indent);
   }

   public static String mcapROS2MessageToString(Mcap.Message message, ROS2MessageSchema schema)
   {
      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(message.messageBuffer(), message.offsetData(), message.lengthData());

      String output = mcapROS2MessageToString(cdr, schema, 0);

      cdr.finalize(true);
      return output;
   }

   private static String mcapROS2MessageToString(CDRDeserializer cdr, ROS2MessageSchema schema, int indent)
   {
      StringBuilder out = new StringBuilder(schema.getName() + ":");
      for (ROS2Field field : schema.fields)
      {
         String fieldToString = mcapROS2MessageFieldToString(cdr, field, schema, indent + 1);
         if (fieldToString != null)
            out.append(fieldToString);
      }
      return out.toString();
   }

   private static String mcapROS2MessageFieldToString(CDRDeserializer cdr, ROS2Field field, ROS2MessageSchema schema, int indent)
   {
      if (schema == null && field.isComplexType())
      { // Dealing with a flat schema, skip this field.
         return null;
      }

      StringBuilder out = new StringBuilder("\n" + indentString(indent) + field.getName() + ": ");

      if (field.isArray())
      {
         out.append("[");
         for (int i = 0; i < field.maxLength; i++)
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
            ROS2MessageSchema subSchema = schema.getSubSchemaMap() == null ? null : schema.getSubSchemaMap().get(field.getType());
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

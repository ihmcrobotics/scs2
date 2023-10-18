package us.ihmc.scs2.session.mcap;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

public class ROS2MessageSchema
{
   private String name;
   private List<ROS2Field> fields;
   private Map<String, ROS2MessageSchema> subSchemaMap;

   public static ROS2MessageSchema loadSchema(String name, byte[] data)
   {
      ROS2MessageSchema schema = new ROS2MessageSchema();
      schema.name = name;

      String schemasBundledString = new String(data);
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
         subSchema.fields = schemaString.substring(firstNewLineCharacter + 1, schemaString.length()).lines().map(ROS2Field::fromLine)
                                        .collect(Collectors.toList());
         schema.subSchemaMap.put(subSchema.name, subSchema);
      }

      return schema;
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public List<ROS2Field> getFields()
   {
      return fields;
   }

   public void setFields(List<ROS2Field> fields)
   {
      this.fields = fields;
   }

   public Map<String, ROS2MessageSchema> getSubSchemaMap()
   {
      return subSchemaMap;
   }

   public void setSubSchemaMap(Map<String, ROS2MessageSchema> subSchemaMap)
   {
      this.subSchemaMap = subSchemaMap;
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
         out += "\n\t-subSchemaMap=\n" + indentString(indent + 2)
               + EuclidCoreIOTools.getCollectionString("\n" + indentString(indent + 2),
                                                       subSchemaMap.entrySet(),
                                                       e -> e.getKey() + "->\n" + e.getValue().toString(indent + 3).replace("^(\t*)", ""));
      return indent(out, indent);
   }

   public static class ROS2Field
   {
      private String type;
      private String name;
      private boolean isArray;
      private int maxLength;

      public static ROS2Field fromLine(String line)
      {
         ROS2Field field = new ROS2Field();
         field.type = line.substring(0, line.indexOf(' ')).trim();
         field.name = line.substring(line.indexOf(' ') + 1, line.length()).trim();

         int lBracketIndex = field.type.indexOf('[');
         int rBracketIndex = field.type.indexOf(']');

         if (lBracketIndex < rBracketIndex)
         {
            field.isArray = true;
            field.maxLength = Integer.parseInt(field.type.substring(lBracketIndex + 1, rBracketIndex));
            field.type = field.type.substring(0, lBracketIndex);
         }
         else
         {
            field.isArray = false;
            field.maxLength = -1;
         }
         return field;
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
}

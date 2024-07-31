package us.ihmc.scs2.session.mcap;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Schema;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Interface used to represent a Java interpreter of a MCAP schema.
 */
public class MCAPSchema
{
   private final int id;
   private final String name;
   private final boolean isEnum;
   private final String[] enumConstants;
   private final List<MCAPSchemaField> staticFields = new ArrayList<>();
   private final List<MCAPSchemaField> fields;
   private final Map<String, MCAPSchema> subSchemaMap;

   protected MCAPSchema(String name, int id)
   {
      this(name, id, new ArrayList<>(), null);
   }

   protected MCAPSchema(String name, int id, List<MCAPSchemaField> fields)
   {
      this(name, id, fields, null);
   }

   /**
    * Creates a schema that represents a struct.
    *
    * @param name         the name of the schema.
    * @param id           the ID of the schema.
    * @param fields       the fields of the schema.
    * @param subSchemaMap the sub-schemas of the schema.
    */
   protected MCAPSchema(String name, int id, List<MCAPSchemaField> fields, Map<String, MCAPSchema> subSchemaMap)
   {
      this.name = name;
      this.id = id;
      this.subSchemaMap = subSchemaMap;
      this.fields = fields;
      this.isEnum = false;
      this.enumConstants = null;
   }

   /**
    * Creates a schema that represents an enum.
    *
    * @param name          the name of the schema.
    * @param id            the ID of the schema.
    * @param enumConstants the enum constants of the schema.
    */
   protected MCAPSchema(String name, int id, String[] enumConstants)
   {
      this.name = name;
      this.id = id;
      this.subSchemaMap = null;
      this.fields = null;
      this.isEnum = true;
      this.enumConstants = enumConstants;
   }

   /**
    * The ID of the schema as defined in the MCAP schema file, {@link Schema#id()}.
    *
    * @return the ID of the schema.
    */
   public int getId()
   {
      return id;
   }

   /**
    * The name of the schema as defined in the MCAP schema file, {@link Schema#name()}.
    *
    * @return the name of the schema.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Whether this schema is an enum.
    *
    * @return {@code true} if this schema is an enum, {@code false} otherwise.
    */
   public boolean isEnum()
   {
      return isEnum;
   }

   /**
    * The enum constants of the schema as defined in the MCAP schema file.
    *
    * @return the enum constants of the schema.
    */
   public String[] getEnumConstants()
   {
      return enumConstants;
   }

   /**
    * The fields declared in the schema.
    * <p>
    * The fields can be of primitive type, array, vector, or sub-schema.
    * </p>
    *
    * @return the fields of the schema.
    */
   public List<MCAPSchemaField> getFields()
   {
      return fields;
   }

   public Map<String, MCAPSchema> getSubSchemaMap()
   {
      return subSchemaMap;
   }

   /**
    * Indicates whether this schema is already flat.
    *
    * @return {@code true} if this schema is already flat, {@code false} otherwise.
    */
   public boolean isSchemaFlat()
   {
      return subSchemaMap == null || subSchemaMap.isEmpty();
   }

   public List<MCAPSchemaField> getStaticFields()
   {
      return staticFields;
   }

   /**
    * Returns a schema equivalent to this one but with all the complex types flattened, i.e. all the fields that are arrays or sub-schemas are expanded into
    * multiple fields.
    *
    * @return the flattened schema.
    */
   public MCAPSchema flattenSchema()
   {
      List<MCAPSchemaField> flattenedFields = new ArrayList<>();

      for (MCAPSchemaField field : fields)
      {
         flattenedFields.addAll(flattenField(field));
      }

      MCAPSchema mcapSchema = new MCAPSchema(name, id, flattenedFields, null);
      mcapSchema.getStaticFields().addAll(staticFields);
      return mcapSchema;
   }

   List<MCAPSchemaField> flattenField(MCAPSchemaField field)
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
         MCAPSchema subSchema = subSchemaMap.get(flatField.getType());
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
      if (enumConstants != null)
         out += "\n\t-enumConstants=" + Arrays.toString(enumConstants);
      if (!staticFields.isEmpty())
         out += "\n\t-staticFields=\n" + EuclidCoreIOTools.getCollectionString("\n", staticFields, f -> f.toString(indent + 2));
      if (subSchemaMap != null)
         out += "\n\t-subSchemaMap=\n" + indentString(indent + 2) + EuclidCoreIOTools.getCollectionString("\n" + indentString(indent + 2),
                                                                                                          subSchemaMap.entrySet(),
                                                                                                          e -> e.getKey() + "->\n" + e.getValue()
                                                                                                                                      .toString(indent + 3)
                                                                                                                                      .replace("^(\t*)", ""));
      return indent(out, indent);
   }

   public static String mcapMCAPMessageToString(Message message, MCAPSchema schema)
   {
      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(message.messageBuffer(), 0, message.dataLength());

      String output = mcapMCAPMessageToString(cdr, schema, 0);

      cdr.finalize(true);
      return output;
   }

   private static String mcapMCAPMessageToString(CDRDeserializer cdr, MCAPSchema schema, int indent)
   {
      StringBuilder out = new StringBuilder(schema.getName() + ":");
      for (MCAPSchemaField field : schema.getFields())
      {
         String fieldToString = mcapMCAPMessageFieldToString(cdr, field, schema, indent + 1);
         if (fieldToString != null)
            out.append(fieldToString);
      }
      return out.toString();
   }

   private static String mcapMCAPMessageFieldToString(CDRDeserializer cdr, MCAPSchemaField field, MCAPSchema schema, int indent)
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
            out.append(mcapMCAPMessageFieldToString(cdr, field, schema, indent + 2));
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
            MCAPSchema subSchema = schema.getSubSchemaMap() == null ? null : schema.getSubSchemaMap().get(field.getType());
            if (subSchema != null)
            {
               fieldValue = "\n" + indentString(indent + 1) + mcapMCAPMessageToString(cdr, subSchema, indent + 1);
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

   /**
    * Interface used to represent a field of a MCAP schema.
    * <p>
    * A field can be a primitive type, an array, a vector, or a sub-schema.
    * </p>
    */
   public static final class MCAPSchemaField
   {
      private MCAPSchemaField parent;
      private String name;
      private String type;
      private boolean isArray;
      private boolean isVector;
      private int maxLength;
      private boolean isComplexType;
      // Used for static fields
      private String defaultValue;

      public MCAPSchemaField()
      {
      }

      public MCAPSchemaField(String name, String type, boolean isArray, boolean isVector, int maxLength, boolean isComplexType)
      {
         this.name = name;
         this.type = type;
         this.isArray = isArray;
         this.isVector = isVector;
         this.maxLength = maxLength;
         this.isComplexType = isComplexType;
      }

      public MCAPSchemaField(MCAPSchemaField other)
      {
         this.name = other.name;
         this.type = other.type;
         this.isArray = other.isArray;
         this.isVector = other.isVector;
         this.maxLength = other.maxLength;
         this.isComplexType = other.isComplexType;
      }

      @Override
      public MCAPSchemaField clone()
      {
         return new MCAPSchemaField(this);
      }

      public void setName(String name)
      {
         this.name = name;
      }

      public void setType(String type)
      {
         this.type = type;
      }

      public void setParent(MCAPSchemaField parent)
      {
         this.parent = parent;
      }

      public void setArray(boolean array)
      {
         isArray = array;
      }

      public void setVector(boolean vector)
      {
         isVector = vector;
      }

      public void setMaxLength(int maxLength)
      {
         this.maxLength = maxLength;
      }

      public void setComplexType(boolean complexType)
      {
         isComplexType = complexType;
      }

      /**
       * The default value of the field as defined in the MCAP schema.
       * <p>
       * This is only used for static fields.
       * </p>
       *
       * @param defaultValue the default value of the field.
       */
      public void setDefaultValue(String defaultValue)
      {
         this.defaultValue = defaultValue;
      }

      /**
       * The name of the field as defined in the MCAP schema.
       *
       * @return the name of the field.
       */
      public String getName()
      {
         return name;
      }

      /**
       * The type of the field as defined in the MCAP schema.
       *
       * @return the type of the field.
       */
      public String getType()
      {
         return type;
      }

      /**
       * The parent schema for this field.
       *
       * @return the parent schema of the field.
       */
      public MCAPSchemaField getParent()
      {
         return parent;
      }

      /**
       * Whether this field is an array.
       *
       * @return {@code true} if this field is an array, {@code false} otherwise.
       */
      public boolean isArray()
      {
         return isArray;
      }

      public boolean isVector()
      {
         return isVector;
      }

      /**
       * Whether this field is a vector.
       *
       * @return {@code true} if this field is a vector, {@code false} otherwise.
       */
      public int getMaxLength()
      {
         return maxLength;
      }

      public String getDefaultValue()
      {
         return defaultValue;
      }

      /**
       * Whether this field is a complex type such as an array, a vector, or a sub-schema.
       *
       * @return {@code true} if this field is complex, {@code false} otherwise.
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
         out += "\n\t-isVector=" + isVector;
         out += "\n\t-isComplexType=" + isComplexType;
         if (isArray || isVector)
            out += "\n\t-maxLength=" + maxLength;
         out += "\n\t-parent=" + (parent == null ? "null" : parent.name);
         out += "\n";
         return indent(out, indent);
      }
   }

   public static String indent(String stringToIndent, int indent)
   {
      if (indent <= 0)
         return stringToIndent;
      String indentStr = indentString(indent);
      return indentStr + stringToIndent.replace("\n", "\n" + indentStr);
   }

   public static String indentString(int indent)
   {
      return "\t".repeat(indent);
   }
}

package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.MCAP.Schema;

import java.util.List;

/**
 * Interface used to represent a Java interpreter of a MCAP schema.
 * <p>
 * Two implementations are provided:
 * <ul>
 *    <li>{@link ROS2Schema} is for the ROS2 message schema, encoding is "ros2msg".
 *    <li>{@link OMGIDLSchema} is for the OMG IDL schema, encoding is "omgidl".
 * </ul>
 */
public interface MCAPSchema
{
   /**
    * The ID of the schema as defined in the MCAP schema file, {@link Schema#id()}.
    *
    * @return the ID of the schema.
    */
   int getId();

   /**
    * The name of the schema as defined in the MCAP schema file, {@link Schema#name()}.
    *
    * @return the name of the schema.
    */
   String getName();

   /**
    * The fields declared in the schema.
    * <p>
    * The fields can be of primitive type, array, vector, or sub-schema.
    * </p>
    *
    * @return the fields of the schema.
    */
   List<? extends MCAPSchemaField> getFields();

   /**
    * Flattens this schema into a new schema where all the fields are primitive types.
    *
    * @param <T> the type of the flattened schema.
    * @return the flattened schema.
    */
   MCAPSchema flattenSchema();

   /**
    * Indicates whether this schema is already flat.
    *
    * @return {@code true} if this schema is already flat, {@code false} otherwise.
    */
   boolean isSchemaFlat();

   /**
    * Interface used to represent a field of a MCAP schema.
    * <p>
    * A field can be a primitive type, an array, a vector, or a sub-schema.
    * </p>
    */
   final class MCAPSchemaField
   {
      private MCAPSchemaField parent;
      private String name;
      private String type;
      private boolean isArray;
      private boolean isVector;
      private int maxLength;
      private boolean isComplexType;

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

      public void setParent(MCAPSchemaField parent)
      {
         this.parent = parent;
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

      /**
       * Whether this field is a complex type such as an array, a vector, or a sub-schema.
       *
       * @return {@code true} if this field is complex, {@code false} otherwise.
       */
      public boolean isComplexType()
      {
         return isComplexType;
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

   static String indent(String stringToIndent, int indent)
   {
      if (indent <= 0)
         return stringToIndent;
      String indentStr = indentString(indent);
      return indentStr + stringToIndent.replace("\n", "\n" + indentStr);
   }

   static String indentString(int indent)
   {
      return "\t".repeat(indent);
   }
}

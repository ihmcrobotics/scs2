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
   interface MCAPSchemaField
   {
      /**
       * The name of the field as defined in the MCAP schema.
       *
       * @return the name of the field.
       */
      String getName();

      /**
       * The type of the field as defined in the MCAP schema.
       *
       * @return the type of the field.
       */
      String getType();

      /**
       * The parent schema for this field.
       *
       * @return the parent schema of the field.
       */
      <T extends MCAPSchemaField> T getParent();

      /**
       * Whether this field is an array.
       *
       * @return {@code true} if this field is an array, {@code false} otherwise.
       */
      boolean isArray();

      /**
       * Whether this field is a vector.
       *
       * @return {@code true} if this field is a vector, {@code false} otherwise.
       */
      boolean isVector();

      /**
       * Whether this field is a complext type such as an array, a vector, or a sub-schema.
       *
       * @return {@code true} if this field is complex, {@code false} otherwise.
       */
      boolean isComplexType();
   }
}

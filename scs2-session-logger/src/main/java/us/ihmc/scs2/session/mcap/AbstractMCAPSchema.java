package us.ihmc.scs2.session.mcap;

import java.util.List;
import java.util.Map;

public abstract class AbstractMCAPSchema
{

//   public static abstract class AbstractMCAPField
//   {
//      /**
//       * The parent is used when flattening the schema.
//       */
//      private AbstractMCAPField parent;
//      private String type;
//      private String name;
//      /**
//       * {@code true} when the element is a sequence or an array
//       */
//      private boolean isArray;
//      /**
//       * For non array fields, maxLength should be <= 0
//       */
//      private int maxLength;
//      /**
//       * This is {@code  true} whenever this field is for an array, sequence, or struct.
//       */
//      private boolean isComplexType;
//
//   }
//
//   private int id;
//   private String name;
//   private List<T extends AbstractMCAPField> fields;
//   private boolean isSchemaFlat;
//   private Map<String, AbstractMCAPSchema> subSchemaMap;
//
//
//   public int getId()
//   {
//      return id;
//   }
//
//   public void setId(int id)
//   {
//      this.id = id;
//   }
//
//   public String getName()
//   {
//      return name;
//   }
//
//   public void setName(String name)
//   {
//      this.name = name;
//   }
//
//   public List<AbstractMCAPField> getFields()
//   {
//      return fields;
//   }
//
//   public void setFields(List<? extends AbstractMCAPField> fields)
//   {
//      this.fields = (List<AbstractMCAPField>) fields;
//   }
//
//   public boolean isSchemaFlat()
//   {
//      return isSchemaFlat;
//   }
//
//   public void setSchemaFlat(boolean schemaFlat)
//   {
//      isSchemaFlat = schemaFlat;
//   }
//
//   public Map<String, AbstractMCAPField> getSubSchemaMap()
//   {
//      return subSchemaMap;
//   }
//
//   public void setSubSchemaMap(Map<String, AbstractMCAPField> subSchemaMap)
//   {
//      this.subSchemaMap = subSchemaMap;
//   }

}

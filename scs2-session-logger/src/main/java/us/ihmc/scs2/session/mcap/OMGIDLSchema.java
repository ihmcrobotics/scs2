package us.ihmc.scs2.session.mcap;

import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLLexer;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLListener;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLParser;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.*;

public class OMGIDLSchema implements MCAPSchema
{
   private int id;
   private String name;
   private List<OMGIDLSchemaField> fields;
   private boolean isSchemaFlat;
   private Map<String, OMGIDLSchema> subSchemaMap;

   public static OMGIDLSchema loadSchema(Mcap.Schema mcapSchema) throws IOException
   {
      return loadSchema(mcapSchema.name(), mcapSchema.id(), mcapSchema.data());
   }

   public static OMGIDLSchema loadSchema(String name, int id, byte[] data) throws IOException
   {
      OMGIDLSchema schema = new OMGIDLSchema();
      schema.name = name;
      schema.id = id;
      schema.subSchemaMap = new LinkedHashMap<>();
      schema.fields = new ArrayList<>();

      String schemasBundledString = new String(data);

      CharStream bytesAsChar = null;

      try
      {
         bytesAsChar = CharStreams.fromStream(new ByteArrayInputStream(data));
      }
      catch (IOException e)
      {
         throw e;
      }

      IDLLexer lexer = new IDLLexer(bytesAsChar);
      CommonTokenStream tokens = new CommonTokenStream(lexer);
      IDLParser parser = new IDLParser(tokens);
      parser.setBuildParseTree(true);
      ParseTree tree = parser.specification();

      IDLListener schemaCreatorListener = new SchemaCreatorListener(schema);
      ParseTreeWalker.DEFAULT.walk(schemaCreatorListener, tree);

      schema.isSchemaFlat = schema.subSchemaMap.isEmpty();
      return schema;
   }

   protected OMGIDLSchema(String name, int id)
   {
      this.name = name;
      this.id = id;
      this.subSchemaMap = new HashMap<>();
      this.fields = new ArrayList<>();
      this.isSchemaFlat = true;
   }

   private OMGIDLSchema()
   {
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

   public List<OMGIDLSchemaField> getFields()
   {
      return fields;
   }

   public Map<String, OMGIDLSchema> getSubSchemaMap()
   {
      return subSchemaMap;
   }

   /**
    * Returns a schema equivalent to this one but with all the complex types flattened, i.e. all the fields that are arrays or sub-schemas are expanded into
    * multiple fields.
    *
    * @return the flattened schema.
    */
   public OMGIDLSchema flattenSchema()
   {
      return flattenSchema(this.subSchemaMap);
   }

   public OMGIDLSchema flattenSchema(Map<String, OMGIDLSchema> subSchemaMap)
   {
      OMGIDLSchema flatSchema = new OMGIDLSchema();
      flatSchema.id = this.id;
      flatSchema.name = this.name;
      flatSchema.isSchemaFlat = true;
      flatSchema.fields = new ArrayList<>();

      for (OMGIDLSchemaField field : this.getFields())
      {
         flatSchema.fields.addAll(this.flattenField(field, subSchemaMap));
      }

      return flatSchema;
   }

   public List<OMGIDLSchemaField> flattenField(OMGIDLSchemaField field)
   {
      return flattenField(field, this.subSchemaMap);
   }

   public List<OMGIDLSchemaField> flattenField(OMGIDLSchemaField field, Map<String, OMGIDLSchema> subSchemaMap)
   {
      //TODO: (AM) Check correctness and refactor, super ugly code follows
      //       1. Note that this will not flatten vectors or arrays of types.
      //       As such, nested arrays and sequences are not supported for now
      //       2. This method doesn't modify the original fields or schemas, and as such
      //       has a bunch of copy operations, this is not ideal, but keeps flattenSchema stateless

      OMGIDLSchemaField flatField = field.clone();

      //TODO: (AM) check that isComplexType is set properly for every non-flat field
      if (!field.isComplexType)
      {
         return Collections.singletonList(flatField);
      }
      List<OMGIDLSchemaField> flatFields = new ArrayList<>();
      //      flatFields.add(flatField);

      List<OMGIDLSchemaField> flatElementFields = new ArrayList<>();
      if (flatField.isArray() || flatField.isVector())
      {
         flatFields.add(flatField);
         String baseType = flatField.getType();
         if (flatField.isVector())
            baseType = baseType.split("[<,>]")[1];

         // This assumes that we have access to the highest level subSchemaMap,
         // if a type is not found in this map, it will be assumed to be a primitive type
         if (subSchemaMap.containsKey(baseType))
         {
            // Array/vector of structs, recursively flatten
            OMGIDLSchema subSchema = subSchemaMap.get(baseType);
            for (OMGIDLSchemaField subField : subSchema.getFields())
            {
               OMGIDLSchemaField flatSubField = subField.clone();
               flatSubField.parent = flatField;
               flatSubField.name = flatField.name + "-" + subField.name;
               flatElementFields.addAll(this.flattenField(flatSubField, subSchemaMap).stream().filter(f -> !f.isComplexType()).toList());
            }
         }
         else
         {
            //Array/vector of primitive types;
            OMGIDLSchemaField elementField = new OMGIDLSchemaField();
            elementField.parent = flatField;
            elementField.type = baseType;
            elementField.name = flatField.name;
            elementField.isArray = false;
            elementField.isVector = false;
            elementField.isComplexType = false;
            elementField.maxLength = -1;
            flatElementFields.add(elementField);
         }
         // Add all the elements to flatFields
//         for (int i = 0; i < flatField.getMaxLength(); i++)
//         {
//            for (OMGIDLSchemaField flatElementField : flatElementFields)
//            {
//               OMGIDLSchemaField subField = flatElementField.clone();
//               subField.name = subField.name + "[" + i + "]";
//               flatFields.add(subField);
//            }
//         }
         flatFields.addAll(flatElementFields);
      }
      else
      {
         OMGIDLSchema subSchema = subSchemaMap.get(flatField.getName());
         if (subSchema != null)
         {
            // we are entering a struct definition
            for (OMGIDLSchemaField subField : subSchema.getFields())
            {
               OMGIDLSchemaField childSubField = subField.clone();
               childSubField.parent = flatField;
               if (subSchemaMap.containsKey(childSubField.getType()) || childSubField.isArray() || childSubField.isVector())
               {
                  // if this field is a struct of another type, then flatten recursively
                  childSubField.parent = flatField;
                  flatFields.addAll(this.flattenField(childSubField));
               }
               else
               {
                  // if this field is a simple element of a primitive type, then add it to flatFields
                  OMGIDLSchemaField flatSubField = subField.clone();
                  flatSubField.parent = flatField;
                  flatFields.add(flatSubField);
               }
            }
         }
         else
         {
            // entering a struct type instantiation
            flatFields.add(flatField);
            subSchema = subSchemaMap.get(flatField.getType());
            if (subSchema != null)
            {
               for (OMGIDLSchemaField subField : subSchema.getFields())
               {
                  OMGIDLSchemaField childSubField = subField.clone();
                  childSubField.parent = flatField;
                  childSubField.name = flatField.name + "-" + subField.name;
                  if (subSchemaMap.containsKey(childSubField.getType()))
                  {
                     // if this field is a struct of another type, then flatten recursively
                     flatFields.addAll(this.flattenField(childSubField));
                  }
                  else
                  {
                     // if this field is a base type, then add it to flatfields
                     OMGIDLSchemaField flatSubField = subField.clone();
                     flatSubField.parent = flatField;
                     flatSubField.name = flatField.getName() + "-" + subField.getName();
                     flatFields.add(flatSubField);
                  }
               }
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
         out += "\n\t-fields=\n" + EuclidCoreIOTools.getCollectionString("\n", fields, f -> f.toString(indent + 1));
      if (subSchemaMap != null)
         out += "\n\t-subSchemaMap=\n" + indentString(indent + 1) + EuclidCoreIOTools.getCollectionString("\n" + indentString(indent + 1),
                                                                                                          subSchemaMap.entrySet(),
                                                                                                          e -> e.getKey() + "->\n" + e.getValue()
                                                                                                                                      .toString(indent + 2)
                                                                                                                                      .replace("^(\t*)", ""));
      return indent(out, indent);
   }

   public static class OMGIDLSchemaField implements MCAPSchemaField
   {
      /**
       * The parent is used when flattening the schema.
       */
      private OMGIDLSchemaField parent;
      private String type;
      private String name;
      private boolean isArray;
      /**
       * For non array fields, maxLength should be <= 0
       */
      private int maxLength;
      /**
       * This is {@code  true} whenever this field is for an array, sequence, or struct.
       */
      private boolean isComplexType;
      private boolean isVector;

      private OMGIDLSchemaField()
      {

      }

      protected OMGIDLSchemaField(String type, String name, int maxLength, boolean isComplexType)
      {
         this.type = type;
         this.name = name;
         this.isArray = (maxLength > -1 && !type.contains("sequence"));
         this.maxLength = maxLength;
         //TODO: (AM) don't assume that all scoped types are complex
         //this.isComplexType = (isArray || type.contains("::") || type.equals("sequence") || type.equals("struct"));
         this.isComplexType = isComplexType;
         this.isVector = type.contains("sequence");
         this.parent = null;
      }

      public OMGIDLSchemaField clone()
      {
         OMGIDLSchemaField clone = new OMGIDLSchemaField();
         clone.parent = parent;
         clone.type = type;
         clone.name = name;
         clone.isArray = isArray;
         clone.maxLength = maxLength;
         clone.isComplexType = isComplexType;
         clone.isVector = isVector;
         return clone;
      }

      public OMGIDLSchemaField getParent()
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
      public boolean isVector()
      {
         return this.isVector;
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

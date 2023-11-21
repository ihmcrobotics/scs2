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
   private List<OMGIDLField> fields;
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

   public List<OMGIDLField> getFields()
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
      OMGIDLSchema flatSchema = new OMGIDLSchema();
      flatSchema.id = this.id;
      flatSchema.name = this.name;
      flatSchema.isSchemaFlat = true;
      flatSchema.fields = new ArrayList<>();

      for (OMGIDLField field : this.getFields())
      {
         flatSchema.fields.addAll(this.flattenField(field));
      }

      return flatSchema;
   }

   private List<OMGIDLField> flattenField(OMGIDLField field)
   {
      //TODO: (AM) Check correctness and refactor, super ugly code follows

      OMGIDLField flatField = field.clone();

      //TODO: (AM) check that isComplexType is set properly for every non-flat field
      if (!field.isComplexType)
      {
         return Collections.singletonList(flatField);
      }
      List<OMGIDLField> flatFields = new ArrayList<>();
//      flatFields.add(flatField);

      if (flatField.isArray())
      {
         for (int i = 0; i < flatField.getMaxLength(); i++)
         {
            OMGIDLField subField = new OMGIDLField();
            subField.parent = flatField;
            subField.type = flatField.type;
            subField.name = flatField.name + "[" + i + "]";
            subField.isArray = false;
            subField.isVector = false;
            subField.maxLength = -1;
            flatFields.add(subField);
         }
      }
      else
      {
         OMGIDLSchema subSchema = subSchemaMap.get(flatField.getName());
         if (subSchema != null)
         {
            // we are entering a struct definition
            for (OMGIDLField subField : subSchema.getFields())
            {
               if (subSchemaMap.containsKey(subField.getType()))
               {
                  // if this field is a struct of another type, then flatten recursively
                  flatFields.addAll(this.flattenField(subField));
               }
               else
               {
                  // if this field is a base type, then add it to flatfields
                  subField.parent = flatField;
                  //subField.name = flatField.getName() + "." + subField.getName();
                  flatFields.add(subField);
               }
               //               subField.parent = flatField;
               //               subField.name = flatField.getName() + "." + subField.getName();
               //               flatFields.add(subField);
            }
         }
         else
         {
            // entering a struct type instantiation
            flatFields.add(flatField);
            subSchema = subSchemaMap.get(flatField.getType());
            if (subSchema != null)
            {
               for (OMGIDLField subField : subSchema.getFields())
               {
                  if (subSchemaMap.containsKey(subField.getType()))
                  {
                     // if this field is a struct of another type, then flatten recursively
                     flatFields.addAll(this.flattenField(subField));
                  }
                  else
                  {
                     // if this field is a base type, then add it to flatfields
                     subField.parent = flatField;
                     subField.name = flatField.getName() + "." + subField.getName();
                     flatFields.add(subField);
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
         out += "\n\t-fields=\n" + EuclidCoreIOTools.getCollectionString("\n", fields, f -> f.toString(indent + 2));
      if (subSchemaMap != null)
         out += "\n\t-subSchemaMap=\n" + indentString(indent + 2) + EuclidCoreIOTools.getCollectionString("\n" + indentString(indent + 2),
                                                                                                          subSchemaMap.entrySet(),
                                                                                                          e -> e.getKey() + "->\n" + e.getValue()
                                                                                                                                      .toString(indent + 3)
                                                                                                                                      .replace("^(\t*)", ""));
      return indent(out, indent);
   }

   public static class OMGIDLField implements MCAPField
   {
      /**
       * The parent is used when flattening the schema.
       */
      private OMGIDLField parent;
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

      private OMGIDLField()
      {

      }

      protected OMGIDLField(String type, String name, int maxLength, boolean isComplexType)
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

      public OMGIDLField clone()
      {
         OMGIDLField clone = new OMGIDLField();
         clone.parent = parent;
         clone.type = type;
         clone.name = name;
         clone.isArray = isArray;
         clone.maxLength = maxLength;
         clone.isComplexType = isComplexType;
         return clone;
      }

      public OMGIDLField getParent()
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
         out += "\n\t-isComplexType" + isComplexType;
         if (isArray || isVector)
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

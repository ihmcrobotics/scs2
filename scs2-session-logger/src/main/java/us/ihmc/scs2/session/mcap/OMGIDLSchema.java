package us.ihmc.scs2.session.mcap;

import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import org.apache.commons.lang3.NotImplementedException;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLLexer;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLListener;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLParser;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.*;

public class OMGIDLSchema
{
   private int id;
   private String name;
   private List<OMGIDLField> fields;
   private boolean isSchemaFlat;
   private Map<String, OMGIDLSchema> subSchemaMap;

   public static OMGIDLSchema loadSchema(Mcap.Schema mcapSchema) throws IOException
   {
      return loadSchema(mcapSchema.name().str(), mcapSchema.id(), mcapSchema.data());
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
      throw new NotImplementedException();
   }

   private List<OMGIDLField> flattenField(OMGIDLField field)
   {
      throw new NotImplementedException();
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

   public static class OMGIDLField
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
       * This is {@code  true} whenever this field is for an array sequence, or struct.
       */
      private boolean isComplexType;

      private OMGIDLField()
      {

      }

      protected OMGIDLField(String type, String name, int maxLength)
      {
         this.type = type;
         this.name = name;
         this.isArray = maxLength > -1;
         this.maxLength = maxLength;
         //TODO: (AM) don't assume that all scoped types are complex
         this.isComplexType = (isArray || type.contains("::") || type.equals("sequence"));
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

   public static String mcapROS2MessageToString(Mcap.Message message, OMGIDLSchema schema)
   {
      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(message.messageBuffer(), message.offsetData(), message.lengthData());

      String output = mcapROS2MessageToString(cdr, schema, 0);

      cdr.finalize(true);
      return output;
   }

   private static String mcapROS2MessageToString(CDRDeserializer cdr, OMGIDLSchema schema, int indent)
   {
      StringBuilder out = new StringBuilder(schema.getName() + ":");
      for (OMGIDLField field : schema.fields)
      {
         String fieldToString = mcapROS2MessageFieldToString(cdr, field, schema, indent + 1);
         if (fieldToString != null)
            out.append(fieldToString);
      }
      return out.toString();
   }

   private static String mcapROS2MessageFieldToString(CDRDeserializer cdr, OMGIDLField field, OMGIDLSchema schema, int indent)
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
            OMGIDLSchema subSchema = schema.getSubSchemaMap() == null ? null : schema.getSubSchemaMap().get(field.getType());
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

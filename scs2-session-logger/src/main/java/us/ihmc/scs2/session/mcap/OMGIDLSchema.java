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

import static us.ihmc.scs2.session.mcap.MCAPSchema.indent;
import static us.ihmc.scs2.session.mcap.MCAPSchema.indentString;

public class OMGIDLSchema implements MCAPSchema
{
   private int id;
   private String name;
   private List<MCAPSchemaField> fields;
   private boolean isSchemaFlat;
   private Map<String, OMGIDLSchema> subSchemaMap;

   public static OMGIDLSchema loadSchema(MCAP.Schema mcapSchema) throws IOException
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

      CharStream bytesAsChar = CharStreams.fromStream(new ByteArrayInputStream(data));

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
   @Override
   public OMGIDLSchema flattenSchema()
   {
      OMGIDLSchema flatSchema = new OMGIDLSchema();
      flatSchema.id = this.id;
      flatSchema.name = this.name;
      flatSchema.isSchemaFlat = true;
      flatSchema.fields = new ArrayList<>();

      for (MCAPSchemaField field : this.getFields())
      {
         flatSchema.fields.addAll(this.flattenField(field));
      }

      return flatSchema;
   }

   public List<MCAPSchemaField> flattenField(MCAPSchemaField field)
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
         OMGIDLSchema subSchema = subSchemaMap.get(flatField.getType());
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
         out += "\n\t-fields=\n" + EuclidCoreIOTools.getCollectionString("\n", fields, f -> f.toString(indent + 1));
      if (subSchemaMap != null)
         out += "\n\t-subSchemaMap=\n" + indentString(indent + 1) + EuclidCoreIOTools.getCollectionString("\n" + indentString(indent + 1),
                                                                                                          subSchemaMap.entrySet(),
                                                                                                          e -> e.getKey() + "->\n" + e.getValue()
                                                                                                                                      .toString(indent + 2)
                                                                                                                                      .replace("^(\t*)", ""));
      return indent(out, indent);
   }
}

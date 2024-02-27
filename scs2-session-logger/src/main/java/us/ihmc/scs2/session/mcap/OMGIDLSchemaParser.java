package us.ihmc.scs2.session.mcap;

import com.fasterxml.jackson.databind.util.ByteBufferBackedInputStream;
import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLBaseListener;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLLexer;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLListener;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLParser;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLParser.Enum_typeContext;
import us.ihmc.scs2.session.mcap.specs.records.Schema;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;

public class OMGIDLSchemaParser
{
   public static MCAPSchema loadSchema(Schema mcapSchema) throws IOException
   {
      return loadSchema(mcapSchema.name(), mcapSchema.id(), new ByteBufferBackedInputStream(mcapSchema.data()));
   }

   public static MCAPSchema loadSchema(String name, int id, byte[] data) throws IOException
   {
      return loadSchema(name, id, new ByteArrayInputStream(data));
   }

   public static MCAPSchema loadSchema(String name, int id, InputStream is) throws IOException
   {
      MCAPSchema schema = new MCAPSchema(name, id, new ArrayList<>(), new HashMap<>());

      CharStream bytesAsChar = CharStreams.fromStream(is);

      IDLLexer lexer = new IDLLexer(bytesAsChar);
      CommonTokenStream tokens = new CommonTokenStream(lexer);
      IDLParser parser = new IDLParser(tokens);
      parser.setBuildParseTree(true);
      ParseTree tree = parser.specification();

      IDLListener schemaCreatorListener = new SchemaCreatorListener(schema);
      ParseTreeWalker.DEFAULT.walk(schemaCreatorListener, tree);

      return schema;
   }

   private static class SchemaCreatorListener extends IDLBaseListener
   {
      // Each top level struct gets its own schema
      private final MCAPSchema rootSchema;
      private MCAPSchema currentSchema = null;
      private MCAPSchema previousSchema = null;
      private final MemberInfo currentMemberInfo = new MemberInfo();

      // Keeping track of current member attributes
      // These can only be used in exitRule() functions

      private static class MemberInfo
      {
         private String type;
         private String name;
         private int maxLength;
         private boolean isArray;
         private boolean isSequence;
         private boolean isEnum;
         private boolean isComplexType;

         private String[] enumConstants;

         public MemberInfo()
         {
            reset();
         }

         private void reset()
         {
            this.type = null;
            this.name = null;
            this.maxLength = -1;
            this.isArray = false;
            this.isSequence = false;
            this.isEnum = false;
            this.isComplexType = false;
            this.enumConstants = null;
         }

         @Override
         public String toString()
         {
            return "MemberInfo{" + "type='" + type + '\'' + ", name='" + name + '\'' + ", maxLength=" + maxLength + ", isArray=" + isArray + ", isSequence="
                   + isSequence + ", isEnum=" + isEnum + ", isComplexType=" + isComplexType + ", enumConstants=" + Arrays.toString(enumConstants) + '}';
         }
      }

      public SchemaCreatorListener(MCAPSchema schema)
      {
         rootSchema = schema;
         this.currentSchema = schema;
      }

      @Override
      public void exitScoped_name(IDLParser.Scoped_nameContext ctx)
      {
         // TODO: (AM) Handle scoped type names properly, currently forcing global scope resolution
         //      String prefixScope = "";
         //      if (this.previousSchema.getName() != null)
         //      {
         //      prefixScope = this.previousSchema.getName().toString();
         //      }
         //      this.currentMemberType = prefixScope + "::" + ctx.ID(0).getText();
         currentMemberInfo.type = ctx.ID(0).getText();
         currentMemberInfo.isComplexType = true;
      }

      @Override
      public void exitConst_decl(IDLParser.Const_declContext ctx)
      {
         // constants have the type "const base_type"
         MCAPSchemaField newField = new MCAPSchemaField(ctx.identifier().getText(), ctx.const_type().getText(), false, false, -1, false);
         newField.setDefaultValue(ctx.const_exp().getText());
         this.currentSchema.getStaticFields().add(newField);
      }

      @Override
      public void exitBase_type_spec(IDLParser.Base_type_specContext ctx)
      {
         //TODO: (AM) does this always get overridden by the array size?
         currentMemberInfo.type = ctx.getText();
      }

      @Override
      public void exitSimple_declarator(IDLParser.Simple_declaratorContext ctx)
      {
         currentMemberInfo.name = ctx.ID().getText();
      }

      @Override
      public void enterStruct_type(IDLParser.Struct_typeContext ctx)
      {
         String structName = ctx.identifier().getText();

         if (ctx.scoped_name() != null)
            structName += (ctx.DOUBLE_COLON().getText() + ctx.scoped_name().getText());

         // Add a subschema whenever we enter a new struct
         if (Objects.equals(rootSchema.getName(), structName))
         {  // If this is the root schema, then we don't need to create a new schema
            this.previousSchema = this.currentSchema;
            this.currentSchema = rootSchema;
         }
         else
         {
            MCAPSchema newCurrentSchema = new MCAPSchema(structName, 0);
            this.currentSchema.getSubSchemaMap().put(structName, newCurrentSchema);

            this.previousSchema = this.currentSchema;
            this.currentSchema = newCurrentSchema;
         }
      }

      @Override
      public void exitStruct_type(IDLParser.Struct_typeContext ctx)
      {
         // The walker does a recursive dfs traversal, so it will only visit every node once
         this.currentSchema = previousSchema;
         this.previousSchema = null;
      }

      @Override
      public void exitMember(IDLParser.MemberContext ctx)
      {
         assert currentMemberInfo.type != null : String.format("Got a null member type for %s", ctx.type_spec().getText());
         assert currentMemberInfo.name != null : String.format("Got a null member name for %s", ctx.declarators().getText());

         //Create field here for each member
         MCAPSchemaField newField = new MCAPSchemaField(currentMemberInfo.name,
                                                        currentMemberInfo.type,
                                                        currentMemberInfo.isArray,
                                                        currentMemberInfo.isSequence,
                                                        currentMemberInfo.maxLength,
                                                        currentMemberInfo.isComplexType);
         this.currentSchema.getFields().add(newField);

         // reset Member stats after we are done visiting the member
         currentMemberInfo.reset();
      }

      @Override
      public void exitSequence_type(IDLParser.Sequence_typeContext ctx)
      {
         //TODO: (AM) support unbounded sequences
         if (ctx.positive_int_const() != null)
         {
            currentMemberInfo.maxLength = Integer.parseInt(ctx.positive_int_const().getText());
         }
         else
         {
            currentMemberInfo.maxLength = 255;
            LogTools.warn("Unbounded sequences are not supported, limiting max length to 255");
         }

         currentMemberInfo.isSequence = true;
         currentMemberInfo.isComplexType = true;
      }

      @Override
      public void exitEnum_type(Enum_typeContext ctx)
      {
         String name = ctx.identifier().getText();
         MCAPSchema enumSchema = new MCAPSchema(name, -1, ctx.enumerator().stream().map(it -> it.identifier().ID().getText()).toArray(String[]::new));
         this.currentSchema.getSubSchemaMap().put(name, enumSchema);
      }

      @Override
      public void exitString_type(IDLParser.String_typeContext ctx)
      {
         //TODO: (AM) treat unbounded strings properly, right now they are treated like a base type of -1 size
         //this.currentMemberMaxLength = Integer.parseInt(ctx.positive_int_const().getText());
         currentMemberInfo.type = "string";
      }

      @Override
      public void exitArray_declarator(IDLParser.Array_declaratorContext ctx)
      {
         currentMemberInfo.name = ctx.ID().getText();
      }

      @Override
      public void exitFixed_array_size(IDLParser.Fixed_array_sizeContext ctx)
      {
         currentMemberInfo.maxLength = Integer.parseInt(ctx.positive_int_const().getText());
         currentMemberInfo.isComplexType = true;
         currentMemberInfo.isArray = true;
      }
   }
}

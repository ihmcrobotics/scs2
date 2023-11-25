package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.OMGIDLSchema.OMGIDLSchemaField;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLListener;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLParser;

public class SchemaCreatorListener implements IDLListener
{
   OMGIDLSchema schema = null;
   // Sequentially generate IDs for sub-schemas
   int idCount = 1;

   // Each top level struct gets its own schema
   OMGIDLSchema currentSchema = null;
   OMGIDLSchema previousSchema = null;

   // Keeping track of current member attributes
   // These can only be used in exitRule() functions
   int currentMemberMaxLength = -1;
   String currentMemberType = null;
   String currentMemberName = null;
   boolean currentMemberIsComplexType = false;

   public SchemaCreatorListener(OMGIDLSchema schema)
   {
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
      this.currentMemberType = ctx.ID(0).getText();
      this.currentMemberMaxLength = -1;
      this.currentMemberIsComplexType = true;
   }

   @Override
   public void exitConst_decl(IDLParser.Const_declContext ctx)
   {
      // constants have the type "const base_type"
      OMGIDLSchemaField newField = new OMGIDLSchemaField("const " + ctx.const_type().getText(), ctx.identifier().getText(), -1, false);
      this.currentSchema.getFields().add(newField);
   }

   @Override
   public void exitBase_type_spec(IDLParser.Base_type_specContext ctx)
   {
      //TODO: (AM) does this always get overridden by the array size?
      this.currentMemberMaxLength = -1;
      this.currentMemberType = ctx.getText();
      this.currentMemberIsComplexType = false;
   }

   @Override
   public void exitTemplate_type_spec(IDLParser.Template_type_specContext ctx)
   {
      //TODO: (AM) does this always get overridden by other type specs?
      this.currentMemberType = ctx.getText();
   }

   @Override
   public void exitSimple_declarator(IDLParser.Simple_declaratorContext ctx)
   {
      this.currentMemberName = ctx.ID().getText();
   }

   @Override
   public void enterStruct_type(IDLParser.Struct_typeContext ctx)
   {
      String structName = ctx.identifier().getText();
      if (ctx.scoped_name() != null)
         structName += (ctx.DOUBLE_COLON().getText() + ctx.scoped_name().getText());

      // Add a subschema whenever we enter a new struct
      OMGIDLSchema newCurrentSchema = new OMGIDLSchema(structName, this.idCount);

      this.currentSchema.getSubSchemaMap().put(structName, newCurrentSchema);

      if (structName.equals(this.currentSchema.getName()))
      {
         this.currentSchema.getFields().add(new OMGIDLSchema.OMGIDLSchemaField(ctx.KW_STRUCT().getText(), structName, -1, true));
      }

      this.idCount += 1;

      this.previousSchema = this.currentSchema;
      this.currentSchema = newCurrentSchema;
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
      assert this.currentMemberType != null : String.format("Got a null member type for %s", ctx.type_spec().getText());
      assert this.currentMemberName != null : String.format("Got a null member name for %s", ctx.declarators().getText());

      //Create field here for each member
      OMGIDLSchemaField newField = new OMGIDLSchemaField(this.currentMemberType,
                                                         this.currentMemberName,
                                                         this.currentMemberMaxLength,
                                                         this.currentMemberIsComplexType);
      this.currentSchema.getFields().add(newField);

      // reset Member stats after we are done visiting the member
      this.currentMemberMaxLength = -1;
      this.currentMemberType = "";
      this.currentMemberName = "";
      this.currentMemberIsComplexType = false;
   }

   @Override
   public void exitSequence_type(IDLParser.Sequence_typeContext ctx)
   {
      //TODO: (AM) support unbounded sequences
      if (ctx.positive_int_const() != null)
      {
         this.currentMemberMaxLength = Integer.parseInt(ctx.positive_int_const().getText());
      }
      else
      {
         this.currentMemberMaxLength = 255;
         System.out.println("Unbounded sequences are not supported, limiting max length to 255");
      }
      //this.currentMemberType = ctx.KW_SEQUENCE().getText();
      this.currentMemberIsComplexType = true;
   }

   @Override
   public void exitString_type(IDLParser.String_typeContext ctx)
   {
      //TODO: (AM) treat unbounded strings properly, right now they are treated like a base type of -1 size
      //this.currentMemberMaxLength = Integer.parseInt(ctx.positive_int_const().getText());
      this.currentMemberType = "string";
      this.currentMemberMaxLength = -1;
      this.currentMemberIsComplexType = false;
   }

   @Override
   public void exitArray_declarator(IDLParser.Array_declaratorContext ctx)
   {
      this.currentMemberName = ctx.ID().getText();
   }

   @Override
   public void exitFixed_array_size(IDLParser.Fixed_array_sizeContext ctx)
   {
      this.currentMemberMaxLength = Integer.parseInt(ctx.positive_int_const().getText());
      this.currentMemberIsComplexType = true;
   }
}

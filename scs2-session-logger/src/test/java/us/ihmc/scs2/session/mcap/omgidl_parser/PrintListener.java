package us.ihmc.scs2.session.mcap.omgidl_parser;

public class PrintListener extends IDLBaseListener
{

   @Override
   public void exitScoped_name(IDLParser.Scoped_nameContext ctx)
   {
      //      System.out.println(ctx.getText());
   }

   @Override
   public void exitFloating_pt_type(IDLParser.Floating_pt_typeContext ctx)
   {
      //      System.out.println(ctx.getText());
   }

   @Override
   public void enterSigned_short_int(IDLParser.Signed_short_intContext ctx)
   {
      //System.out.println(ctx.toString());
   }

   @Override
   public void exitSigned_short_int(IDLParser.Signed_short_intContext ctx)
   {
      //      System.out.println(ctx.getText());
   }

   @Override
   public void exitUnsigned_int(IDLParser.Unsigned_intContext ctx)
   {
      //      System.out.println(ctx.getText());
   }

   @Override
   public void exitChar_type(IDLParser.Char_typeContext ctx)
   {
      //      System.out.println(ctx.getText());
   }

   @Override
   public void exitBoolean_type(IDLParser.Boolean_typeContext ctx)
   {
      //      System.out.println(ctx.getText());
   }

   @Override
   public void exitOctet_type(IDLParser.Octet_typeContext ctx)
   {
      //      System.out.println(ctx.getText());
   }

   @Override
   public void enterStruct_type(IDLParser.Struct_typeContext ctx)
   {
      System.out.println("\n\n" + ctx.identifier().getText());
   }

   @Override
   public void exitStruct_type(IDLParser.Struct_typeContext ctx)
   {
      //System.out.println(ctx.member_list().getText());
   }

   @Override
   public void exitMember_list(IDLParser.Member_listContext ctx)
   {
      //      for(IDLParser.MemberContext m: ctx.member())
      //      {
      //         this.enterMember(m);
      //      }
   }

   @Override
   public void exitMember(IDLParser.MemberContext ctx)
   {
      System.out.println(String.format("%s is of type %s", ctx.declarators().getText(), ctx.type_spec().getText()));
   }

   @Override
   public void exitString_type(IDLParser.String_typeContext ctx)
   {
      //System.out.println(ctx.getText());
   }

   @Override
   public void exitArray_declarator(IDLParser.Array_declaratorContext ctx)
   {
      //      System.out.println(ctx.getText());
      //      System.out.println(ctx.getParent().getParent().getParent().getText());
   }
}

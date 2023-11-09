package us.ihmc.scs2.session.mcap;

import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLLexer;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLParser;
import us.ihmc.scs2.session.mcap.omgidl_parser.PrintListener;

import java.io.InputStream;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

public class OMGIDLSchemaTest
{
   @Test
   @Disabled
   public void testOMGIDLParser() throws Exception
   {
      String schemaName = "simple-idl-with-structs.idl";
      CharStream bytesAsChar = CharStreams.fromStream(getClass().getResourceAsStream(schemaName));
      IDLLexer lexer = new IDLLexer(bytesAsChar);
      CommonTokenStream tokens = new CommonTokenStream(lexer);
      IDLParser parser = new IDLParser(tokens);
      parser.setBuildParseTree(true);
      ParseTree tree = parser.specification();

      PrintListener printListener = new PrintListener();
      ParseTreeWalker.DEFAULT.walk(printListener, tree);
   }

   @Test
   public void testLoadSchemaFromFile() throws Exception
   {
      String schemaName = "simple-idl-with-structs";
      InputStream is = getClass().getResourceAsStream(schemaName + ".idl");
      OMGIDLSchema schema = OMGIDLSchema.loadSchema(schemaName, 0, is.readAllBytes());

      // Anything with structures in it is not flat
      assertFalse(schema.isSchemaFlat());

      // No const support yet
      assertTrue(schema.getFields().isEmpty());

      // Check that sub-schemas for structures exist
      assertTrue(schema.getSubSchemaMap().containsKey("struct_1"));
      assertTrue(schema.getSubSchemaMap().containsKey("struct_2"));

      // Check fields in the subschemas
      OMGIDLSchema expectedSubSchema = schema.getSubSchemaMap().get("struct_1");
      assertTrue(expectedSubSchema.isSchemaFlat());
      assertEquals(6, expectedSubSchema.getFields().size());
      assertEquals(1, expectedSubSchema.getId());

      // short signed_short_var;
      assertTrue(expectedSubSchema.getFields().get(0).getName().equals("signed_short_var"));
      assertTrue(expectedSubSchema.getFields().get(0).getType().equals("short"));
      assertFalse(expectedSubSchema.getFields().get(0).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(0).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(0).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(0).getParent() == null);

      // unsigned short unsigned_short_var;
      assertTrue(expectedSubSchema.getFields().get(1).getName().equals("unsigned_short_var"));
      assertTrue(expectedSubSchema.getFields().get(1).getType().equals("unsignedshort"));
      assertFalse(expectedSubSchema.getFields().get(1).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(1).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(1).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(1).getParent() == null);

      // octet octet_var;
      assertTrue(expectedSubSchema.getFields().get(2).getName().equals("octet_var"));
      assertTrue(expectedSubSchema.getFields().get(2).getType().equals("octet"));
      assertFalse(expectedSubSchema.getFields().get(2).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(2).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(2).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(2).getParent() == null);

      // char char_var;
      assertTrue(expectedSubSchema.getFields().get(3).getName().equals("char_var"));
      assertTrue(expectedSubSchema.getFields().get(3).getType().equals("char"));
      assertFalse(expectedSubSchema.getFields().get(3).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(3).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(3).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(3).getParent() == null);

      // string string_var;
      assertTrue(expectedSubSchema.getFields().get(4).getName().equals("string_var"));
      assertTrue(expectedSubSchema.getFields().get(4).getType().equals("string"));
      // TODO: (AM) treat string like a complex type?
      assertFalse(expectedSubSchema.getFields().get(4).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(4).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(4).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(4).getParent() == null);

      // sequence<double, 2> sequence_var;
      assertTrue(expectedSubSchema.getFields().get(5).getName().equals("sequence_var"));
      assertTrue(expectedSubSchema.getFields().get(5).getType().equals("sequence"));
      assertTrue(expectedSubSchema.getFields().get(5).isComplexType());
      assertTrue(expectedSubSchema.getFields().get(5).isArray());
      assertEquals(27, expectedSubSchema.getFields().get(5).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(5).getParent() == null);

      // Check the second sub-schema
      expectedSubSchema = schema.getSubSchemaMap().get("struct_2");
      assertTrue(expectedSubSchema.isSchemaFlat());
      assertEquals(5, expectedSubSchema.getFields().size());
      assertEquals(2, expectedSubSchema.getId());

      // float float_var;
      assertTrue(expectedSubSchema.getFields().get(0).getName().equals("float_var"));
      assertTrue(expectedSubSchema.getFields().get(0).getType().equals("float"));
      assertFalse(expectedSubSchema.getFields().get(0).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(0).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(0).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(0).getParent() == null);

      // boolean boolean_var;
      assertTrue(expectedSubSchema.getFields().get(1).getName().equals("boolean_var"));
      assertTrue(expectedSubSchema.getFields().get(1).getType().equals("boolean"));
      assertFalse(expectedSubSchema.getFields().get(1).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(1).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(1).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(1).getParent() == null);

      // long long_array_var[13];
      assertTrue(expectedSubSchema.getFields().get(2).getName().equals("long_array_var"));
      assertTrue(expectedSubSchema.getFields().get(2).getType().equals("long"));
      assertTrue(expectedSubSchema.getFields().get(2).isComplexType());
      assertTrue(expectedSubSchema.getFields().get(2).isArray());
      assertEquals(13, expectedSubSchema.getFields().get(2).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(2).getParent() == null);

      // struct_1 strut_1_var;
      assertTrue(expectedSubSchema.getFields().get(3).getName().equals("struct_1_var"));
      assertTrue(expectedSubSchema.getFields().get(3).getType().equals("simple-idl-with-structs::struct_1"));
      assertTrue(expectedSubSchema.getFields().get(3).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(3).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(3).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(3).getParent() == null);


      // long double long_double_var;
      assertTrue(expectedSubSchema.getFields().get(4).getName().equals("long_double_var"));
      assertTrue(expectedSubSchema.getFields().get(4).getType().equals("longdouble"));
      assertFalse(expectedSubSchema.getFields().get(4).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(4).isArray());
      assertEquals(-1, expectedSubSchema.getFields().get(4).getMaxLength());
      assertTrue(expectedSubSchema.getFields().get(4).getParent() == null);

   }

}

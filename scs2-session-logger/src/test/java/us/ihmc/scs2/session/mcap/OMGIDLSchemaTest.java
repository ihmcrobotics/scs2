package us.ihmc.scs2.session.mcap;

import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLLexer;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLParser;
import us.ihmc.scs2.session.mcap.omgidl_parser.PrintListener;

import java.io.InputStream;

import static org.junit.jupiter.api.Assertions.*;

public class OMGIDLSchemaTest
{
   @Test
   @Disabled
   public void testOMGIDLParser() throws Exception
   {
      String schemaName = "simple-idl-one-struct.idl";
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
      String schemaName = "simple_idl_with_structs";
      InputStream is = getClass().getResourceAsStream(schemaName + ".idl");
      OMGIDLSchema schema = OMGIDLSchema.loadSchema(schemaName, 0, is.readAllBytes());

      // Anything with structures in it is not flat
      assertFalse(schema.isSchemaFlat());
      assertEquals(2, schema.getFields().size());
      assertEquals("double_const", schema.getFields().get(0).getName());
      assertEquals("simple_idl_with_structs", schema.getFields().get(1).getName());

      // Check const field
      assertEquals("double_const", schema.getFields().get(0).getName());
      assertEquals("const double", schema.getFields().get(0).getType());
      assertFalse(schema.getFields().get(0).isComplexType());
      assertFalse(schema.getFields().get(0).isArray());
      assertEquals(-1, schema.getFields().get(0).getMaxLength());
      assertNull(schema.getFields().get(0).getParent());

      // Check that sub-schemas for structures exist
      assertTrue(schema.getSubSchemaMap().containsKey("struct_1"));
      assertTrue(schema.getSubSchemaMap().containsKey("simple_idl_with_structs"));

      // Check fields in the subschemas
      //TODO: (AM) Check isVector for everything
      OMGIDLSchema expectedSubSchema = schema.getSubSchemaMap().get("struct_1");
      assertTrue(expectedSubSchema.isSchemaFlat());
      assertEquals(6, expectedSubSchema.getFields().size());
      assertEquals(1, expectedSubSchema.getId());

      // short signed_short_var;
      assertEquals("signed_short_var", expectedSubSchema.getFields().get(0).getName());
      assertEquals("short", expectedSubSchema.getFields().get(0).getType());
      assertFalse(expectedSubSchema.getFields().get(0).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(0).isArray());
      assertFalse(expectedSubSchema.getFields().get(0).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(0).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(0).getParent());

      // unsigned short unsigned_short_var;
      assertEquals("unsigned_short_var", expectedSubSchema.getFields().get(1).getName());
      assertEquals("unsignedshort", expectedSubSchema.getFields().get(1).getType());
      assertFalse(expectedSubSchema.getFields().get(1).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(1).isArray());
      assertFalse(expectedSubSchema.getFields().get(1).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(1).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(1).getParent());

      // octet octet_var;
      assertEquals("octet_var", expectedSubSchema.getFields().get(2).getName());
      assertEquals("octet", expectedSubSchema.getFields().get(2).getType());
      assertFalse(expectedSubSchema.getFields().get(2).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(2).isArray());
      assertFalse(expectedSubSchema.getFields().get(2).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(2).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(2).getParent());

      // char char_var;
      assertEquals("char_var", expectedSubSchema.getFields().get(3).getName());
      assertEquals("char", expectedSubSchema.getFields().get(3).getType());
      assertFalse(expectedSubSchema.getFields().get(3).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(3).isArray());
      assertFalse(expectedSubSchema.getFields().get(3).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(3).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(3).getParent());

      // string string_var;
      assertEquals("string_var", expectedSubSchema.getFields().get(4).getName());
      assertEquals("string", expectedSubSchema.getFields().get(4).getType());
      // TODO: (AM) treat string like a complex type?
      assertFalse(expectedSubSchema.getFields().get(4).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(4).isArray());
      assertFalse(expectedSubSchema.getFields().get(4).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(4).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(4).getParent());

      // sequence<double, 27> sequence_var;
      assertEquals("sequence_var", expectedSubSchema.getFields().get(5).getName());
      assertEquals("sequence<double,27>", expectedSubSchema.getFields().get(5).getType());
      assertTrue(expectedSubSchema.getFields().get(5).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(5).isArray());
      assertTrue(expectedSubSchema.getFields().get(5).isVector());
      assertEquals(27, expectedSubSchema.getFields().get(5).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(5).getParent());

      // Check the second sub-schema
      expectedSubSchema = schema.getSubSchemaMap().get("simple_idl_with_structs");
      assertTrue(expectedSubSchema.isSchemaFlat());
      assertEquals(5, expectedSubSchema.getFields().size());
      assertEquals(2, expectedSubSchema.getId());

      // float float_var;
      assertEquals("float_var", expectedSubSchema.getFields().get(0).getName());
      assertEquals("float", expectedSubSchema.getFields().get(0).getType());
      assertFalse(expectedSubSchema.getFields().get(0).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(0).isArray());
      assertFalse(expectedSubSchema.getFields().get(0).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(0).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(0).getParent());

      // boolean boolean_var;
      assertEquals("boolean_var", expectedSubSchema.getFields().get(1).getName());
      assertEquals("boolean", expectedSubSchema.getFields().get(1).getType());
      assertFalse(expectedSubSchema.getFields().get(1).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(1).isArray());
      assertFalse(expectedSubSchema.getFields().get(1).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(1).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(1).getParent());

      // long long_array_var[13];
      assertEquals("long_array_var", expectedSubSchema.getFields().get(2).getName());
      assertEquals("long", expectedSubSchema.getFields().get(2).getType());
      assertTrue(expectedSubSchema.getFields().get(2).isComplexType());
      assertTrue(expectedSubSchema.getFields().get(2).isArray());
      assertFalse(expectedSubSchema.getFields().get(2).isVector());
      assertEquals(13, expectedSubSchema.getFields().get(2).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(2).getParent());

      // struct_1 strut_1_var;
      assertEquals("struct_1_var", expectedSubSchema.getFields().get(3).getName());
      assertEquals("struct_1", expectedSubSchema.getFields().get(3).getType());
      assertTrue(expectedSubSchema.getFields().get(3).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(3).isArray());
      assertFalse(expectedSubSchema.getFields().get(3).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(3).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(3).getParent());

      // long double long_double_var;
      assertEquals("long_double_var", expectedSubSchema.getFields().get(4).getName());
      assertEquals("longdouble", expectedSubSchema.getFields().get(4).getType());
      assertFalse(expectedSubSchema.getFields().get(4).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(4).isArray());
      assertFalse(expectedSubSchema.getFields().get(4).isVector());
      assertEquals(-1, expectedSubSchema.getFields().get(4).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(4).getParent());
   }

   @Disabled
   @Test
   public void testFlattenSchema() throws Exception
   {
      //TODO: (AM) implement test
      String schemaName = "vision_msgs::LandmarkInfoArray";
      InputStream is = getClass().getResourceAsStream(schemaName + ".idl");
      OMGIDLSchema schema = OMGIDLSchema.loadSchema(schemaName, 0, is.readAllBytes());
      OMGIDLSchema flatSchema = schema.flattenSchema();
   }
}

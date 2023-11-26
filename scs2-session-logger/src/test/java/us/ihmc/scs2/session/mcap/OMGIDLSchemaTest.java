package us.ihmc.scs2.session.mcap;

import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLLexer;
import us.ihmc.scs2.session.mcap.omgidl_parser.IDLParser;
import us.ihmc.scs2.session.mcap.omgidl_parser.PrintListener;

import java.io.InputStream;
import java.util.List;

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
      MCAPSchema schema = OMGIDLSchemaParser.loadSchema(schemaName, 0, is.readAllBytes());

      // Anything with structures in it is not flat
      assertFalse(schema.isSchemaFlat());
      assertEquals(1, schema.getStaticFields().size());
      assertEquals(5, schema.getFields().size());
      assertEquals("double_const", schema.getStaticFields().get(0).getName());
      assertEquals("6.9", schema.getStaticFields().get(0).getDefaultValue());
      assertEquals("float_var", schema.getFields().get(0).getName());
      assertEquals("boolean_var", schema.getFields().get(1).getName());
      assertEquals("long_array_var", schema.getFields().get(2).getName());
      assertEquals("struct_1_var", schema.getFields().get(3).getName());
      assertEquals("long_double_var", schema.getFields().get(4).getName());

      // Check const field
      assertEquals("double_const", schema.getStaticFields().get(0).getName());
      assertEquals("double", schema.getStaticFields().get(0).getType());
      assertFalse(schema.getFields().get(0).isComplexType());
      assertFalse(schema.getFields().get(0).isArray());
      assertEquals(-1, schema.getFields().get(0).getMaxLength());
      assertNull(schema.getFields().get(0).getParent());

      // Check that sub-schemas for structures exist
      assertTrue(schema.getSubSchemaMap().containsKey("struct_1"));

      // Check fields in the subschemas
      //TODO: (AM) Check isVector for everything
      MCAPSchema expectedSubSchema = schema.getSubSchemaMap().get("struct_1");
      assertTrue(expectedSubSchema.isSchemaFlat());
      assertEquals(6, expectedSubSchema.getFields().size());

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
      assertEquals("double", expectedSubSchema.getFields().get(5).getType());
      assertTrue(expectedSubSchema.getFields().get(5).isComplexType());
      assertFalse(expectedSubSchema.getFields().get(5).isArray());
      assertTrue(expectedSubSchema.getFields().get(5).isVector());
      assertEquals(27, expectedSubSchema.getFields().get(5).getMaxLength());
      assertNull(expectedSubSchema.getFields().get(5).getParent());

      // Check the main schema
      assertFalse(schema.isSchemaFlat());
      assertEquals(5, schema.getFields().size());

      // float float_var;
      assertEquals("float_var", schema.getFields().get(0).getName());
      assertEquals("float", schema.getFields().get(0).getType());
      assertFalse(schema.getFields().get(0).isComplexType());
      assertFalse(schema.getFields().get(0).isArray());
      assertFalse(schema.getFields().get(0).isVector());
      assertEquals(-1, schema.getFields().get(0).getMaxLength());
      assertNull(schema.getFields().get(0).getParent());

      // boolean boolean_var;
      assertEquals("boolean_var", schema.getFields().get(1).getName());
      assertEquals("boolean", schema.getFields().get(1).getType());
      assertFalse(schema.getFields().get(1).isComplexType());
      assertFalse(schema.getFields().get(1).isArray());
      assertFalse(schema.getFields().get(1).isVector());
      assertEquals(-1, schema.getFields().get(1).getMaxLength());
      assertNull(schema.getFields().get(1).getParent());

      // long long_array_var[13];
      assertEquals("long_array_var", schema.getFields().get(2).getName());
      assertEquals("long", schema.getFields().get(2).getType());
      assertTrue(schema.getFields().get(2).isComplexType());
      assertTrue(schema.getFields().get(2).isArray());
      assertFalse(schema.getFields().get(2).isVector());
      assertEquals(13, schema.getFields().get(2).getMaxLength());
      assertNull(schema.getFields().get(2).getParent());

      // struct_1 strut_1_var;
      assertEquals("struct_1_var", schema.getFields().get(3).getName());
      assertEquals("struct_1", schema.getFields().get(3).getType());
      assertTrue(schema.getFields().get(3).isComplexType());
      assertFalse(schema.getFields().get(3).isArray());
      assertFalse(schema.getFields().get(3).isVector());
      assertEquals(-1, schema.getFields().get(3).getMaxLength());
      assertNull(schema.getFields().get(3).getParent());

      // long double long_double_var;
      assertEquals("long_double_var", schema.getFields().get(4).getName());
      assertEquals("longdouble", schema.getFields().get(4).getType());
      assertFalse(schema.getFields().get(4).isComplexType());
      assertFalse(schema.getFields().get(4).isArray());
      assertFalse(schema.getFields().get(4).isVector());
      assertEquals(-1, schema.getFields().get(4).getMaxLength());
      assertNull(schema.getFields().get(4).getParent());
   }

   @Disabled
   @Test
   public void testFlattenSchema() throws Exception
   {
      //TODO: (AM) implement test
      String schemaName = "NavigationAppHealth";
      InputStream is = getClass().getResourceAsStream(schemaName + ".idl");
      MCAPSchema schema = OMGIDLSchemaParser.loadSchema(schemaName, 0, is.readAllBytes());
      MCAPSchema flatSchema = schema.flattenSchema();
      //      System.out.println(schema.toString());
      System.out.println(flatSchema);
   }

   @Disabled
   @Test
   public void testFlattenArray() throws Exception
   {
      String schemaName = "flatten_array_test";
      InputStream is = getClass().getResourceAsStream(schemaName + ".idl");
      MCAPSchema schema = OMGIDLSchemaParser.loadSchema(schemaName, 0, is.readAllBytes());
      //      OMGIDLSchemaParser subSchema = schema.getSubSchemaMap().get("a");
      MCAPSchemaField field = schema.getSubSchemaMap().get(schemaName).getFields().get(0);
      List<MCAPSchemaField> flatFields = schema.flattenField(field);
      //      System.out.println(subSchema.flattenSchema(schema.getSubSchemaMap()));
      System.out.println(flatFields);

      //      OMGIDLSchemaParser flatSchema = schema.flattenSchema();
      //      System.out.println(schema.flattenSchema());
   }
}

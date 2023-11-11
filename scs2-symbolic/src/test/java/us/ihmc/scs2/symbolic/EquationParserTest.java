package us.ihmc.scs2.symbolic;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.symbolic.EquationBuilder.EquationBuilderException;
import us.ihmc.scs2.symbolic.EquationInput.DoubleVariable;
import us.ihmc.scs2.symbolic.EquationInput.IntegerVariable;
import us.ihmc.scs2.symbolic.EquationInput.Type;
import us.ihmc.scs2.symbolic.parser.EquationParseError;
import us.ihmc.scs2.symbolic.parser.EquationParser;

import java.util.StringTokenizer;

import static org.junit.jupiter.api.Assertions.*;

public class EquationParserTest
{

   @Test
   public void testSimpleExamples()
   {
      EquationParser parser = new EquationParser();

      EquationBuilder equationBuilder = parser.parse("x = 2.0*a+b");
      try
      {
         equationBuilder.build();
         fail("Should have thrown an exception");
      }
      catch (EquationBuilderException e)
      {
         assertEquals("Missing inputs: [a, b, x]", e.getMessage());
      }

      equationBuilder.getAliasManager().addConstant("a", 0.2);
      equationBuilder.getAliasManager().addConstant("b", 10.0);
      equationBuilder.getAliasManager().addVariable("x", Type.DOUBLE);
      Equation compile = equationBuilder.build();
      compile.compute();
      assertEquals(10.4, ((DoubleVariable) compile.getResult()).getAsDouble());

      parser.getAliasManager().addConstant("a", 0.2);
      parser.getAliasManager().addConstant("b", 10.0);
      parser.getAliasManager().addVariable("x", Type.DOUBLE);

      compile = parser.parse("x = -2.0*a+b").build();
      compile.compute();
      assertEquals(9.6, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = parser.parse("x = 2.0e-04*a+b").build();
      compile.compute();
      assertEquals(10.00004, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = parser.parse("x = 1-2.0e-04*a+b").build();
      compile.compute();
      assertEquals(10.99996, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = parser.parse("x = 1-2.0e-04*a+b/2").build();
      compile.compute();
      assertEquals(5.99996, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = parser.parse("x = 0.5 * (2.0 *a + 2*b)").build();
      compile.compute();
      assertEquals(10.2, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = parser.parse("x = atan2(a, b)").build();
      compile.compute();
      assertEquals(0.019997333973150535, ((DoubleVariable) compile.getResult()).getAsDouble());

      assertThrows(EquationParseError.class, () -> parser.parse("x = 0.5 ** (2.0 *a + 2*b)").build());
      assertThrows(EquationParseError.class, () -> parser.parse("x = 0.5 // (2.0 *a + 2*b)").build());
      assertThrows(EquationParseError.class, () -> parser.parse("x = 0.5.0 * (2.0 *a + 2*b)").build());
      assertThrows(EquationParseError.class, () -> parser.parse("x = 0.5 * (2.0 *a + 2*b").build());

      EquationParser parser2 = new EquationParser();
      parser2.getAliasManager().addConstant("a", 0.2);
      parser2.getAliasManager().addConstant("b", 10.0);
      parser2.getAliasManager().addVariable("x", Type.INTEGER);
      compile = parser2.parse("x = 1 - 2 * 2 + 4").build();
      compile.compute();
      assertEquals(1, ((IntegerVariable) compile.getResult()).getAsInt());

      compile = parser2.parse("x = 4 / 4 * 4").build();
      compile.compute();
      assertEquals(4, ((IntegerVariable) compile.getResult()).getAsInt());

      compile = parser2.parse("x = 4 / 4 * 4 / 4").build();
      compile.compute();
      assertEquals(1, ((IntegerVariable) compile.getResult()).getAsInt());

      compile = parser2.parse("x = 4 * 4 / 4 - 2 + 3 - 3 + 2").build();
      compile.compute();
      assertEquals(4, ((IntegerVariable) compile.getResult()).getAsInt());
   }

   public static void main(String[] args)
   {
      StringTokenizer st = new StringTokenizer("x = 2.0*a+b");
      while (st.hasMoreTokens())
         System.out.println(st.nextToken());
   }
}

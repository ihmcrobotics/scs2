package us.ihmc.scs2.symbolic;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.StringTokenizer;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.symbolic.EquationBuilder.EquationBuilderException;
import us.ihmc.scs2.symbolic.EquationInput.DoubleInput;
import us.ihmc.scs2.symbolic.EquationInput.InputType;
import us.ihmc.scs2.symbolic.EquationInput.IntegerInput;
import us.ihmc.scs2.symbolic.parser.EquationParseError;
import us.ihmc.scs2.symbolic.parser.EquationParser;

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

      double time = 0.0;

      equationBuilder.getAliasManager().addConstant("a", 0.2);
      equationBuilder.getAliasManager().addConstant("b", 10.0);
      equationBuilder.getAliasManager().addAlias("x", InputType.DOUBLE);
      Equation equation = new Equation(null, null, equationBuilder);
      equation.compute(time);
      assertEquals(10.4, ((DoubleInput) equation.getResult()).getValue());

      parser.getAliasManager().addConstant("a", 0.2);
      parser.getAliasManager().addConstant("b", 10.0);
      parser.getAliasManager().addAlias("x", InputType.DOUBLE);

      equation = Equation.parse("x = -2.0*a+b", parser);
      equation.compute(time);
      assertEquals(9.6, ((DoubleInput) equation.getResult()).getValue());

      equation = Equation.parse("x = 2.0e-04*a+b", parser);
      equation.compute(time);
      assertEquals(10.00004, ((DoubleInput) equation.getResult()).getValue());

      equation = Equation.parse("x = 1-2.0e-04*a+b", parser);
      equation.compute(time);
      assertEquals(10.99996, ((DoubleInput) equation.getResult()).getValue());

      equation = Equation.parse("x = 1-2.0e-04*a+b/2", parser);
      equation.compute(time);
      assertEquals(5.99996, ((DoubleInput) equation.getResult()).getValue());

      equation = Equation.parse("x = 0.5 * (2.0 *a + 2*b)", parser);
      equation.compute(time);
      assertEquals(10.2, ((DoubleInput) equation.getResult()).getValue());

      equation = Equation.parse("x = atan2(a, b)", parser);
      equation.compute(time);
      assertEquals(0.019997333973150535, ((DoubleInput) equation.getResult()).getValue());

      assertThrows(EquationParseError.class, () -> Equation.parse("x = 0.5 ** (2.0 *a + 2*b)"));
      assertThrows(EquationParseError.class, () -> Equation.parse("x = 0.5 // (2.0 *a + 2*b)"));
      assertThrows(EquationParseError.class, () -> Equation.parse("x = 0.5.0 * (2.0 *a + 2*b)"));
      assertThrows(EquationParseError.class, () -> Equation.parse("x = 0.5 * (2.0 *a + 2*b"));

      EquationParser parser2 = new EquationParser();
      parser2.getAliasManager().addConstant("a", 0.2);
      parser2.getAliasManager().addConstant("b", 10.0);
      parser2.getAliasManager().addAlias("x", InputType.INTEGER);
      equation = Equation.parse("x = 1 - 2 * 2 + 4", parser2);
      equation.compute(time);
      assertEquals(1, ((IntegerInput) equation.getResult()).getValue());

      equation = Equation.parse("x = 4 / 4 * 4", parser2);
      equation.compute(time);
      assertEquals(4, ((IntegerInput) equation.getResult()).getValue());

      equation = Equation.parse("x = 4 / 4 * 4 / 4", parser2);
      equation.compute(time);
      assertEquals(1, ((IntegerInput) equation.getResult()).getValue());

      equation = Equation.parse("x = 4 * 4 / 4 - 2 + 3 - 3 + 2", parser2);
      equation.compute(time);
      assertEquals(4, ((IntegerInput) equation.getResult()).getValue());
   }

   public static void main(String[] args)
   {
      StringTokenizer st = new StringTokenizer("x = 2.0*a+b");
      while (st.hasMoreTokens())
         System.out.println(st.nextToken());
   }
}

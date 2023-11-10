package us.ihmc.scs2.symbolic.parser;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.symbolic.parser.EquationInput.DoubleVariable;
import us.ihmc.scs2.symbolic.parser.EquationInput.IntegerVariable;
import us.ihmc.scs2.symbolic.parser.parser.EquationParseError;
import us.ihmc.scs2.symbolic.parser.parser.EquationParser;

import java.util.StringTokenizer;

import static org.junit.jupiter.api.Assertions.*;

public class EquationParserTest
{

   @Test
   public void testSimpleExamples()
   {
      EquationParser parser = new EquationParser();
      parser.getAliasManager().addConstant("a", 0.2);
      parser.getAliasManager().addConstant("b", 10.0);
      Equation compile = Equation.parse("x = 2.0*a+b", parser);
      compile.compute();
      assertEquals(10.4, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = Equation.parse("x = -2.0*a+b", parser);
      compile.compute();
      assertEquals(9.6, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = Equation.parse("x = 2.0e-04*a+b", parser);
      compile.compute();
      assertEquals(10.00004, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = Equation.parse("x = 1 - 2 * 2 + 4", parser);
      compile.compute();
      assertEquals(1, ((IntegerVariable) compile.getResult()).getAsInt());

      compile = Equation.parse("x = 4 / 4 * 4", parser);
      compile.compute();
      assertEquals(4, ((IntegerVariable) compile.getResult()).getAsInt());

      compile = Equation.parse("x = 4 / 4 * 4 / 4", parser);
      compile.compute();
      assertEquals(1, ((IntegerVariable) compile.getResult()).getAsInt());

      compile = Equation.parse("x = 4 * 4 / 4 - 2 + 3 - 3 + 2", parser);
      compile.compute();
      assertEquals(4, ((IntegerVariable) compile.getResult()).getAsInt());

      compile = Equation.parse("x = 1-2.0e-04*a+b", parser);
      compile.compute();
      assertEquals(10.99996, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = Equation.parse("x = 1-2.0e-04*a+b/2", parser);
      compile.compute();
      assertEquals(5.99996, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = Equation.parse("x = 0.5 * (2.0 *a + 2*b)", parser);
      compile.compute();
      assertEquals(10.2, ((DoubleVariable) compile.getResult()).getAsDouble());

      compile = Equation.parse("x = atan2(a, b)", parser);
      compile.compute();
      assertEquals(0.019997333973150535, ((DoubleVariable) compile.getResult()).getAsDouble());

      assertThrows(EquationParseError.class, () -> Equation.parse("x = 0.5 ** (2.0 *a + 2*b)", parser));
      assertThrows(EquationParseError.class, () -> Equation.parse("x = 0.5 // (2.0 *a + 2*b)", parser));
      assertThrows(EquationParseError.class, () -> Equation.parse("x = 0.5.0 * (2.0 *a + 2*b)", parser));
      assertThrows(EquationParseError.class, () -> Equation.parse("x = 0.5 * (2.0 *a + 2*b", parser));
   }

   public static void main(String[] args)
   {
      StringTokenizer st = new StringTokenizer("x = 2.0*a+b");
      while (st.hasMoreTokens())
         System.out.println(st.nextToken());
   }
}

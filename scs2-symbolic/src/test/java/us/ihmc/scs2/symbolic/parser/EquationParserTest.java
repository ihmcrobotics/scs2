package us.ihmc.scs2.symbolic.parser;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.symbolic.parser.EquationVariable.EquationVariableDouble;
import us.ihmc.scs2.symbolic.parser.parser.EquationParseError;
import us.ihmc.scs2.symbolic.parser.parser.EquationParser;

import java.util.StringTokenizer;

import static org.junit.jupiter.api.Assertions.*;

public class EquationParserTest
{

   @Test
   public void testSimpleExamples()
   {
      EquationParser yoEquation = new EquationParser();
      yoEquation.alias("a", 0.2);
      yoEquation.alias("b", 10.0);
      Equation compile = yoEquation.parse("x = 2.0*a+b");
      compile.compute();
      assertEquals(10.4, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = -2.0*a+b");
      compile.compute();
      assertEquals(9.6, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = 2.0e-04*a+b");
      compile.compute();
      assertEquals(10.00004, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = 1 - 2 * 2 + 4");
      compile.compute();
      assertEquals(1, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = 4 / 4 * 4");
      compile.compute();
      assertEquals(4, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = 4 / 4 * 4 / 4");
      compile.compute();
      assertEquals(1, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = 4 * 4 / 4 - 2 + 3 - 3 + 2");
      compile.compute();
      assertEquals(4, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = 1-2.0e-04*a+b");
      compile.compute();
      assertEquals(10.99996, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = 1-2.0e-04*a+b/2");
      compile.compute();
      assertEquals(5.99996, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = 0.5 * (2.0 *a + 2*b)");
      compile.compute();
      assertEquals(10.2, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      compile = yoEquation.parse("x = atan2(a, b)");
      compile.compute();
      assertEquals(0.019997333973150535, ((EquationVariableDouble) compile.getResult()).getAsDouble());

      assertThrows(EquationParseError.class, () -> yoEquation.parse("x = 0.5 ** (2.0 *a + 2*b)"));
      assertThrows(EquationParseError.class, () -> yoEquation.parse("x = 0.5 // (2.0 *a + 2*b)"));
      assertThrows(EquationParseError.class, () -> yoEquation.parse("x = 0.5.0 * (2.0 *a + 2*b)"));
      assertThrows(EquationParseError.class, () -> yoEquation.parse("x = 0.5 * (2.0 *a + 2*b"));
   }

   public static void main(String[] args)
   {
      StringTokenizer st = new StringTokenizer("x = 2.0*a+b");
      while (st.hasMoreTokens())
         System.out.println(st.nextToken());

   }
}

package us.ihmc.scs2.symbolic;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.symbolic.EquationBuilder.EquationBuilderException;
import us.ihmc.scs2.symbolic.EquationInput.DoubleInput;
import us.ihmc.scs2.symbolic.EquationInput.DoubleVariable;
import us.ihmc.scs2.symbolic.EquationInput.InputType;
import us.ihmc.scs2.symbolic.EquationInput.IntegerInput;
import us.ihmc.scs2.symbolic.parser.EquationParseError;
import us.ihmc.scs2.symbolic.parser.EquationParser;

import java.util.Random;
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

   @Test
   public void testComputeYawFromQuaternion()
   {
      String eqString = "yaw = atan2(2.0 * (qx * qy + qz * qs), 1.0 - 2.0 * (qy * qy + qz * qz))";
      EquationParser parser = new EquationParser();
      parser.getAliasManager().addAlias("yaw", InputType.DOUBLE);
      DoubleVariable qx = (DoubleVariable) parser.getAliasManager().addAlias("qx", InputType.DOUBLE).input();
      DoubleVariable qy = (DoubleVariable) parser.getAliasManager().addAlias("qy", InputType.DOUBLE).input();
      DoubleVariable qz = (DoubleVariable) parser.getAliasManager().addAlias("qz", InputType.DOUBLE).input();
      DoubleVariable qs = (DoubleVariable) parser.getAliasManager().addAlias("qs", InputType.DOUBLE).input();
      Equation equation = Equation.parse(eqString, parser);

      Random random = new Random(234);

      for (int i = 0; i < 1000; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         qx.setValue(0, quaternion.getX());
         qy.setValue(0, quaternion.getY());
         qz.setValue(0, quaternion.getZ());
         qs.setValue(0, quaternion.getS());
         equation.compute(0.0);
         double yaw = ((DoubleInput) equation.getResult()).getValue();
         assertEquals(quaternion.getYaw(), yaw, 1.0e-10, "Iteration: " + i);
      }
   }

   @Test
   public void testSum()
   {
      String eqString = "a = ( b + c ) / 20 + ( d + e ) * 0.32 + f";
      EquationParser parser = new EquationParser();
      parser.getAliasManager().addAlias("a", InputType.DOUBLE);
      DoubleVariable b = (DoubleVariable) parser.getAliasManager().addAlias("b", InputType.DOUBLE).input();
      DoubleVariable c = (DoubleVariable) parser.getAliasManager().addAlias("c", InputType.DOUBLE).input();
      DoubleVariable d = (DoubleVariable) parser.getAliasManager().addAlias("d", InputType.DOUBLE).input();
      DoubleVariable e = (DoubleVariable) parser.getAliasManager().addAlias("e", InputType.DOUBLE).input();
      DoubleVariable f = (DoubleVariable) parser.getAliasManager().addAlias("f", InputType.DOUBLE).input();
      Equation equation = Equation.parse(eqString, parser);

      Random random = new Random(234);

      for (int i = 0; i < 1000; i++)
      {
         b.setValue(0, random.nextDouble());
         c.setValue(0, random.nextDouble());
         d.setValue(0, random.nextDouble());
         e.setValue(0, random.nextDouble());
         f.setValue(0, random.nextDouble());

         equation.compute(0.0);
         double aActual = ((DoubleInput) equation.getResult()).getValue();
         double aExpected = (b.getValue() + c.getValue()) / 20.0 + (d.getValue() + e.getValue()) * 0.32 + f.getValue();
         assertEquals(aExpected, aActual, 1.0e-10, "Iteration: " + i);
      }
   }

   public static void main(String[] args)
   {
      StringTokenizer st = new StringTokenizer("x = 2.0*a+b");
      while (st.hasMoreTokens())
         System.out.println(st.nextToken());
   }
}

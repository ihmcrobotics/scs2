package us.ihmc.scs2.symbolic.parser;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.scs2.symbolic.EquationInput;
import us.ihmc.scs2.symbolic.EquationInput.DoubleVariable;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.AbsoluteOperation.AbsoluteDoubleOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.AddOperation.AddDoubleOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.*;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.DivideOperation.DivideDoubleOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.MaxOperation.MaxDoubleOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.MinOperation.MinDoubleOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.ModuloOperation.ModuloDoubleOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.MultiplyOperation.MultiplyDoubleOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.SignOperation.SignDoubleOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.SubtractOperation.SubtractDoubleOperation;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class EquationOperationLibraryTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;
   private static final double FD_EPSILON = 2.0e-3;

   @Test
   public void testAddDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      AddDoubleOperation operation = new AddDoubleOperation(List.of(A, B));

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = A.getValue() + B.getValue();
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         double bValue = Math.cos(0.9 * t + 0.2);
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = A.getValue() + B.getValue();
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() + B.getValueDot();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testSubtractDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      SubtractDoubleOperation operation = new SubtractDoubleOperation(A, B);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = A.getValue() - B.getValue();
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         double bValue = Math.cos(0.9 * t + 0.2);
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = A.getValue() - B.getValue();
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() - B.getValueDot();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testMultiplyDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      MultiplyDoubleOperation operation = new MultiplyDoubleOperation(List.of(A, B));

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = A.getValue() * B.getValue();
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         double bValue = Math.cos(0.9 * t + 0.2);
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = A.getValue() * B.getValue();
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() * B.getValue() + A.getValue() * B.getValueDot();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testDivideDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      DivideDoubleOperation operation = new DivideDoubleOperation(A, B);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = A.getValue() / B.getValue();
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         double bValue = Math.cos(0.9 * t + 0.2);
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = A.getValue() / B.getValue();
         double valueActual = operation.getValue();
         double valueDotExpected = (A.getValueDot() * B.getValue() - A.getValue() * B.getValueDot()) / (B.getValue() * B.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testPowerDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      PowerDoubleOperation operation = new PowerDoubleOperation(A, B);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = Math.pow(A.getValue(), B.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         double bValue = Math.cos(0.9 * t + 0.2);
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = Math.pow(A.getValue(), B.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected =
               Math.pow(A.getValue(), B.getValue()) * (B.getValueDot() * Math.log(A.getValue()) + B.getValue() * A.getValueDot() / A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testAbsoluteDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      AbsoluteDoubleOperation operation = new AbsoluteDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = Math.abs(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.abs(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() * Math.signum(A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testSineDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      SineDoubleOperation operation = new SineDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = Math.sin(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.sin(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() * Math.cos(A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testCosineDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      CosineDoubleOperation operation = new CosineDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = Math.cos(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.cos(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = -A.getValueDot() * Math.sin(A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testTangentDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      TangentDoubleOperation operation = new TangentDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = Math.tan(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(0.3 * t) + 1.0;
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.tan(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() / Math.cos(A.getValue()) / Math.cos(A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testArcSineDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      ArcSineDoubleOperation operation = new ArcSineDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = Math.asin(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.asin(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() / Math.sqrt(1.0 - A.getValue() * A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testArcCosineDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      ArcCosineDoubleOperation operation = new ArcCosineDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = Math.acos(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.acos(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = -A.getValueDot() / Math.sqrt(1.0 - A.getValue() * A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testArcTangentDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      ArcTangentDoubleOperation operation = new ArcTangentDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = Math.atan(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.atan(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() / (1.0 + A.getValue() * A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testArcTangent2DoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      ArcTangent2DoubleOperation operation = new ArcTangent2DoubleOperation(A, B);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = Math.atan2(A.getValue(), B.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(t + 0.5);
         double bValue = Math.sin(2.0 * t + 0.5);
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = Math.atan2(A.getValue(), B.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected =
               (A.getValueDot() * B.getValue() - A.getValue() * B.getValueDot()) / (A.getValue() * A.getValue() + B.getValue() * B.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testExponentialDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      ExponentialDoubleOperation operation = new ExponentialDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 100.0));
         operation.updateValue(0);
         double expected = Math.exp(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;
      double valueDotPrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.exp(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() * Math.exp(A.getValue());
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testNaturalLogarithmDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      NaturalLogarithmDoubleOperation operation = new NaturalLogarithmDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = Math.log(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;
      double valueDotPrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.log(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() / A.getValue();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testLogarithmBase10DoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      LogarithmBase10DoubleOperation operation = new LogarithmBase10DoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = Math.log10(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;
      double valueDotPrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.log10(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() / (A.getValue() * Math.log(10.0));
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testSquareRootDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      SquareRootDoubleOperation operation = new SquareRootDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = Math.sqrt(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.0001;
      double valuePrevious = Double.NaN;
      double valueDotPrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.sqrt(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot() / (2.0 * Math.sqrt(A.getValue()));
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testMaxDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      MaxDoubleOperation operation = new MaxDoubleOperation(A, B);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = Math.max(A.getValue(), B.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.001;
      double valuePrevious = Double.NaN;
      boolean previousIsA = false;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(2.0 * t + 0.5);
         double bValue = Math.sin(3.0 * t + 0.5);
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = Math.max(A.getValue(), B.getValue());
         double valueActual = operation.getValue();
         boolean currentIsA = A.getValue() > B.getValue();
         double valueDotExpected = currentIsA ? A.getValueDot() : B.getValueDot();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0 && previousIsA == currentIsA)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         previousIsA = currentIsA;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testMinDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      MinDoubleOperation operation = new MinDoubleOperation(A, B);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = Math.min(A.getValue(), B.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.001;
      double valuePrevious = Double.NaN;
      boolean previousIsA = false;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(2.0 * t + 0.5);
         double bValue = Math.sin(3.0 * t + 0.5);
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = Math.min(A.getValue(), B.getValue());
         double valueActual = operation.getValue();
         boolean currentIsA = A.getValue() < B.getValue();
         double valueDotExpected = currentIsA ? A.getValueDot() : B.getValueDot();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0 && previousIsA == currentIsA)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         previousIsA = currentIsA;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testSignDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      SignDoubleOperation operation = new SignDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = Math.signum(A.getValue());
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.001;
      double valuePrevious = Double.NaN;
      boolean previousIsPositive = false;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(2.0 * t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = Math.signum(A.getValue());
         double valueActual = operation.getValue();
         double valueDotExpected = 0.0;
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0 && previousIsPositive == (A.getValue() > 0.0))
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         previousIsPositive = A.getValue() > 0.0;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testModuloDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable B = EquationInput.newDoubleVariable();
      ModuloDoubleOperation operation = new ModuloDoubleOperation(A, B);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         B.setValue(0, EuclidCoreRandomTools.nextDouble(random, 1.0));
         operation.updateValue(0);
         double expected = A.getValue() % B.getValue();
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(2.0 * t + 0.5);
         double bValue = 5.0; // TODO Keeping this one constant as for now we do not consider it in the derivative.
         A.setValue(t, aValue);
         B.setValue(t, bValue);
         operation.updateValue(t);

         double valueExpected = A.getValue() % B.getValue();
         double valueActual = operation.getValue();
         double valueDotExpected = A.getValueDot();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         assertEquals(valueDotExpected, valueDotActual, FD_EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testDifferentiateDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DifferentiateDoubleOperation operation = new DifferentiateDoubleOperation(A);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random));
         operation.updateValue(0);
         double expected = A.getValueDot();
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON);
      }

      double dt = 0.001;
      double valuePrevious = Double.NaN;

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(2.0 * t + 0.5);
         A.setValue(t, aValue);
         operation.updateValue(t);

         double valueExpected = A.getValueDot();
         double valueActual = operation.getValue();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);

         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }

   @Test
   public void testLowPassFilterDoubleOperation()
   {
      Random random = new Random(34234);
      DoubleVariable A = EquationInput.newDoubleVariable();
      DoubleVariable alpha = EquationInput.newDoubleVariable();
      LowPassFilterDoubleOperation operation = new LowPassFilterDoubleOperation(A, alpha);

      for (int i = 0; i < ITERATIONS; i++)
      {
         A.setValue(0, EuclidCoreRandomTools.nextDouble(random));
         alpha.setValue(0, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         double expected = Double.isNaN(operation.getValue()) ?
               A.getValue() :
               EuclidCoreTools.interpolate(A.getValue(), operation.getValue(), alpha.getValue());
         operation.updateValue(0);
         double actual = operation.getValue();
         assertEquals(expected, actual, EPSILON, "Iteration: " + i);
      }

      double dt = 0.001;
      double valuePrevious = Double.NaN;
      operation.reset();

      for (double t = 0.0; t <= 1.0; t += dt)
      {
         double aValue = Math.sin(2.0 * t + 0.5);
         A.setValue(t, aValue);
         alpha.setValue(t, Math.cos(3.0 * t + 0.2));

         double valueExpected = Double.isNaN(operation.getValue()) ?
               A.getValue() :
               EuclidCoreTools.interpolate(A.getValue(), operation.getValue(), alpha.getValue());
         operation.updateValue(t);
         double valueActual = operation.getValue();
         double valueDotActual = operation.getValueDot();

         assertEquals(valueExpected, valueActual, EPSILON);
         if (t > 0.0)
            assertEquals((valueActual - valuePrevious) / dt, valueDotActual, FD_EPSILON);

         valuePrevious = valueActual;
         operation.updatePreviousValue();
      }
   }
}

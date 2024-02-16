package us.ihmc.scs2.symbolic.parser;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.scs2.symbolic.EquationInput;
import us.ihmc.scs2.symbolic.EquationInput.DoubleVariable;
import us.ihmc.scs2.symbolic.EquationInput.IntegerVariable;
import us.ihmc.scs2.symbolic.EquationInput.ScalarInput;
import us.ihmc.scs2.symbolic.parser.EquationOperation.DoubleEquationOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperation.IntegerEquationOperation;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class EquationOperationLibrary
{
   private static final List<String> operationNames;
   private static final Map<String, String> operationDescriptionMap;
   private static final Map<String, Function<List<? extends EquationInput>, EquationOperation<?>>> operationBuilderMap;

   static
   {
      List<Class<?>> operationClasses = new ArrayList<>(Arrays.asList(EquationOperationLibrary.class.getDeclaredClasses()));
      operationClasses.sort(Comparator.comparing(Class::getSimpleName));

      List<String> names = new ArrayList<>(operationClasses.size());
      Map<String, String> descriptionMap = new LinkedHashMap<>(operationClasses.size());
      Map<String, Function<List<? extends EquationInput>, EquationOperation<?>>> builderMap = new LinkedHashMap<>(operationClasses.size());

      try
      {
         for (Class<?> operationClass : operationClasses)
         {
            String name = (String) operationClass.getField("NAME").get(null);
            String description = (String) operationClass.getField("DESCRIPTION").get(null);
            Method newOperationMethod = operationClass.getMethod("newOperation", List.class);

            Function<List<? extends EquationInput>, EquationOperation<?>> builder = inputs ->
            {
               try
               {
                  return (EquationOperation<?>) newOperationMethod.invoke(null, inputs);
               }
               catch (Exception e)
               {
                  throw new RuntimeException(e);
               }
            };

            names.add(name);
            descriptionMap.put(name, description);
            builderMap.put(name, builder);
         }
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      operationNames = Collections.unmodifiableList(names);
      operationDescriptionMap = Collections.unmodifiableMap(descriptionMap);
      operationBuilderMap = Collections.unmodifiableMap(builderMap);
   }

   public static List<String> getOperationNames()
   {
      return operationNames;
   }

   public static String getOperationDescription(String operationName)
   {
      return operationDescriptionMap.get(operationName);
   }

   public static Function<List<? extends EquationInput>, EquationOperation<?>> getOperationBuilder(String operationName)
   {
      return operationBuilderMap.get(operationName);
   }

   public static EquationOperation<?> newOperation(String operationName, List<? extends EquationInput> inputs)
   {
      Function<List<? extends EquationInput>, EquationOperation<?>> builder = operationBuilderMap.get(operationName);
      if (builder == null)
         throw new IllegalArgumentException("Unknown operation: " + operationName);
      return builder.apply(inputs);
   }

   @SuppressWarnings("unchecked")
   private static List<ScalarInput> toScalarInputList(List<? extends EquationInput> inputs)
   {
      if (inputs == null || inputs.isEmpty())
         throw new IllegalArgumentException("Cannot create an operation with no inputs.");
      if (!areAllScalarInputs(inputs))
         throw new IllegalArgumentException("All inputs must be scalar.");
      return (List<ScalarInput>) inputs;
   }

   private static boolean areAllScalarInputs(List<? extends EquationInput> inputs)
   {
      for (EquationInput input : inputs)
      {
         if (!(input instanceof ScalarInput))
            return false;
      }
      return true;
   }

   private static boolean isAtLeastOneDoubleInput(List<? extends EquationInput> inputs)
   {
      for (EquationInput input : inputs)
      {
         if (input instanceof ScalarInput && ((ScalarInput) input).getType() == EquationInput.InputType.DOUBLE)
            return true;
      }
      return false;
   }

   static void checkNumberOfInputs(List<? extends EquationInput> inputs, int expected)
   {
      if (inputs.size() != expected)
         throw new IllegalOperationException("Function expects: " + expected + " inputs but got: " + inputs.size());
   }

   public static final class AddOperation
   {
      public static final String NAME = "add";
      public static final String DESCRIPTION = "Performs an addition.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         return isAtLeastOneDoubleInput(inputs) ? new AddDoubleOperation(scalarInputList) : new AddIntegerOperation(scalarInputList);
      }

      static final class AddDoubleOperation extends DoubleEquationOperation
      {
         public AddDoubleOperation(List<? extends ScalarInput> inputs)
         {
            super(NAME, DESCRIPTION, inputs);
         }

         @Override
         protected void computeValue(double time)
         {
            value = 0.0;
            for (int i = 0; i < getNumberOfInputs(); i++)
               value += getInput(i).getValueAsDouble();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = 0.0;
            for (int i = 0; i < getNumberOfInputs(); i++)
               derivative += getInput(i).getValueDot();
         }

         @Override
         public String toString()
         {
            return "(%s + %s)".formatted(getInput(0), getInput(1));
         }
      }

      static final class AddIntegerOperation extends IntegerEquationOperation
      {
         public AddIntegerOperation(List<? extends ScalarInput> inputs)
         {
            super(NAME, DESCRIPTION, inputs);
         }

         @Override
         protected void computeValue(double time)
         {
            value = 0;
            for (int i = 0; i < getNumberOfInputs(); i++)
               value += getInput(i).getValueAsInteger();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = 0;
            for (int i = 0; i < getNumberOfInputs(); i++)
               derivative += getInput(i).getValueDot();
         }

         @Override
         public String toString()
         {
            return "(%s + %s)".formatted(getInput(0), getInput(1));
         }
      }
   }

   public static final class SubtractOperation
   {

      public static final String NAME = "subtract";
      public static final String DESCRIPTION = "Performs a subtraction.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput B = scalarInputList.get(1);
         return isAtLeastOneDoubleInput(inputs) ? new SubtractDoubleOperation(A, B) : new SubtractIntegerOperation(A, B);
      }

      static final class SubtractDoubleOperation extends DoubleEquationOperation
      {
         private final ScalarInput A, B;

         public SubtractDoubleOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = A.getValueAsDouble() - B.getValueAsDouble();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueDot() - B.getValueDot();
         }

         @Override
         public String toString()
         {
            return "(%s - %s)".formatted(getInput(0), getInput(1));
         }
      }

      static final class SubtractIntegerOperation extends IntegerEquationOperation
      {
         private final ScalarInput A, B;

         public SubtractIntegerOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = A.getValueAsInteger() - B.getValueAsInteger();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueDot() - B.getValueDot();
         }

         @Override
         public String toString()
         {
            return "(%s - %s)".formatted(getInput(0), getInput(1));
         }
      }
   }

   public static final class MultiplyOperation
   {
      public static final String NAME = "multiply";
      public static final String DESCRIPTION = "Performs a multiplication.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         return isAtLeastOneDoubleInput(inputs) ? new MultiplyDoubleOperation(scalarInputList) : new MultiplyIntegerOperation(scalarInputList);
      }

      static final class MultiplyDoubleOperation extends DoubleEquationOperation
      {
         public MultiplyDoubleOperation(List<? extends ScalarInput> inputs)
         {
            super(NAME, DESCRIPTION, inputs);
         }

         @Override
         protected void computeValue(double time)
         {
            value = 1.0;
            for (int i = 0; i < getNumberOfInputs(); i++)
               value *= getInput(i).getValueAsDouble();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = 0.0;
            for (int i = 0; i < getNumberOfInputs(); i++)
            {
               double product = getInput(i).getValueDot();
               for (int j = 0; j < getNumberOfInputs(); j++)
               {
                  if (i == j)
                     continue;
                  product *= getInput(j).getValueAsDouble();
               }
               derivative += product;
            }
         }

         @Override
         public String toString()
         {
            return "(%s * %s)".formatted(getInput(0), getInput(1));
         }
      }

      static final class MultiplyIntegerOperation extends IntegerEquationOperation
      {
         public MultiplyIntegerOperation(List<? extends ScalarInput> inputs)
         {
            super(NAME, DESCRIPTION, inputs);
         }

         @Override
         protected void computeValue(double time)
         {
            value = 1;
            for (int i = 0; i < getNumberOfInputs(); i++)
               value *= getInput(i).getValueAsInteger();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = 0;
            for (int i = 0; i < getNumberOfInputs(); i++)
            {
               double product = getInput(i).getValueDot();
               for (int j = 0; j < getNumberOfInputs(); j++)
               {
                  if (i == j)
                     continue;
                  product *= getInput(j).getValueAsInteger();
               }
               derivative += product;
            }
         }

         @Override
         public String toString()
         {
            return "(%s * %s)".formatted(getInput(0), getInput(1));
         }
      }
   }

   public static final class DivideOperation
   {
      public static final String NAME = "divide";
      public static final String DESCRIPTION = "Performs a division.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput B = scalarInputList.get(1);
         return isAtLeastOneDoubleInput(inputs) ? new DivideDoubleOperation(A, B) : new DivideIntegerOperation(A, B);
      }

      static final class DivideDoubleOperation extends DoubleEquationOperation
      {
         private final ScalarInput A, B;

         public DivideDoubleOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = A.getValueAsDouble() / B.getValueAsDouble();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = (A.getValueDot() * B.getValueAsDouble() - A.getValueAsDouble() * B.getValueDot()) / (B.getValueAsDouble() * B.getValueAsDouble());
         }

         @Override
         public String toString()
         {
            return "(%s / %s)".formatted(getInput(0), getInput(1));
         }
      }

      static final class DivideIntegerOperation extends IntegerEquationOperation
      {
         private final ScalarInput A, B;

         public DivideIntegerOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = A.getValueAsInteger() / B.getValueAsInteger();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = (A.getValueDot() * B.getValueAsInteger() - A.getValueAsInteger() * B.getValueDot()) / (B.getValueAsInteger() * B.getValueAsInteger());
         }

         @Override
         public String toString()
         {
            return "(%s / %s)".formatted(getInput(0), getInput(1));
         }
      }
   }

   public static class PowerDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "power";
      public static final String DESCRIPTION = "Computes  the value of the first value raised to the power of the second value.";

      public static PowerDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput B = scalarInputList.get(1);
         return new PowerDoubleOperation(A, B);
      }

      private final ScalarInput A, B;

      public PowerDoubleOperation(ScalarInput A, ScalarInput B)
      {
         super(NAME, DESCRIPTION, List.of(A, B));
         this.A = A;
         this.B = B;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.pow(A.getValueAsDouble(), B.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = Math.pow(A.getValueAsDouble(), B.getValueAsDouble()) * (B.getValueAsDouble() * A.getValueDot() / A.getValueAsDouble()
                                                                              + Math.log(A.getValueAsDouble()) * B.getValueDot());
      }

      @Override
      public String toString()
      {
         return "(%s ^ %s)".formatted(getInput(0), getInput(1));
      }
   }

   public static class AbsoluteOperation
   {
      public static final String NAME = "abs";
      public static final String DESCRIPTION = "Computes the absolute value of a value.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return isAtLeastOneDoubleInput(inputs) ? new AbsoluteDoubleOperation(A) : new AbsoluteIntegerOperation(A);
      }

      static class AbsoluteDoubleOperation extends DoubleEquationOperation
      {
         private final ScalarInput A;

         public AbsoluteDoubleOperation(ScalarInput A)
         {
            super(NAME, DESCRIPTION, List.of(A));
            this.A = A;
         }

         @Override
         protected void computeValue(double time)
         {
            value = Math.abs(A.getValueAsDouble());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueDot() * Math.signum(A.getValueAsDouble());
         }

         @Override
         public String toString()
         {
            return "|%s|".formatted(getInput(0));
         }
      }

      static class AbsoluteIntegerOperation extends IntegerEquationOperation
      {
         private final ScalarInput A;

         public AbsoluteIntegerOperation(ScalarInput A)
         {
            super(NAME, DESCRIPTION, List.of(A));
            this.A = A;
         }

         @Override
         protected void computeValue(double time)
         {
            value = Math.abs(A.getValueAsInteger());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueDot() * Math.signum(A.getValueAsInteger());
         }

         @Override
         public String toString()
         {
            return "|%s|".formatted(getInput(0));
         }
      }
   }

   public static class SineDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "sin";
      public static final String DESCRIPTION = "Computes the trigonometric sine of an angle (rad).";

      public static SineDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new SineDoubleOperation(A);
      }

      private final ScalarInput A;

      public SineDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.sin(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = A.getValueDot() * Math.cos(A.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "sin(%s)".formatted(getInput(0));
      }
   }

   public static class CosineDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "cos";
      public static final String DESCRIPTION = "Computes the trigonometric cosine of an angle (rad).";

      public static CosineDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new CosineDoubleOperation(A);
      }

      private final ScalarInput A;

      public CosineDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.cos(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = -A.getValueDot() * Math.sin(A.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "cos(%s)".formatted(getInput(0));
      }
   }

   public static class TangentDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "tan";
      public static final String DESCRIPTION = "Computes the trigonometric tangent of an angle (rad).";

      public static TangentDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new TangentDoubleOperation(A);
      }

      private final ScalarInput A;

      public TangentDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.tan(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = A.getValueDot() / Math.cos(A.getValueAsDouble()) / Math.cos(A.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "tan(%s)".formatted(getInput(0));
      }
   }

   public static class ArcSineDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "asin";
      public static final String DESCRIPTION = "Compute the arc sine of a value; the returned angle is in the range -pi/2 through pi/2.";

      public static ArcSineDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new ArcSineDoubleOperation(A);
      }

      private final ScalarInput A;

      public ArcSineDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.asin(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = A.getValueDot() / Math.sqrt(1.0 - A.getValueAsDouble() * A.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "asin(%s)".formatted(getInput(0));
      }
   }

   public static class ArcCosineDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "acos";
      public static final String DESCRIPTION = "Compute the arc cosine of a value; the returned angle is in the range 0.0 through pi.";

      public static ArcCosineDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new ArcCosineDoubleOperation(A);
      }

      private final ScalarInput A;

      public ArcCosineDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.acos(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = -A.getValueDot() / Math.sqrt(1.0 - A.getValueAsDouble() * A.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "acos(%s)".formatted(getInput(0));
      }
   }

   public static class ArcTangentDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "atan";
      public static final String DESCRIPTION = "Compute the arc tangent of a value; the returned angle is in the range -pi/2 through pi/2.";

      public static ArcTangentDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new ArcTangentDoubleOperation(A);
      }

      private final ScalarInput A;

      public ArcTangentDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.atan(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = A.getValueDot() / (1.0 + A.getValueAsDouble() * A.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "atan(%s)".formatted(getInput(0));
      }
   }

   public static class ArcTangent2DoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "atan2";
      public static final String DESCRIPTION = "Computes the angle theta from the conversion of rectangular coordinates (x, y) to polar coordinates (r, theta). This method computes the phase theta by computing an arc tangent of y/x in the range of -pi to pi.";

      public static ArcTangent2DoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput B = scalarInputList.get(1);
         return new ArcTangent2DoubleOperation(A, B);
      }

      private final ScalarInput A, B;

      public ArcTangent2DoubleOperation(ScalarInput A, ScalarInput B)
      {
         super(NAME, DESCRIPTION, List.of(A, B));
         this.A = A;
         this.B = B;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.atan2(A.getValueAsDouble(), B.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = (A.getValueDot() * B.getValueAsDouble() - A.getValueAsDouble() * B.getValueDot()) / (A.getValueAsDouble() * A.getValueAsDouble()
                                                                                                           + B.getValueAsDouble() * B.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "atan2(%s, %s)".formatted(getInput(0), getInput(1));
      }
   }

   public static class ExponentialDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "exp";
      public static final String DESCRIPTION = "Computes the base-e exponential function of a value.";

      public static ExponentialDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new ExponentialDoubleOperation(A);
      }

      private final ScalarInput A;

      public ExponentialDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.exp(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = A.getValueDot() * Math.exp(A.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "exp(%s)".formatted(getInput(0));
      }
   }

   public static class NaturalLogarithmDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "log";
      public static final String DESCRIPTION = "Computes the natural logarithm (base e) of a value.";

      public static NaturalLogarithmDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new NaturalLogarithmDoubleOperation(A);
      }

      private final ScalarInput A;

      public NaturalLogarithmDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.log(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = A.getValueDot() / A.getValueAsDouble();
      }

      @Override
      public String toString()
      {
         return "log(%s)".formatted(getInput(0));
      }
   }

   public static class LogarithmBase10DoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "log10";
      public static final String DESCRIPTION = "Computes the base 10 logarithm of a value.";

      public static LogarithmBase10DoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new LogarithmBase10DoubleOperation(A);
      }

      private final ScalarInput A;

      public LogarithmBase10DoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.log10(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = A.getValueDot() / A.getValueAsDouble() / Math.log(10.0);
      }

      @Override
      public String toString()
      {
         return "log10(%s)".formatted(getInput(0));
      }
   }

   public static class SquareRootDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "sqrt";
      public static final String DESCRIPTION = "Computes the square root of a value.";

      public static SquareRootDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new SquareRootDoubleOperation(A);
      }

      private final ScalarInput A;

      public SquareRootDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = Math.sqrt(A.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         derivative = A.getValueDot() / 2.0 / Math.sqrt(A.getValueAsDouble());
      }

      @Override
      public String toString()
      {
         return "sqrt(%s)".formatted(getInput(0));
      }
   }

   public static class MaxOperation
   {
      public static final String NAME = "max";
      public static final String DESCRIPTION = "Computes the maximum of two values.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput B = scalarInputList.get(1);
         return isAtLeastOneDoubleInput(inputs) ? new MaxDoubleOperation(A, B) : new MaxIntegerOperation(A, B);
      }

      static class MaxDoubleOperation extends DoubleEquationOperation
      {
         private final ScalarInput A, B;

         public MaxDoubleOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = Math.max(A.getValueAsDouble(), B.getValueAsDouble());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueAsDouble() > B.getValueAsDouble() ? A.getValueDot() : B.getValueDot();
         }

         @Override
         public String toString()
         {
            return "max(%s, %s)".formatted(getInput(0), getInput(1));
         }
      }

      static class MaxIntegerOperation extends IntegerEquationOperation
      {
         private final ScalarInput A, B;

         public MaxIntegerOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = Math.max(A.getValueAsInteger(), B.getValueAsInteger());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueAsInteger() > B.getValueAsInteger() ? A.getValueDot() : B.getValueDot();
         }

         @Override
         public String toString()
         {
            return "max(%s, %s)".formatted(getInput(0), getInput(1));
         }
      }
   }

   public static class MinOperation
   {
      public static final String NAME = "min";
      public static final String DESCRIPTION = "Computes the minimum of two values.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput B = scalarInputList.get(1);
         return isAtLeastOneDoubleInput(inputs) ? new MinDoubleOperation(A, B) : new MinIntegerOperation(A, B);
      }

      static class MinDoubleOperation extends DoubleEquationOperation
      {
         private final ScalarInput A, B;

         public MinDoubleOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = Math.min(A.getValueAsDouble(), B.getValueAsDouble());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueAsDouble() < B.getValueAsDouble() ? A.getValueDot() : B.getValueDot();
         }

         @Override
         public String toString()
         {
            return "min(%s, %s)".formatted(getInput(0), getInput(1));
         }
      }

      static class MinIntegerOperation extends IntegerEquationOperation
      {
         private final ScalarInput A, B;

         public MinIntegerOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = Math.min(A.getValueAsInteger(), B.getValueAsInteger());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueAsInteger() < B.getValueAsInteger() ? A.getValueDot() : B.getValueDot();
         }

         @Override
         public String toString()
         {
            return "min(%s, %s)".formatted(getInput(0), getInput(1));
         }
      }
   }

   public static class SignOperation
   {
      public static final String NAME = "sign";
      public static final String DESCRIPTION = "Computes the sign of a value.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return isAtLeastOneDoubleInput(inputs) ? new SignDoubleOperation(A) : new SignIntegerOperation(A);
      }

      static class SignDoubleOperation extends DoubleEquationOperation
      {
         private final ScalarInput A;

         public SignDoubleOperation(ScalarInput A)
         {
            super(NAME, DESCRIPTION, List.of(A));
            this.A = A;
         }

         @Override
         protected void computeValue(double time)
         {
            value = Math.signum(A.getValueAsDouble());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = 0.0;
         }

         @Override
         public String toString()
         {
            return "sign(%s)".formatted(getInput(0));
         }
      }

      static class SignIntegerOperation extends IntegerEquationOperation
      {
         private final ScalarInput A;

         public SignIntegerOperation(ScalarInput A)
         {
            super(NAME, DESCRIPTION, List.of(A));
            this.A = A;
         }

         @Override
         protected void computeValue(double time)
         {
            value = (int) Math.signum(A.getValueAsInteger());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = 0;
         }

         @Override
         public String toString()
         {
            return "sign(%s)".formatted(getInput(0));
         }
      }
   }

   public static class ModuloOperation
   {
      public static final String NAME = "mod";
      public static final String DESCRIPTION = "Computes the modulo of the two values.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput B = scalarInputList.get(1);
         return isAtLeastOneDoubleInput(inputs) ? new ModuloDoubleOperation(A, B) : new ModuloIntegerOperation(A, B);
      }

      static class ModuloDoubleOperation extends DoubleEquationOperation
      {
         private final ScalarInput A, B;

         public ModuloDoubleOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = A.getValueAsDouble() % B.getValueAsDouble();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueDot() % B.getValueAsDouble();
         }

         @Override
         public String toString()
         {
            return "(%s %% %s)".formatted(getInput(0), getInput(1));
         }
      }

      static class ModuloIntegerOperation extends IntegerEquationOperation
      {
         private final ScalarInput A, B;

         public ModuloIntegerOperation(ScalarInput A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            value = A.getValueAsInteger() % B.getValueAsInteger();
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = A.getValueDot() % B.getValueAsInteger();
         }

         @Override
         public String toString()
         {
            return "(%s %% %s)".formatted(getInput(0), getInput(1));
         }
      }
   }

   public static class DifferentiateDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "diff";
      public static final String DESCRIPTION = "Differentiates the input with respect to time.";

      public static DifferentiateDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 1);
         ScalarInput A = scalarInputList.get(0);
         return new DifferentiateDoubleOperation(A);
      }

      private final ScalarInput A;

      public DifferentiateDoubleOperation(ScalarInput A)
      {
         super(NAME, DESCRIPTION, List.of(A));
         this.A = A;
      }

      @Override
      protected void computeValue(double time)
      {
         value = A.getValueDot();
      }

      @Override
      protected void computeDerivative(double time)
      { // TODO Fix the derivative
         if (Double.isNaN(previousTime))
            derivative = 0.0;
         else
            derivative = (value - previousValue) / (time - previousTime);
      }

      @Override
      public String toString()
      {
         return "diff(%s)".formatted(getInput(0));
      }
   }

   public static class LowPassFilterDoubleOperation extends DoubleEquationOperation
   {
      public static final String NAME = "lpf";
      public static final String DESCRIPTION = "Low pass filter. First parameter is the input, second parameter is the filter gain [0, 1].";

      public static LowPassFilterDoubleOperation newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput alpha = scalarInputList.get(1);
         return new LowPassFilterDoubleOperation(A, alpha);
      }

      private final ScalarInput A;
      private final ScalarInput alpha;

      private double previousValue = Double.NaN;

      public LowPassFilterDoubleOperation(ScalarInput A, ScalarInput alpha)
      {
         super(NAME, DESCRIPTION, List.of(A, alpha));
         this.A = A;
         this.alpha = alpha;
      }

      @Override
      protected void computeValue(double time)
      {
         previousValue = value;
         if (Double.isNaN(previousValue))
            value = A.getValueAsDouble();
         else
            value = EuclidCoreTools.interpolate(A.getValueAsDouble(), previousValue, alpha.getValueAsDouble());
      }

      @Override
      protected void computeDerivative(double time)
      {
         if (Double.isNaN(previousValue))
            derivative = 0.0;
         else
            derivative = (value - previousValue) / (time - previousTime);
      }

      @Override
      public String toString()
      {
         return "lpf(%s, %s)".formatted(getInput(0), getInput(1));
      }
   }

   public static class AssignmentOperation
   {
      public static final String NAME = "assign";
      public static final String DESCRIPTION = "Assigns the value of the right hand side to the left hand side.";

      public static EquationOperation<?> newOperation(List<? extends EquationInput> inputs)
      {
         List<ScalarInput> scalarInputList = toScalarInputList(inputs);
         checkNumberOfInputs(inputs, 2);
         ScalarInput A = scalarInputList.get(0);
         ScalarInput B = scalarInputList.get(1);
         return A instanceof DoubleVariable ? new AssignmentDoubleOperation((DoubleVariable) A, B) : new AssignmentIntegerOperation((IntegerVariable) A, B);
      }

      private static class AssignmentDoubleOperation extends DoubleEquationOperation
      {
         private final DoubleVariable A;
         private final ScalarInput B;

         public AssignmentDoubleOperation(DoubleVariable A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            this.value = B.getValueAsDouble();
            A.setValue(time, B.getValueAsDouble());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = B.getValueDot();
            // TODO See if we can also set the derivative of A, although I don't see where it'd be used for now.
         }

         @Override
         public String toString()
         {
            return "%s = %s".formatted(getInput(0), getInput(1));
         }
      }

      private static class AssignmentIntegerOperation extends IntegerEquationOperation
      {
         private final IntegerVariable A;
         private final ScalarInput B;

         public AssignmentIntegerOperation(IntegerVariable A, ScalarInput B)
         {
            super(NAME, DESCRIPTION, List.of(A, B));
            this.A = A;
            this.B = B;
         }

         @Override
         protected void computeValue(double time)
         {
            this.value = B.getValueAsInteger();
            A.setValue(time, B.getValueAsInteger());
         }

         @Override
         protected void computeDerivative(double time)
         {
            derivative = B.getValueDot();
            // TODO See if we can also set the derivative of A, although I don't see where it'd be used for now.
         }

         @Override
         public String toString()
         {
            return "%s = %s".formatted(getInput(0), getInput(1));
         }
      }
   }
}

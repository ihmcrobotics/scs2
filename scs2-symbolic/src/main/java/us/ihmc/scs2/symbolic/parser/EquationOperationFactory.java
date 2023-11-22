package us.ihmc.scs2.symbolic.parser;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.scs2.symbolic.EquationInput;
import us.ihmc.scs2.symbolic.EquationInput.*;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

import java.util.List;
import java.util.Objects;
import java.util.function.*;

/**
 * Base class for creating new instances of functions/operators
 */
public abstract class EquationOperationFactory
{
   protected final String name;
   protected final String description;

   private Supplier<List<EquationInput>> inputsSupplier;
   private EquationOperation<?> operation;

   public EquationOperationFactory(String name, String description)
   {
      this.name = name;
      this.description = description;
   }

   public void setInputs(Supplier<List<EquationInput>> inputsSupplier)
   {
      this.inputsSupplier = inputsSupplier;
   }

   public abstract EquationOperationFactory duplicate();

   public EquationOperation<?> build()
   {
      operation = buildImpl(Objects.requireNonNull(inputsSupplier.get(), "Inputs supplier returned null"));
      return operation;
   }

   public EquationOperation<?> getOperation()
   {
      if (operation == null)
         throw new IllegalStateException("Operation has not been built yet.");
      return operation;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return description;
   }

   /**
    * Create a new instance of a function.
    *
    * @param inputs the inputs to the function
    * @return the resulting operation
    */
   protected abstract EquationOperation<?> buildImpl(List<EquationInput> inputs);

   static void checkNumberOfInputs(List<EquationInput> inputs, int expected)
   {
      if (inputs.size() != expected)
         throw new IllegalOperationException("Function expects: " + expected + " inputs but got: " + inputs.size());
   }

   public static class UnaryOperationFactory extends EquationOperationFactory
   {
      private final IntUnaryOperator integerValueOperator;
      private final DoubleUnaryOperator doubleValueOperator;
      private final ToDoubleFunction<ScalarInput> derivativeOperator;

      public UnaryOperationFactory(String name,
                                   String description,
                                   IntUnaryOperator integerValueOperator,
                                   DoubleUnaryOperator doubleValueOperator,
                                   ToDoubleFunction<ScalarInput> derivativeOperator)
      {
         super(name, description);
         this.doubleValueOperator = doubleValueOperator;
         this.integerValueOperator = integerValueOperator;
         this.derivativeOperator = derivativeOperator;
      }

      @Override
      public EquationOperationFactory duplicate()
      {
         return new UnaryOperationFactory(name, description, integerValueOperator, doubleValueOperator, derivativeOperator);
      }

      @Override
      protected EquationOperation<?> buildImpl(List<EquationInput> inputs)
      {
         EquationOperationFactory.checkNumberOfInputs(inputs, 1);
         return create(inputs.get(0));
      }

      public EquationOperation<?> create(EquationInput A)
      {
         if (integerValueOperator != null)
         {
            if (A instanceof IntegerInput intA)
            {
               return new EquationOperation<>(name + "-i",
                                              description,
                                              new SimpleIntegerVariable(),
                                              List.of(A),
                                              (time, result) -> result.setValue(time, integerValueOperator.applyAsInt(intA.getValue())),
                                              (time, value, resultToPack) -> resultToPack.setValue(time, derivativeOperator.applyAsDouble(intA)));
            }
         }

         if (doubleValueOperator != null)
         {
            if (A instanceof ScalarInput scalarA)
            {
               return new EquationOperation<>(name + "-s",
                                              description,
                                              new SimpleDoubleVariable(),
                                              List.of(A),
                                              (time, result) -> result.setValue(time, doubleValueOperator.applyAsDouble(scalarA.getValueAsDouble())),
                                              (time, value, resultToPack) -> resultToPack.setValue(time, derivativeOperator.applyAsDouble(scalarA)));
            }
         }

         throw new UnsupportedOperationException("Unexpected types: A = " + A.getClass().getSimpleName());
      }
   }

   public static class BinaryOperationFactory extends EquationOperationFactory
   {
      private final IntBinaryOperator integerOperator;
      private final DoubleBinaryOperator doubleOperator;
      private final ToDoubleBiFunction<ScalarInput, ScalarInput> derivativeOperator;

      public BinaryOperationFactory(String name,
                                    String description,
                                    IntBinaryOperator integerOperator,
                                    DoubleBinaryOperator doubleOperator,
                                    ToDoubleBiFunction<ScalarInput, ScalarInput> derivativeOperator)
      {
         super(name, description);
         this.doubleOperator = doubleOperator;
         this.integerOperator = integerOperator;
         this.derivativeOperator = derivativeOperator;
      }

      @Override
      public EquationOperationFactory duplicate()
      {
         return new BinaryOperationFactory(name, description, integerOperator, doubleOperator, derivativeOperator);
      }

      @Override
      protected EquationOperation<?> buildImpl(List<EquationInput> inputs)
      {
         EquationOperationFactory.checkNumberOfInputs(inputs, 2);
         return create(inputs.get(0), inputs.get(1));
      }

      public EquationOperation<?> create(EquationInput A, EquationInput B)
      {
         if (integerOperator != null)
         {
            if (A instanceof IntegerInput intA && B instanceof IntegerInput intB)
            {
               return new EquationOperation<>(name + "-ii",
                                              description,
                                              new SimpleIntegerVariable(),
                                              List.of(A, B),
                                              (time, value) -> value.setValue(time, integerOperator.applyAsInt(intA.getValue(), intB.getValue())),
                                              (time, value, derivative) -> derivative.setValue(time, derivativeOperator.applyAsDouble(intA, intB)));
            }
         }

         if (doubleOperator != null)
         {
            if (A instanceof ScalarInput scalarA && B instanceof ScalarInput scalarB)
            {
               return new EquationOperation<>(name + "-ss",
                                              description,
                                              new SimpleDoubleVariable(),
                                              List.of(A, B),
                                              (time, result) -> result.setValue(time,
                                                                                doubleOperator.applyAsDouble(scalarA.getValueAsDouble(),
                                                                                                             scalarB.getValueAsDouble())),
                                              ((time, value, resultToPack) -> resultToPack.setValue(time, derivativeOperator.applyAsDouble(scalarA, scalarB))));
            }
         }

         throw new UnsupportedOperationException("Unexpected types: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
      }
   }

   public static class AssignmentOperationFactory extends EquationOperationFactory
   {
      public AssignmentOperationFactory()
      {
         super("assign", "Assigns the value of the right hand side to the left hand side.");
      }

      @Override
      public EquationOperationFactory duplicate()
      {
         return new AssignmentOperationFactory();
      }

      @Override
      protected EquationOperation<?> buildImpl(List<EquationInput> inputs)
      {
         EquationOperationFactory.checkNumberOfInputs(inputs, 2);
         EquationInput A = inputs.get(0);
         EquationInput B = inputs.get(1);

         if (A instanceof IntegerVariable intA && B instanceof IntSupplier intB)
         {
            return new EquationOperation<>(name + "-ii", description, intA, List.of(B), (time, result) -> intA.setValue(time, intB.getAsInt()), null); // FIXME
         }

         if (A instanceof DoubleVariable doubleA && B instanceof ScalarInput scalarB)
         {
            return new EquationOperation<>(name + "-ds",
                                           description,
                                           doubleA,
                                           List.of(B),
                                           (time, resultToPack) -> doubleA.setValue(time, scalarB.getValueAsDouble()),
                                           null); // No derivative
         }

         throw new RuntimeException("Unsupported types for assignment: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
      }
   }

   public static class DifferentiateOperationFactory extends EquationOperationFactory
   {
      public DifferentiateOperationFactory()
      {
         super("diff", "Differentiates the input with respect to time.");
      }

      @Override
      public EquationOperationFactory duplicate()
      {
         return new DifferentiateOperationFactory();
      }

      @Override
      protected EquationOperation<?> buildImpl(List<EquationInput> inputs)
      {
         EquationOperationFactory.checkNumberOfInputs(inputs, 1);
         EquationInput A = inputs.get(0);

         if (A instanceof ScalarInput scalarA)
         {
            return new EquationOperation<>(name + "-s", description, new SimpleDoubleVariable(), List.of(A), (time, result) ->
            {
               result.setValue(time, scalarA.getValueDot());
            }, null); // No derivative
         }

         throw new RuntimeException("Unsupported types for assignment: A = " + A.getClass().getSimpleName());
      }
   }

   public static class LowPassFilterOperationFactory extends EquationOperationFactory
   {
      public LowPassFilterOperationFactory()
      {
         super("lpf", "Low pass filter. First parameter is the input, second parameter is the filter gain [0, 1].");
      }

      @Override
      public EquationOperationFactory duplicate()
      {
         return new LowPassFilterOperationFactory();
      }

      @Override
      protected EquationOperation<?> buildImpl(List<EquationInput> inputs)
      {
         EquationOperationFactory.checkNumberOfInputs(inputs, 2);
         EquationInput A = inputs.get(0);
         EquationInput B = inputs.get(1);

         if (A instanceof ScalarInput scalarA && B instanceof ScalarInput scalarB)
         {
            return new EquationOperation<>(name + "-ss", description, new SimpleDoubleVariable(), List.of(A, B), (time, result) ->
            {
               if (Double.isNaN(result.getValueAsDouble()))
                  result.setValue(time, scalarA.getValueAsDouble());
               else
                  result.setValue(time, EuclidCoreTools.interpolate(scalarA.getValueAsDouble(), result.getValueAsDouble(), scalarB.getValueAsDouble()));
            }, null); // No derivative
         }

         throw new RuntimeException("Unsupported types for assignment: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
      }
   }
}

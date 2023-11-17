package us.ihmc.scs2.symbolic.parser;

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
      private final IntUnaryOperator integerOperator;
      private final DoubleUnaryOperator doubleOperator;

      public UnaryOperationFactory(String name, String description, IntUnaryOperator integerOperator, DoubleUnaryOperator doubleOperator)
      {
         super(name, description);
         this.doubleOperator = doubleOperator;
         this.integerOperator = integerOperator;
      }

      @Override
      public EquationOperationFactory duplicate()
      {
         return new UnaryOperationFactory(name, description, integerOperator, doubleOperator);
      }

      @Override
      protected EquationOperation<?> buildImpl(List<EquationInput> inputs)
      {
         EquationOperationFactory.checkNumberOfInputs(inputs, 1);
         return create(inputs.get(0));
      }

      public EquationOperation<?> create(EquationInput A)
      {
         if (integerOperator != null)
         {
            if (A instanceof IntSupplier intA)
            {
               return new EquationOperation<>(name + "-i",
                                              description,
                                              new SimpleIntegerVariable(0),
                                              result -> result.setValue(integerOperator.applyAsInt(intA.getAsInt())));
            }
         }

         if (doubleOperator != null)
         {
            if (A instanceof ScalarConstant scalarA)
            {
               DoubleSupplier asDoubleA = scalarA.toDoubleSupplier();
               return new EquationOperation<>(name + "-s",
                                              description,
                                              new SimpleDoubleVariable(0),
                                              result -> result.setValue(doubleOperator.applyAsDouble(asDoubleA.getAsDouble())));
            }
         }

         throw new UnsupportedOperationException("Unexpected types: A = " + A.getClass().getSimpleName());
      }
   }

   public static class BinaryOperationFactory extends EquationOperationFactory
   {
      private final IntBinaryOperator integerOperator;
      private final DoubleBinaryOperator doubleOperator;

      public BinaryOperationFactory(String name, String description, IntBinaryOperator integerOperator, DoubleBinaryOperator doubleOperator)
      {
         super(name, description);
         this.doubleOperator = doubleOperator;
         this.integerOperator = integerOperator;
      }

      @Override
      public EquationOperationFactory duplicate()
      {
         return new BinaryOperationFactory(name, description, integerOperator, doubleOperator);
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
            if (A instanceof IntSupplier intA && B instanceof IntSupplier intB)
            {
               return new EquationOperation<>(name + "-ii",
                                              description,
                                              new SimpleIntegerVariable(0),
                                              result -> result.setValue(integerOperator.applyAsInt(intA.getAsInt(), intB.getAsInt())));
            }
         }

         if (doubleOperator != null)
         {
            if (A instanceof ScalarConstant scalarA && B instanceof ScalarConstant scalarB)
            {
               DoubleSupplier asDoubleA = scalarA.toDoubleSupplier();
               DoubleSupplier asDoubleB = scalarB.toDoubleSupplier();
               return new EquationOperation<>(name + "-ss",
                                              description,
                                              new SimpleDoubleVariable(0),
                                              result -> result.setValue(doubleOperator.applyAsDouble(asDoubleA.getAsDouble(), asDoubleB.getAsDouble())));
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
            return new EquationOperation<>(name + "-ii", description, intA, result -> result.setValue(intB.getAsInt()));
         }

         if (A instanceof DoubleVariable doubleA && B instanceof ScalarConstant scalarB)
         {
            DoubleSupplier asDoubleB = scalarB.toDoubleSupplier();
            return new EquationOperation<>(name + "-ds", description, doubleA, result -> result.setValue(asDoubleB.getAsDouble()));
         }

         throw new RuntimeException("Unsupported types for assignment: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
      }
   }
}

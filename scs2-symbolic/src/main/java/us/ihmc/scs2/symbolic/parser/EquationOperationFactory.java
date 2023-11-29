package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.EquationInput;

import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Class for creating new instances of functions/operators
 */
public final class EquationOperationFactory
{
   /**
    * Name of the operation.
    */
   protected final String name;
   /**
    * Description of the operation.
    */
   protected final String description;
   /**
    * Function that builds the operation.
    */
   private final Function<List<? extends EquationInput>, EquationOperation<?>> operationBuilder;
   /**
    * Supplier of the inputs for the operation, allows to delay the creation of the inputs until the operation is built.
    */
   private Supplier<List<EquationInput>> inputsSupplier;
   /**
    * The operation.
    */
   private EquationOperation<?> operation;

   public EquationOperationFactory(String name, String description, Function<List<? extends EquationInput>, EquationOperation<?>> operationBuilder)
   {
      this.name = name;
      this.description = description;
      this.operationBuilder = operationBuilder;
   }

   public void setInputs(Supplier<List<EquationInput>> inputsSupplier)
   {
      this.inputsSupplier = inputsSupplier;
   }

   public EquationOperationFactory duplicate()
   {
      return new EquationOperationFactory(name, description, operationBuilder);
   }

   public EquationOperation<?> build()
   {
      if (operation == null)
         operation = operationBuilder.apply(Objects.requireNonNull(inputsSupplier.get(), "Inputs supplier returned null"));
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
}

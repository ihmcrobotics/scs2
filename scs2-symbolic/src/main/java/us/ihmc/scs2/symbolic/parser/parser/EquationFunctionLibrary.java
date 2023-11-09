package us.ihmc.scs2.symbolic.parser.parser;

import us.ihmc.scs2.symbolic.parser.EquationVariable;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class EquationFunctionLibrary
{
   private final Map<String, YoFunctionFactory> functionFactoryMap = new HashMap<>();

   public EquationFunctionLibrary()
   {
      addBuiltIn();
   }

   /**
    * Returns true if the string matches the name of a function
    */
   public boolean isFunctionName(String s)
   {
      return functionFactoryMap.containsKey(s);
   }

   /**
    * Create a new instance of single input functions
    *
    * @param name  function name
    * @param input Input variable
    * @return Resulting operation
    */
   public EquationOperation<?> create(String name, EquationVariable input)
   {
      YoFunctionFactory functionFactory = functionFactoryMap.get(name);
      if (functionFactory == null)
         return null;
      return functionFactory.create(Collections.singletonList(input));
   }

   /**
    * Create a new instance of single input functions
    *
    * @param name function name
    * @param vars Input variables
    * @return Resulting operation
    */
   public EquationOperation<?> create(String name, List<EquationVariable> vars)
   {
      YoFunctionFactory functionFactory = functionFactoryMap.get(name);
      if (functionFactory == null)
         return null;
      return functionFactory.create(vars);
   }

   /**
    * Create a new instance of a two input function from an operator character
    *
    * @param op    Which operation
    * @param left  Input variable on left
    * @param right Input variable on right
    * @return Resulting operation
    */
   public EquationOperation<?> create(EquationSymbol op, EquationVariable left, EquationVariable right)
   {
      return switch (op)
      {
         case PLUS -> EquationOperation.add(left, right);
         case MINUS -> EquationOperation.subtract(left, right);
         case TIMES -> EquationOperation.multiply(left, right);
         case DIVIDE -> EquationOperation.divide(left, right);
         case POWER -> EquationOperation.pow(left, right);
         default -> throw new RuntimeException("Unknown operation " + op);
      };
   }

   /**
    * Register a new function given a name and a factory.
    *
    * @param name            name of function
    * @param functionFactory the factory
    */
   public void add(String name, YoFunctionFactory functionFactory)
   {
      functionFactoryMap.put(name, functionFactory);
   }

   /**
    * Adds built-in functions
    */
   private void addBuiltIn()
   {
      add("abs", (inputs) ->
      {
         checkNumberOfInputs(inputs, 1);
         return EquationOperation.abs(inputs.get(0));
      });
      add("sin", (inputs) ->
      {
         checkNumberOfInputs(inputs, 1);
         return EquationOperation.sin(inputs.get(0));
      });
      add("cos", (inputs) ->
      {
         checkNumberOfInputs(inputs, 1);
         return EquationOperation.cos(inputs.get(0));
      });
      add("atan", (inputs) ->
      {
         checkNumberOfInputs(inputs, 1);
         return EquationOperation.atan(inputs.get(0));
      });
      add("exp", (inputs) ->
      {
         checkNumberOfInputs(inputs, 1);
         return EquationOperation.exp(inputs.get(0));
      });
      add("log", (inputs) ->
      {
         checkNumberOfInputs(inputs, 1);
         return EquationOperation.log(inputs.get(0));
      });
      add("sqrt", (inputs) ->
      {
         checkNumberOfInputs(inputs, 1);
         return EquationOperation.sqrt(inputs.get(0));
      });
      add("pow", (inputs) ->
      {
         checkNumberOfInputs(inputs, 2);
         return EquationOperation.pow(inputs.get(0), inputs.get(1));
      });

      add("atan2", (inputs) ->
      {
         checkNumberOfInputs(inputs, 2);
         return EquationOperation.atan2(inputs.get(0), inputs.get(1));
      });
   }

   private static void checkNumberOfInputs(List<EquationVariable> inputs, int expected)
   {
      if (inputs.size() != expected)
         throw new IllegalOperationException("Function expects: " + expected + " inputs but got: " + inputs.size());
   }

   /**
    * Functional interface for creating new instances of functions
    */
   public interface YoFunctionFactory
   {
      /**
       * Create a new instance of a function.
       *
       * @param inputs the inputs to the function
       * @return the resulting operation
       */
      EquationOperation<?> create(List<EquationVariable> inputs);
   }
}
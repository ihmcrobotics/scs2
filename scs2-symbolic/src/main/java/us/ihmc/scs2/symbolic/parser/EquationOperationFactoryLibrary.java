package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.EquationInput;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class EquationOperationFactoryLibrary
{
   private final Map<String, EquationOperationFactory> functionFactoryMap = new HashMap<>();
   private final Map<EquationSymbol, EquationOperationFactory> operatorFactoryMap = new HashMap<>();

   public EquationOperationFactoryLibrary()
   {
      for (String operationName : EquationOperationLibrary.getOperationNames())
      {
         String operationDescription = EquationOperationLibrary.getOperationDescription(operationName);
         Function<List<? extends EquationInput>, EquationOperation<?>> operationBuilder = EquationOperationLibrary.getOperationBuilder(operationName);
         add(operationName, new EquationOperationFactory(operationName, operationDescription, operationBuilder));
      }

      for (EquationSymbol symbol : EquationSymbol.values())
      {
         if (symbol.operationName != null)
         {
            EquationOperationFactory factory = get(symbol.operationName);
            if (factory != null)
               operatorFactoryMap.put(symbol, factory);
         }
      }
   }

   /**
    * Returns true if the string matches the name of a function
    */
   public boolean isFunctionName(String s)
   {
      return functionFactoryMap.containsKey(s);
   }

   public EquationOperationFactory get(String name)
   {
      EquationOperationFactory factory = functionFactoryMap.get(name);
      if (factory == null)
         return null;
      return factory.duplicate();
   }

   public EquationOperationFactory get(EquationSymbol op)
   {
      EquationOperationFactory factory = operatorFactoryMap.get(op);
      if (factory == null)
         return null;
      return factory.duplicate();
   }

   /**
    * Register a new function given a name and a factory.
    *
    * @param name                     name of function
    * @param equationOperationFactory the factory
    */
   public void add(String name, EquationOperationFactory equationOperationFactory)
   {
      functionFactoryMap.put(name, equationOperationFactory);
   }

   public List<EquationOperationFactory> getAllFunctionFactories()
   {
      return List.copyOf(functionFactoryMap.values());
   }
}
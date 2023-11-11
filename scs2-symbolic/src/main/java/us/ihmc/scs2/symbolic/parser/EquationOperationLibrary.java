package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.parser.EquationOperationFactory.AssignmentOperationFactory;
import us.ihmc.scs2.symbolic.parser.EquationOperationFactory.BinaryOperationFactory;
import us.ihmc.scs2.symbolic.parser.EquationOperationFactory.UnaryOperationFactory;

import java.util.HashMap;
import java.util.Map;

public class EquationOperationLibrary
{
   private final Map<String, EquationOperationFactory> functionFactoryMap = new HashMap<>();
   private final Map<EquationSymbol, EquationOperationFactory> operatorFactoryMap = new HashMap<>();

   public EquationOperationLibrary()
   {
      addDefaultOperations();
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

   /**
    * Adds built-in functions
    */
   private void addDefaultOperations()
   {
      add("abs", new UnaryOperationFactory("abs", Math::abs, Math::abs));
      add("sin", new UnaryOperationFactory("sin", null, Math::sin));
      add("cos", new UnaryOperationFactory("cos", null, Math::cos));
      add("tan", new UnaryOperationFactory("tan", null, Math::tan));
      add("asin", new UnaryOperationFactory("asin", null, Math::asin));
      add("acos", new UnaryOperationFactory("acos", null, Math::acos));
      add("atan", new UnaryOperationFactory("atan", null, Math::atan));
      add("atan2", new BinaryOperationFactory("atan2", null, Math::atan2));
      add("exp", new UnaryOperationFactory("exp", null, Math::exp));
      add("log", new UnaryOperationFactory("log", null, Math::log));
      add("log10", new UnaryOperationFactory("log10", null, Math::log10));
      add("sqrt", new UnaryOperationFactory("sqrt", null, Math::sqrt));
      add("pow", new BinaryOperationFactory("pow", null, Math::pow));
      add("max", new BinaryOperationFactory("max", Math::max, Math::max));
      add("min", new BinaryOperationFactory("min", Math::min, Math::min));
      add("sign", new UnaryOperationFactory("sign", null, Math::signum));

      operatorFactoryMap.put(EquationSymbol.PLUS, new BinaryOperationFactory("add", Integer::sum, Double::sum));
      operatorFactoryMap.put(EquationSymbol.MINUS, new BinaryOperationFactory("subtract", (a, b) -> a - b, (a, b) -> a - b));
      operatorFactoryMap.put(EquationSymbol.TIMES, new BinaryOperationFactory("multiply", (a, b) -> a * b, (a, b) -> a * b));
      operatorFactoryMap.put(EquationSymbol.DIVIDE, new BinaryOperationFactory("divide", (a, b) -> a / b, (a, b) -> a / b));
      operatorFactoryMap.put(EquationSymbol.POWER, new BinaryOperationFactory("power", null, Math::pow));
      operatorFactoryMap.put(EquationSymbol.ASSIGN, new AssignmentOperationFactory());
   }
}
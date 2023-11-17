package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.parser.EquationOperationFactory.AssignmentOperationFactory;
import us.ihmc.scs2.symbolic.parser.EquationOperationFactory.BinaryOperationFactory;
import us.ihmc.scs2.symbolic.parser.EquationOperationFactory.UnaryOperationFactory;

import java.util.HashMap;
import java.util.List;
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
      add("abs", new UnaryOperationFactory("abs", "Computes the absolute value of a value.", Math::abs, Math::abs));
      add("sin", new UnaryOperationFactory("sin", "Computes the trigonometric sine of an angle (rad).", null, Math::sin));
      add("cos", new UnaryOperationFactory("cos", "Computes the trigonometric cosine of an angle (rad).", null, Math::cos));
      add("tan", new UnaryOperationFactory("tan", "Computes the trigonometric tangent of an angle (rad).", null, Math::tan));
      add("asin",
          new UnaryOperationFactory("asin", "Compute the arc sine of a value; the returned angle is in the range -pi/2 through pi/2.", null, Math::asin));
      add("acos", new UnaryOperationFactory("acos", "Compute the arc cosine of a value; the returned angle is in the range 0.0 through pi.", null, Math::acos));
      add("atan",
          new UnaryOperationFactory("atan", "Compute the arc tangent of a value; the returned angle is in the range -pi/2 through pi/2.", null, Math::atan));
      add("atan2",
          new BinaryOperationFactory("atan2",
                                     "Computes the angle theta from the conversion of rectangular coordinates (x, y) to polar coordinates (r, theta). This method computes the phase theta by computing an arc tangent of y/x in the range of -pi to pi.",
                                     null,
                                     Math::atan2));
      add("exp", new UnaryOperationFactory("exp", "Computes the base-e exponential function of a value.", null, Math::exp));
      add("log", new UnaryOperationFactory("log", "Computes the natural logarithm (base e) of a value.", null, Math::log));
      add("log10", new UnaryOperationFactory("log10", "Computes the base 10 logarithm of a value.", null, Math::log10));
      add("sqrt", new UnaryOperationFactory("sqrt", "Computes the square root of a value.", null, Math::sqrt));
      add("pow", new BinaryOperationFactory("pow", "Computes  the value of the first value raised to the power of the second value.", null, Math::pow));
      add("max", new BinaryOperationFactory("max", "Computes the maximum of two values.", Math::max, Math::max));
      add("min", new BinaryOperationFactory("min", "Computes the minimum of two values.", Math::min, Math::min));
      add("sign", new UnaryOperationFactory("sign", "Computes the sign of a value.", null, Math::signum));

      operatorFactoryMap.put(EquationSymbol.PLUS, new BinaryOperationFactory("add", "Performs an addition.", Integer::sum, Double::sum));
      operatorFactoryMap.put(EquationSymbol.MINUS, new BinaryOperationFactory("subtract", "Performs a subtraction.", (a, b) -> a - b, (a, b) -> a - b));
      operatorFactoryMap.put(EquationSymbol.TIMES, new BinaryOperationFactory("multiply", "Performs a multiplication.", (a, b) -> a * b, (a, b) -> a * b));
      operatorFactoryMap.put(EquationSymbol.DIVIDE, new BinaryOperationFactory("divide", "Performs a division.", (a, b) -> a / b, (a, b) -> a / b));
      operatorFactoryMap.put(EquationSymbol.POWER,
                             new BinaryOperationFactory("power", "Raises the left value to the power of the right value.", null, Math::pow));
      operatorFactoryMap.put(EquationSymbol.ASSIGN, new AssignmentOperationFactory());
   }

   public List<EquationOperationFactory> getAllFunctionFactories()
   {
      return List.copyOf(functionFactoryMap.values());
   }
}
package us.ihmc.scs2.symbolic.parser;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.scs2.symbolic.parser.EquationOperationFactory.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class EquationOperationFactoryLibrary
{
   private final Map<String, EquationOperationFactory> functionFactoryMap = new HashMap<>();
   private final Map<EquationSymbol, EquationOperationFactory> operatorFactoryMap = new HashMap<>();

   public EquationOperationFactoryLibrary()
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
      add("abs",
          new UnaryOperationFactory("abs",
                                    "Computes the absolute value of a value.",
                                    Math::abs,
                                    Math::abs,
                                    x -> x.getValueAsDouble() > 0.0 ? x.getValueDot() : -x.getValueDot()));
      add("sin",
          new UnaryOperationFactory("sin",
                                    "Computes the trigonometric sine of an angle (rad).",
                                    null,
                                    Math::sin,
                                    x -> Math.cos(x.getValueAsDouble()) * x.getValueDot()));
      add("cos",
          new UnaryOperationFactory("cos",
                                    "Computes the trigonometric cosine of an angle (rad).",
                                    null,
                                    Math::cos,
                                    x -> -Math.sin(x.getValueAsDouble()) * x.getValueDot()));
      add("tan",
          new UnaryOperationFactory("tan",
                                    "Computes the trigonometric tangent of an angle (rad).",
                                    null,
                                    Math::tan,
                                    x -> x.getValueDot() / EuclidCoreTools.square(Math.cos(x.getValueAsDouble()))));
      add("asin",
          new UnaryOperationFactory("asin",
                                    "Compute the arc sine of a value; the returned angle is in the range -pi/2 through pi/2.",
                                    null,
                                    Math::asin,
                                    x -> x.getValueDot() / Math.sqrt(1.0 - EuclidCoreTools.square(Math.cos(x.getValueAsDouble())))));
      add("acos",
          new UnaryOperationFactory("acos",
                                    "Compute the arc cosine of a value; the returned angle is in the range 0.0 through pi.",
                                    null,
                                    Math::acos,
                                    x -> -x.getValueDot() / Math.sqrt(1.0 - EuclidCoreTools.square(Math.cos(x.getValueAsDouble())))));
      add("atan",
          new UnaryOperationFactory("atan",
                                    "Compute the arc tangent of a value; the returned angle is in the range -pi/2 through pi/2.",
                                    null,
                                    Math::atan,
                                    x -> x.getValueDot() / (1.0 + EuclidCoreTools.square(x.getValueAsDouble()))));
      add("atan2",
          new BinaryOperationFactory("atan2",
                                     "Computes the angle theta from the conversion of rectangular coordinates (x, y) to polar coordinates (r, theta). This method computes the phase theta by computing an arc tangent of y/x in the range of -pi to pi.",
                                     null,
                                     Math::atan2,
                                     (y, x) -> (y.getValueDot() * x.getValueAsDouble() - y.getValueAsDouble() * x.getValueDot()) / (
                                           EuclidCoreTools.square(x.getValueAsDouble()) + EuclidCoreTools.square(y.getValueAsDouble()))));
      add("exp",
          new UnaryOperationFactory("exp",
                                    "Computes the base-e exponential function of a value.",
                                    null,
                                    Math::exp,
                                    x -> x.getValueDot() * Math.exp(x.getValueAsDouble())));
      add("log",
          new UnaryOperationFactory("log",
                                    "Computes the natural logarithm (base e) of a value.",
                                    null,
                                    Math::log,
                                    x -> x.getValueDot() / x.getValueAsDouble()));
      add("log10",
          new UnaryOperationFactory("log10",
                                    "Computes the base 10 logarithm of a value.",
                                    null,
                                    Math::log10,
                                    x -> x.getValueDot() / (x.getValueAsDouble() * Math.log(10.0))));
      add("sqrt",
          new UnaryOperationFactory("sqrt",
                                    "Computes the square root of a value.",
                                    null,
                                    Math::sqrt,
                                    x -> x.getValueDot() / (2.0 * Math.sqrt(x.getValueAsDouble()))));
      BinaryOperationFactory powFactory = new BinaryOperationFactory("pow",
                                                                     "Computes  the value of the first value raised to the power of the second value.",
                                                                     null,
                                                                     Math::pow,
                                                                     (a, b) ->
                                                                     {
                                                                        double aDot = a.getValueDot();
                                                                        double bDot = b.getValueDot();
                                                                        double aVal = a.getValueAsDouble();
                                                                        double bVal = b.getValueAsDouble();
                                                                        return aDot * bVal * Math.pow(aVal, bVal - 1.0) + bDot * Math.log(aVal) * Math.pow(aVal,
                                                                                                                                                           bVal);
                                                                     });
      add("pow", powFactory);
      add("max",
          new BinaryOperationFactory("max",
                                     "Computes the maximum of two values.",
                                     Math::max,
                                     Math::max,
                                     (a, b) -> a.getValueAsDouble() > b.getValueAsDouble() ? a.getValueDot() : b.getValueDot()));
      add("min",
          new BinaryOperationFactory("min",
                                     "Computes the minimum of two values.",
                                     Math::min,
                                     Math::min,
                                     (a, b) -> a.getValueAsDouble() < b.getValueAsDouble() ? a.getValueDot() : b.getValueDot()));
      add("sign", new UnaryOperationFactory("sign", "Computes the sign of a value.", null, Math::signum, x -> 0.0));
      add("diff", new DifferentiateOperationFactory());
      add("lpf", new LowPassFilterOperationFactory());

      operatorFactoryMap.put(EquationSymbol.PLUS,
                             new BinaryOperationFactory("add",
                                                        "Performs an addition.",
                                                        Integer::sum,
                                                        Double::sum,
                                                        (a, b) -> a.getValueDot() + b.getValueDot()));
      operatorFactoryMap.put(EquationSymbol.MINUS,
                             new BinaryOperationFactory("subtract",
                                                        "Performs a subtraction.",
                                                        (a, b) -> a - b,
                                                        (a, b) -> a - b,
                                                        (a, b) -> a.getValueDot() - b.getValueDot()));
      operatorFactoryMap.put(EquationSymbol.TIMES,
                             new BinaryOperationFactory("multiply",
                                                        "Performs a multiplication.",
                                                        (a, b) -> a * b,
                                                        (a, b) -> a * b,
                                                        (a, b) -> a.getValueDot() * b.getValueAsDouble() + b.getValueDot() * a.getValueAsDouble()));
      operatorFactoryMap.put(EquationSymbol.DIVIDE, new BinaryOperationFactory("divide", "Performs a division.", (a, b) -> a / b, (a, b) -> a / b, (a, b) ->
      {
         double aDot = a.getValueDot();
         double bDot = b.getValueDot();
         double aVal = a.getValueAsDouble();
         double bVal = b.getValueAsDouble();
         return (aDot * bVal - bDot * aVal) / (bVal * bVal);
      }));
      operatorFactoryMap.put(EquationSymbol.POWER, powFactory);
      operatorFactoryMap.put(EquationSymbol.ASSIGN, new AssignmentOperationFactory());
   }

   public List<EquationOperationFactory> getAllFunctionFactories()
   {
      return List.copyOf(functionFactoryMap.values());
   }
}
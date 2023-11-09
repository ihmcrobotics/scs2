package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.parser.parser.EquationParser;
import us.ihmc.scs2.symbolic.parser.parser.EquationOperation;

import java.util.ArrayList;
import java.util.List;

public class Equation
{
   /**
    * The original equation string.
    */
   private final String equationString;

   /**
    * List of operations representing this equation.
    */
   private final List<EquationOperation<?>> operations = new ArrayList<>();

   /**
    * The result of this equation.
    */
   private EquationVariable result;

   /**
    * Creates a new {@code YoEquation} from the given equation string.
    *
    * @param equationString the equation string to parse.
    * @return the parsed equation.
    */
   public static Equation parse(String equationString)
   {
      return new EquationParser().parse(equationString);
   }

   /**
    * Creates a new {@code YoEquation} from the given equation string.
    *
    * @param equationString the equation string to parse.
    */
   public Equation(String equationString)
   {
      this.equationString = equationString;
   }

   public void addOperation(EquationOperation<?> operation)
   {
      operations.add(operation);
      result = operation.getResult();
   }

   /**
    * Executes the sequence of operations
    */
   public EquationVariable compute()
   {
      for (int i = 0; i < operations.size(); i++)
      {
         operations.get(i).calculate();
      }
      return result;
   }

   /**
    * Returns the reference to the variable containing the result of this equation.
    *
    * @return the result variable.
    */
   public EquationVariable getResult()
   {
      return result;
   }

   /**
    * Returns the original string that was parsed to create this equation.
    *
    * @return the original string.
    */
   public String getEquationString()
   {
      return equationString;
   }
}
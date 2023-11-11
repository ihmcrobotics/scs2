package us.ihmc.scs2.symbolic;

import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.symbolic.parser.EquationOperation;

import java.util.List;

public class Equation
{
   private final String name;
   private final String description;
   /**
    * The original equation string.
    */
   private final String equationString;

   /**
    * List of operations representing this equation.
    */
   private final List<? extends EquationOperation<?>> operations;

   /**
    * The result of this equation.
    */
   private EquationInput result;

   /**
    * Creates a new {@code YoEquation} from the given equation string.
    *
    * @param name           the name of this equation.
    * @param description    the description of this equation.
    * @param equationString the equation string to parse.
    * @param operations     the list, in order, of operations representing this equation.
    */
   Equation(String name, String description, String equationString, List<? extends EquationOperation<?>> operations)
   {
      this.name = name;
      this.description = description;
      this.equationString = equationString;
      this.operations = operations;
      result = operations.get(operations.size() - 1).getResult();
   }

   /**
    * Executes the sequence of operations
    */
   public EquationInput compute()
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
   public EquationInput getResult()
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

   public YoEquationDefinition toYoEquationDefinition()
   {
      YoEquationDefinition definition = new YoEquationDefinition();
      definition.setName(name);
      definition.setDescription(description);
      definition.setEquation(equationString);
      // TODO Figure out what to do with the aliases, maybe we don't even need them here.
      return definition;
   }
}
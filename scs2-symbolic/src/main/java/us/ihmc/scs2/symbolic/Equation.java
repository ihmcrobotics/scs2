package us.ihmc.scs2.symbolic;

import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.symbolic.parser.EquationOperation;
import us.ihmc.scs2.symbolic.parser.EquationParser;

import java.util.List;

public class Equation
{
   private final String name;
   private final String description;

   private final EquationBuilder builder;

   /**
    * List of operations representing this equation.
    */
   private List<? extends EquationOperation<?>> operations;

   /**
    * The result of this equation.
    */
   private EquationInput result;

   public static Equation parse(String equationString)
   {
      return parse(null, null, equationString);
   }

   public static Equation parse(String equationString, EquationParser parser)
   {
      return parse(null, null, equationString, parser);
   }

   public static Equation parse(String name, String equationString)
   {
      return parse(name, null, equationString);
   }

   public static Equation parse(String name, String equationString, EquationParser parser)
   {
      return parse(name, null, equationString, parser);
   }

   public static Equation parse(String name, String description, String equationString)
   {
      return parse(name, description, equationString, new EquationParser());
   }

   public static Equation parse(String name, String description, String equationString, EquationParser parser)
   {
      return new Equation(name, description, parser.parse(equationString));
   }

   /**
    * Creates a new {@code YoEquation} from the given equation string.
    *
    * @param name        the name of this equation.
    * @param description the description of this equation.
    */
   Equation(String name, String description, EquationBuilder builder)
   {
      this.name = name;
      this.description = description;
      this.builder = builder;
   }

   public void build()
   {
      if (operations != null)
         throw new RuntimeException("Already built!");
      if (!builder.isReady())
         return;

      this.operations = builder.build();
      result = operations.get(operations.size() - 1).getResult();
   }

   public boolean isBuilt()
   {
      return operations != null;
   }

   /**
    * Executes the sequence of operations
    */
   public EquationInput compute()
   {
      if (!isBuilt())
         build();

      if (isBuilt())
      {
         for (int i = 0; i < operations.size(); i++)
         {
            operations.get(i).calculate();
         }
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
      return builder.getEquationString();
   }

   public YoEquationDefinition toYoEquationDefinition()
   {
      YoEquationDefinition definition = new YoEquationDefinition();
      definition.setName(name);
      definition.setDescription(description);
      definition.setEquation(getEquationString());
      // TODO Figure out what to do with the aliases, maybe we don't even need them here.
      return definition;
   }
}
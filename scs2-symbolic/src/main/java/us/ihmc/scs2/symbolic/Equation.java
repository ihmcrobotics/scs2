package us.ihmc.scs2.symbolic;

import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationAliasDefinition;
import us.ihmc.scs2.symbolic.parser.EquationAliasManager;
import us.ihmc.scs2.symbolic.parser.EquationOperation;
import us.ihmc.scs2.symbolic.parser.EquationParser;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Collectors;

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

   public static Equation fromDefinition(YoEquationDefinition equationDefinition)
   {
      return fromDefinition(equationDefinition, null);
   }

   public static Equation fromDefinition(YoEquationDefinition equationDefinition, EquationParser parser)
   {
      return parse(equationDefinition.getName(),
                   equationDefinition.getDescription(),
                   equationDefinition.getEquation(),
                   equationDefinition.getAliases().stream().collect(Collectors.toMap(EquationAliasDefinition::getName, EquationAliasDefinition::getValue)),
                   parser);
   }

   public static Equation parse(String equationString)
   {
      return parse(null, null, equationString, null, null);
   }

   public static Equation parse(String equationString, EquationParser parser)
   {
      return parse(null, null, equationString, null, parser);
   }

   public static Equation parse(String name, String description, String equationString, Map<String, String> aliasMap, EquationParser parser)
   {
      if (parser == null)
         parser = new EquationParser();

      EquationBuilder equationBuilder = parser.parse(equationString);

      if (aliasMap != null)
      {
         EquationAliasManager equationAliasManager = equationBuilder.getAliasManager();
         for (Entry<String, String> entry : aliasMap.entrySet())
         {
            String aliasName = entry.getKey();
            String aliasValue = entry.getValue();
            equationAliasManager.addVariable(aliasName, aliasValue);
         }
      }

      return new Equation(name, description, equationBuilder);
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
      else
      {
         throw new RuntimeException(
               "Failed to build the equation: " + builder.getEquationString() + ", missing inputs: " + builder.getAliasManager().getMissingInputs());
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

   public EquationBuilder getBuilder()
   {
      return builder;
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
      definition.setEquation(builder.getEquationString());
      definition.setAliases(builder.getAliasManager().toUserAliasDefinitions());
      return definition;
   }
}
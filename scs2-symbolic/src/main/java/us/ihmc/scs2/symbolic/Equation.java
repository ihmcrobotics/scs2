package us.ihmc.scs2.symbolic;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationAliasDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationInputDefinition;
import us.ihmc.scs2.sharedMemory.YoDoubleBuffer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.symbolic.parser.EquationAliasManager;
import us.ihmc.scs2.symbolic.parser.EquationOperation;
import us.ihmc.scs2.symbolic.parser.EquationParser;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

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

   private boolean warnOnMissingInputs = true;

   public static Equation fromDefinition(YoEquationDefinition equationDefinition)
   {
      return fromDefinition(equationDefinition, null);
   }

   public static Equation fromDefinition(YoEquationDefinition equationDefinition, EquationParser parser)
   {
      if (parser == null)
         parser = new EquationParser();

      EquationBuilder equationBuilder = parser.parse(equationDefinition.getEquation());

      List<EquationAliasDefinition> aliases = equationDefinition.getAliases();
      if (aliases != null)
      {
         EquationAliasManager equationAliasManager = equationBuilder.getAliasManager();
         for (EquationAliasDefinition alias : aliases)
         {
            String aliasName = alias.getName();
            EquationInputDefinition aliasValue = alias.getValue();
            equationAliasManager.addAlias(aliasName, aliasValue);
         }
      }

      return new Equation(equationDefinition.getName(), equationDefinition.getDescription(), equationBuilder);
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
            equationAliasManager.addAlias(aliasName, aliasValue);
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
      result = operations.get(operations.size() - 1);
   }

   public boolean isBuilt()
   {
      return operations != null;
   }

   /**
    * Executes the sequence of operations
    */
   public EquationInput compute(double time)
   {
      checkBuildStatus();

      if (!isBuilt())
         return null;

      operations.forEach(equationOperation -> equationOperation.updateValue(time));
      operations.forEach(equationOperation -> equationOperation.updatePreviousValue());

      return result;
   }

   /**
    * Updates the history of the variables used in this equation by evaluating every single data point in the active part of the buffer.
    */
   public void updateHistory(YoDouble yoTime)
   {
      if (!builder.getAliasManager().hasBuffer())
         return;

      checkBuildStatus();

      if (!isBuilt())
         return;

      builder.getAliasManager().setHistoryUpdate(true);
      operations.forEach(EquationOperation::reset);
      YoBufferPropertiesReadOnly bufferProperties = builder.getAliasManager().getBufferProperties();
      YoDoubleBuffer timeBuffer = (YoDoubleBuffer) builder.getAliasManager().getYoSharedBuffer().getRegistryBuffer().findYoVariableBuffer(yoTime);

      int historyIndex = bufferProperties.getInPoint();

      for (int i = 0; i < bufferProperties.getActiveBufferLength(); i++)
      {
         builder.getAliasManager().setHistoryIndex(historyIndex);

         double time = timeBuffer.getBuffer()[historyIndex];
         operations.forEach(equationOperation -> equationOperation.updateValue(time));
         operations.forEach(equationOperation -> equationOperation.updatePreviousValue());

         historyIndex = SharedMemoryTools.increment(historyIndex, 1, bufferProperties.getSize());
      }

      builder.getAliasManager().setHistoryUpdate(false);
      operations.forEach(EquationOperation::reset);
   }

   public void reset()
   {
      checkBuildStatus();

      if (!isBuilt())
         return;

      for (EquationOperation<?> operation : operations)
      {
         operation.reset();
      }
   }

   private void checkBuildStatus()
   {
      if (!isBuilt())
         build();

      if (!isBuilt())
      {
         if (warnOnMissingInputs)
            LogTools.error("Failed to build the equation: {}, missing inputs: {}.", builder.getEquationString(), builder.getAliasManager().getMissingInputs());
         warnOnMissingInputs = false;
      }
      else
      {
         warnOnMissingInputs = true;
      }
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
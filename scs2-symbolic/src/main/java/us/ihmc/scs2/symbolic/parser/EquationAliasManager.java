package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationAliasDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationInputDefinition;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.symbolic.EquationInput;
import us.ihmc.scs2.symbolic.EquationInput.*;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.*;
import java.util.function.Supplier;

public class EquationAliasManager
{
   /**
    * Default aliases that are always available.
    */
   public static final Map<String, EquationAlias> defaultAliases;

   static
   {
      LinkedHashMap<String, EquationAlias> aliases = new LinkedHashMap<>();
      aliases.put("pi", new EquationAlias("pi", new SimpleDoubleConstant(Math.PI)));
      aliases.put("e", new EquationAlias("e", new SimpleDoubleConstant(Math.E)));
      defaultAliases = Collections.unmodifiableMap(aliases);
   }

   /**
    * User-defined aliases.
    */
   private final Map<String, EquationAlias> userAliases = new LinkedHashMap<>();
   private YoEquationInputHandler yoInputHandler = new YoEquationInputHandler();
   private final Set<String> missingInputs = new LinkedHashSet<>();

   public EquationAliasManager()
   {
   }

   public void addRegistry(YoRegistry registry)
   {
      yoInputHandler.addRegistry(registry);
   }

   public void setYoSharedBuffer(YoSharedBuffer yoSharedBuffer)
   {
      yoInputHandler.setYoSharedBuffer(yoSharedBuffer);
   }

   public boolean hasBuffer()
   {
      return yoInputHandler.hasBuffer();
   }

   public YoBufferPropertiesReadOnly getBufferProperties()
   {
      return yoInputHandler.getBufferProperties();
   }

   public void setHistoryUpdate(boolean enable)
   {
      yoInputHandler.setHistoryUpdate(enable);
   }

   public void setHistoryIndex(int historyIndex)
   {
      yoInputHandler.setHistoryIndex(historyIndex);
   }

   public EquationAliasManager duplicate()
   {
      EquationAliasManager duplicate = new EquationAliasManager();
      duplicate.userAliases.putAll(userAliases);
      duplicate.missingInputs.addAll(missingInputs);
      duplicate.yoInputHandler = yoInputHandler.duplicate();
      return duplicate;
   }

   public Supplier<List<EquationInput>> submitInputRequest(EquationToken... tokens)
   {
      return submitInputRequest(List.of(tokens));
   }

   public Supplier<List<EquationInput>> submitInputRequest(Collection<EquationToken> tokens)
   {
      for (EquationToken token : tokens)
      {
         if (token.getType() == EquationToken.Type.WORD)
         {
            EquationAlias alias = userAliases.get(token.getWord());
            if (alias == null)
            {
               missingInputs.add(token.getWord());
            }
         }
      }

      return () ->
      {
         List<EquationInput> inputs = new ArrayList<>(tokens.size());

         for (EquationToken token : tokens)
         {
            if (token.getType() == EquationToken.Type.OPERATION)
               inputs.add(token.getOperationFactory().getOperation().getResult());
            else if (token.getType() == EquationToken.Type.WORD)
               inputs.add(getAlias(token.getWord()).input);
            else if (token.getType() == EquationToken.Type.VARIABLE)
               inputs.add(token.getVariable());
            else
               throw new IllegalArgumentException("Unexpected token: " + token);
         }
         return inputs;
      };
   }

   public Set<String> getMissingInputs()
   {
      return missingInputs;
   }

   public EquationAlias getAlias(String name)
   {
      EquationAlias alias = userAliases.get(name);
      if (alias == null)
         return defaultAliases.get(name);
      return alias;
   }

   public Map<String, EquationAlias> getUserAliases()
   {
      return userAliases;
   }

   public EquationAlias addConstant(String name, double value)
   {
      return addAlias(name, new SimpleDoubleConstant(value));
   }

   public EquationAlias addConstant(String name, int value)
   {
      return addAlias(name, new SimpleIntegerConstant(value));
   }

   /**
    * Adds a new constant which can either be a simple value like a double or integer, or a {@code YoVariable}.
    *
    * @param name  the name of the alias which can be used in the equation.
    * @param value the value of the constant, can either be a double value, an integer value, or the name of a {@code YoVariable}.
    * @return the alias that was added or {@code null} if the value is neither a double, an integer, nor the name of a {@code YoVariable}.
    */
   public EquationAlias addConstant(String name, String value)
   {
      try
      {
         return addConstant(name, Double.parseDouble(value));
      }
      catch (NumberFormatException e)
      {
         // ignore, just means it's not a double
      }

      try
      {
         return addConstant(name, Integer.parseInt(value));
      }
      catch (NumberFormatException e)
      {
         // ignore, just means it's not an integer
      }

      return addAlias(name, yoInputHandler.searchYoEquationInput(value));
   }

   public EquationAlias addVariable(String name, Type type)
   {
      return addAlias(name, EquationInput.newVariable(type));
   }

   public EquationAlias addVariable(String name, double value)
   {
      return addAlias(name, new SimpleDoubleVariable(value));
   }

   public EquationAlias addVariable(String name, int value)
   {
      return addAlias(name, new SimpleIntegerVariable(value));
   }

   /**
    * Adds a new variable which can either be a simple value like a double or integer, or a {@code YoVariable}.
    *
    * @param name  the name of the alias which can be used in the equation.
    * @param value the value of the variable, can either be a double value, an integer value, or the name of a {@code YoVariable}.
    * @return the alias that was added or {@code null} if the value is neither a double, an integer, nor the name of a {@code YoVariable}.
    */
   public EquationAlias addVariable(String name, String value)
   {
      try
      {
         return addVariable(name, Double.parseDouble(value));
      }
      catch (NumberFormatException e)
      {
         // ignore, just means it's not a double
      }

      try
      {
         return addVariable(name, Integer.parseInt(value));
      }
      catch (NumberFormatException e)
      {
         // ignore, just means it's not an integer
      }

      return addAlias(name, yoInputHandler.searchYoEquationInput(value));
   }

   public EquationAlias addVariable(String name, EquationInputDefinition inputDefinition)
   {
      if (inputDefinition == null)
         return null;

      if (inputDefinition.getValue() != null)
      {
         if (inputDefinition.isConstant())
            return addConstant(name, inputDefinition.getValue());
         else
            return addVariable(name, inputDefinition.getValue());
      }
      else
      {
         return addAlias(name, yoInputHandler.searchYoEquationInput(inputDefinition.getYoVariableValue()));
      }
   }

   public boolean addAliases(List<EquationAliasDefinition> aliasDefinitions)
   {
      boolean success = true;

      for (EquationAliasDefinition aliasDefinition : aliasDefinitions)
      {
         EquationAlias alias = addVariable(aliasDefinition.getName(), aliasDefinition.getValue());
         if (alias == null)
            success = false;
      }

      return success;
   }

   public List<EquationAliasDefinition> toUserAliasDefinitions()
   {
      List<EquationAliasDefinition> aliasDefinitions = new ArrayList<>(userAliases.size());

      for (EquationAlias alias : userAliases.values())
      {
         EquationInputDefinition aliasValue = alias.input == null ? null : alias.input.toInputDefinition();
         aliasDefinitions.add(new EquationAliasDefinition(alias.name, aliasValue));
      }

      return aliasDefinitions;
   }

   public EquationAlias addAlias(String name, EquationInput input)
   {
      if (input == null)
         return null;
      if (userAliases.containsKey(name))
         throw new IllegalArgumentException("Alias already exists: " + name);
      EquationAlias alias = new EquationAlias(name, input);
      userAliases.put(name, alias);
      missingInputs.remove(name);
      return alias;
   }

   public record EquationAlias(String name, EquationInput input)
   {
   }
}

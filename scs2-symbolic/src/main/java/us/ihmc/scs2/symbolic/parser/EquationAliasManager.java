package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.EquationInput;
import us.ihmc.scs2.symbolic.EquationInput.*;

import java.util.HashMap;
import java.util.Map;

public class EquationAliasManager
{
   private final Map<String, EquationAlias> aliases = new HashMap<>();
   private final YoLibrary yoLibrary = new YoLibrary();

   public EquationAliasManager()
   {
      addConstant("pi", Math.PI);
      addConstant("e", Math.E);
   }

   public EquationInput getAlias(String name)
   {
      EquationAlias alias = aliases.get(name);
      if (alias == null)
         return null;
      return alias.input;
   }

   public Map<String, EquationAlias> getAliases()
   {
      return aliases;
   }

   public void addConstant(String name, double value)
   {
      addAlias(name, new SimpleDoubleConstant(value));
   }

   public void addConstant(String name, int value)
   {
      addAlias(name, new SimpleIntegerConstant(value));
   }

   public EquationInput addVariable(String name, Type type)
   {
      return addAlias(name, EquationInput.newVariable(type));
   }

   public EquationInput addVariable(String name, double value)
   {
      return addAlias(name, new SimpleDoubleVariable(value));
   }

   public EquationInput addVariable(String name, int value)
   {
      return addAlias(name, new SimpleIntegerVariable(value));
   }

   public EquationInput addAlias(String name, EquationInput input)
   {
      if (aliases.containsKey(name))
         throw new IllegalArgumentException("Alias already exists: " + name);
      EquationAlias alias = new EquationAlias(name, input);
      aliases.put(name, alias);
      return alias.input;
   }

   public EquationInput addYoVariable(String variableName, Type type)
   {
      return addAlias(variableName, EquationInput.newYoVariable(variableName, yoLibrary, type));
   }

   public record EquationAlias(String name, EquationInput input)
   {
   }
}

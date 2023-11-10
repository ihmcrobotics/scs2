package us.ihmc.scs2.symbolic.parser.parser;

import us.ihmc.scs2.symbolic.parser.EquationInput;
import us.ihmc.scs2.symbolic.parser.EquationInput.DoubleVariable;
import us.ihmc.scs2.symbolic.parser.EquationInput.IntegerConstant;
import us.ihmc.scs2.symbolic.parser.EquationInput.IntegerVariable;
import us.ihmc.scs2.symbolic.parser.EquationInput.Type;

import java.util.HashMap;
import java.util.Map;

public class EquationAliasManager
{
   private final Map<String, EquationAlias> aliases = new HashMap<>();

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
      addAlias(name, new EquationInput.DoubleConstant(value));
   }

   public void addConstant(String name, int value)
   {
      addAlias(name, new IntegerConstant(value));
   }

   public EquationInput addVariable(String name, Type type)
   {
      switch (type)
      {
         case INTEGER:
            return addVariable(name, 0);
         case DOUBLE:
            return addVariable(name, 0.0);
         default:
            throw new UnsupportedOperationException("Unexpected type: " + type);
      }
   }

   public EquationInput addVariable(String name, double value)
   {
      return addAlias(name, new DoubleVariable(value));
   }

   public EquationInput addVariable(String name, int value)
   {
      return addAlias(name, new IntegerVariable(value));
   }

   public EquationInput addAlias(String name, EquationInput input)
   {
      if (aliases.containsKey(name))
         throw new IllegalArgumentException("Alias already exists: " + name);
      EquationAlias alias = new EquationAlias(name, input);
      aliases.put(name, alias);
      return alias.input;
   }

   public record EquationAlias(String name, EquationInput input)
   {
   }
}

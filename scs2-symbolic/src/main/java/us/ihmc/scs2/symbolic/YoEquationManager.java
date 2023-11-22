package us.ihmc.scs2.symbolic;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationAliasDefinition;
import us.ihmc.scs2.definition.yoVariable.YoVariableDefinition;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.symbolic.parser.EquationParser;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;
import java.util.function.Consumer;

public class YoEquationManager
{
   private static final boolean SILENCE_PARSING_ERRORS = true;

   private final EquationParser equationParser = new EquationParser();

   private final Map<String, Equation> equations = new LinkedHashMap<>();

   private final List<Consumer<YoEquationListChange>> changeListeners = new ArrayList<>();
   private final YoDouble yoTime;
   private final YoRegistry userRegistry;

   public YoEquationManager(YoDouble yoTime, YoRegistry rootRegistry, YoRegistry userRegistry)
   {
      this.yoTime = yoTime;
      this.userRegistry = userRegistry;
      equationParser.getAliasManager().addRegistry(rootRegistry);
   }

   public YoEquationManager(YoDouble yoTime, YoSharedBuffer yoSharedBuffer, YoRegistry userRegistry)
   {
      this.yoTime = yoTime;
      this.userRegistry = userRegistry;
      equationParser.getAliasManager().setYoSharedBuffer(yoSharedBuffer);
   }

   public void addChangeListener(Consumer<YoEquationListChange> changeListener)
   {
      changeListeners.add(changeListener);
   }

   public boolean removeChangeListener(Consumer<YoEquationListChange> changeListener)
   {
      return changeListeners.remove(changeListener);
   }

   public void setEquationListChange(YoEquationListChange change)
   {
      switch (change.getChangeType())
      {
         case ADD:
            addEquation(change.getAddedEquations());
            break;
         case REMOVE:
            removeEquations(change.getRemovedEquations());
            break;
         case SET_ALL:
            setAllEquations(change.getEquations());
            break;
      }
   }

   public void addEquation(YoEquationDefinition equationDefinition)
   {
      addEquation(List.of(equationDefinition));
   }

   public void addEquation(List<YoEquationDefinition> equationDefinitions)
   {
      for (YoEquationDefinition equationDefinition : equationDefinitions)
      {
         Objects.requireNonNull(equationDefinition.getName());
         if (equations.containsKey(equationDefinition.getName()))
         {
            LogTools.warn("Duplicate equation name: {}; skipping.", equationDefinition.getName());
            continue;
         }
         ensureUserAliasesExist(equationDefinition);
         Equation newEquation = Equation.fromDefinition(equationDefinition, equationParser);
         updateEquationHistory(newEquation, yoTime);
         equations.put(equationDefinition.getName(), newEquation);
      }

      changeListeners.forEach(listener -> listener.accept(new YoEquationListChange(YoEquationListChange.ChangeType.ADD,
                                                                                   equationDefinitions,
                                                                                   null,
                                                                                   getEquationDefinitions())));
   }

   public Equation removeEquation(YoEquationDefinition equationDefinition)
   {
      return removeEquations(List.of(equationDefinition)).get(0);
   }

   public List<Equation> removeEquations(List<YoEquationDefinition> equationDefinitions)
   {
      List<Equation> removedEquations = new ArrayList<>();

      Equation removedEquation = null;
      for (YoEquationDefinition equationDefinition : equationDefinitions)
      {
         removedEquation = equations.remove(equationDefinition.getName());
         if (removedEquation != null)
            removedEquations.add(removedEquation);
      }
      changeListeners.forEach(listener -> listener.accept(new YoEquationListChange(YoEquationListChange.ChangeType.REMOVE,
                                                                                   null,
                                                                                   equationDefinitions,
                                                                                   getEquationDefinitions())));
      return removedEquations;
   }

   public void setAllEquations(List<YoEquationDefinition> equationDefinitions)
   {
      Set<String> newEquationNames = new HashSet<>();

      for (YoEquationDefinition equationDefinition : equationDefinitions)
      {
         Objects.requireNonNull(equationDefinition.getName());
         newEquationNames.add(equationDefinition.getName());
         Equation equation = equations.get(equationDefinition.getName());

         if (equation == null || !equation.toYoEquationDefinition().equals(equationDefinition))
         {
            ensureUserAliasesExist(equationDefinition);
            Equation newEquation = Equation.fromDefinition(equationDefinition, equationParser);
            updateEquationHistory(newEquation, yoTime);
            equations.put(equationDefinition.getName(), newEquation);
         }
      }

      equations.entrySet().removeIf(entry -> !newEquationNames.contains(entry.getKey()));
      changeListeners.forEach(listener -> listener.accept(YoEquationListChange.newList(equationDefinitions)));
   }

   private void ensureUserAliasesExist(YoEquationDefinition equationDefinition)
   {
      ensureUserAliasesExist(equationDefinition, userRegistry);
   }

   public static void ensureUserAliasesExist(YoEquationDefinition equationDefinition, YoRegistry userRegistry)
   {
      if (equationDefinition.getAliases() == null || equationDefinition.getAliases().isEmpty())
         return;

      for (EquationAliasDefinition alias : equationDefinition.getAliases())
      {
         YoVariableDefinition yoVariableValue = alias.getValue().getYoVariableValue();
         if (yoVariableValue == null)
            continue;
         if (yoVariableValue.getNamespace().startsWith(userRegistry.getNamespace().toString()))
            SharedMemoryTools.ensureYoVariableExists(userRegistry, yoVariableValue);
      }
   }

   public void reset()
   {
      for (Equation equation : equations.values())
      {
         equation.reset();
      }
   }

   public void update(double time)
   {
      for (Equation equation : equations.values())
      {
         equationCompute(equation, time);
      }
   }

   private static void equationCompute(Equation equation, double time)
   {
      try
      {
         equation.compute(time);
      }
      catch (Exception e)
      {
         if (!SILENCE_PARSING_ERRORS)
            throw e;
      }
   }

   public List<YoEquationDefinition> getEquationDefinitions()
   {
      return equations.values().stream().map(Equation::toYoEquationDefinition).toList();
   }

   private static void updateEquationHistory(Equation equation, YoDouble yoTime)
   {
      try
      {
         equation.updateHistory(yoTime);
      }
      catch (Exception e)
      {
         if (!SILENCE_PARSING_ERRORS)
            throw e;
      }
   }

   public static class YoEquationListChange
   {
      public enum ChangeType
      {ADD, REMOVE, SET_ALL}

      private final ChangeType changeType;
      private final List<YoEquationDefinition> addedEquations;
      private final List<YoEquationDefinition> removedEquations;

      private final List<YoEquationDefinition> equations;

      public static YoEquationListChange add(YoEquationDefinition addEquation)
      {
         return add(List.of(addEquation));
      }

      public static YoEquationListChange add(List<YoEquationDefinition> addEquations)
      {
         return new YoEquationListChange(ChangeType.ADD, addEquations, null, null);
      }

      public static YoEquationListChange remove(YoEquationDefinition removeEquation)
      {
         return remove(List.of(removeEquation));
      }

      public static YoEquationListChange remove(List<YoEquationDefinition> removeEquations)
      {
         return new YoEquationListChange(ChangeType.REMOVE, null, removeEquations, null);
      }

      public static YoEquationListChange newList(List<YoEquationDefinition> equations)
      {
         return new YoEquationListChange(ChangeType.SET_ALL, null, null, equations);
      }

      private YoEquationListChange(ChangeType changeType,
                                   List<YoEquationDefinition> addedEquations,
                                   List<YoEquationDefinition> removedEquations,
                                   List<YoEquationDefinition> equations)
      {
         this.changeType = changeType;
         this.addedEquations = addedEquations;
         this.removedEquations = removedEquations;
         this.equations = equations;
      }

      public ChangeType getChangeType()
      {
         return changeType;
      }

      public List<YoEquationDefinition> getAddedEquations()
      {
         return addedEquations;
      }

      public List<YoEquationDefinition> getRemovedEquations()
      {
         return removedEquations;
      }

      public List<YoEquationDefinition> getEquations()
      {
         return equations;
      }
   }
}

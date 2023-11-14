package us.ihmc.scs2.symbolic;

import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.symbolic.parser.EquationParser;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.*;
import java.util.function.Consumer;

public class YoEquationManager
{
   private final YoRegistry rootRegistry;
   private final EquationParser equationParser = new EquationParser();

   private final Map<String, Equation> equations = new LinkedHashMap<>();

   private final List<Consumer<YoEquationListChange>> changeListeners = new ArrayList<>();

   public YoEquationManager(YoRegistry rootRegistry)
   {
      this.rootRegistry = rootRegistry;
      equationParser.getAliasManager().addRegistry(rootRegistry);
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
            addEquation(change.getAddedEquation());
            break;
         case REMOVE:
            removeEquation(change.getRemovedEquation().getName());
            break;
         case REPLACE:
            replaceEquation(change.getAddedEquation());
            break;
         case SET_ALL:
            setAllEquations(change.getEquations());
            break;
      }
   }

   public void addEquation(YoEquationDefinition equationDefinition)
   {
      Objects.requireNonNull(equationDefinition.getName());
      if (equations.containsKey(equationDefinition.getName()))
         throw new IllegalArgumentException("Duplicate equation name: " + equationDefinition.getName());
      equations.put(equationDefinition.getName(), Equation.fromDefinition(equationDefinition, equationParser));
      changeListeners.forEach(listener -> listener.accept(YoEquationListChange.add(equationDefinition)));
   }

   public Equation removeEquation(String equationName)
   {
      Equation removedEquation = equations.remove(equationName);
      changeListeners.forEach(listener -> listener.accept(YoEquationListChange.remove(removedEquation.toYoEquationDefinition())));
      return removedEquation;
   }

   public Equation replaceEquation(YoEquationDefinition equationDefinition)
   {
      Equation removedEquation = equations.put(equationDefinition.getName(), Equation.fromDefinition(equationDefinition, equationParser));
      if (removedEquation == null)
         throw new IllegalArgumentException("Unknown equation name: " + equationDefinition.getName());
      changeListeners.forEach(listener -> listener.accept(YoEquationListChange.replace(equationDefinition, removedEquation.toYoEquationDefinition())));
      return removedEquation;
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
            Equation newEquation = Equation.fromDefinition(equationDefinition, equationParser);
            equations.put(equationDefinition.getName(), newEquation);
         }
      }

      equations.entrySet().removeIf(entry -> !newEquationNames.contains(entry.getKey()));
      changeListeners.forEach(listener -> listener.accept(YoEquationListChange.newList(equationDefinitions)));
   }

   public void update()
   {
      for (Equation equation : equations.values())
      {
         equation.compute();
      }
   }

   public List<YoEquationDefinition> getEquationDefinitions()
   {
      return equations.values().stream().map(Equation::toYoEquationDefinition).toList();
   }

   public static class YoEquationListChange
   {
      public enum ChangeType
      {ADD, REMOVE, REPLACE, SET_ALL}

      private final ChangeType changeType;
      private final YoEquationDefinition addedEquation;
      private final YoEquationDefinition removedEquation;

      private final List<YoEquationDefinition> equations;

      public static YoEquationListChange add(YoEquationDefinition addEquation)
      {
         return new YoEquationListChange(ChangeType.ADD, addEquation, null, null);
      }

      public static YoEquationListChange remove(YoEquationDefinition removeEquation)
      {
         return new YoEquationListChange(ChangeType.REMOVE, null, removeEquation, null);
      }

      public static YoEquationListChange replace(YoEquationDefinition addEquation, YoEquationDefinition removeEquation)
      {
         return new YoEquationListChange(ChangeType.REPLACE, addEquation, removeEquation, null);
      }

      public static YoEquationListChange newList(List<YoEquationDefinition> equations)
      {
         return new YoEquationListChange(ChangeType.SET_ALL, null, null, equations);
      }

      private YoEquationListChange(ChangeType changeType,
                                   YoEquationDefinition addedEquation,
                                   YoEquationDefinition removedEquation,
                                   List<YoEquationDefinition> equations)
      {
         this.changeType = changeType;
         this.addedEquation = addedEquation;
         this.removedEquation = removedEquation;
         this.equations = equations;
      }

      public ChangeType getChangeType()
      {
         return changeType;
      }

      public YoEquationDefinition getAddedEquation()
      {
         return addedEquation;
      }

      public YoEquationDefinition getRemovedEquation()
      {
         return removedEquation;
      }

      public List<YoEquationDefinition> getEquations()
      {
         return equations;
      }
   }
}

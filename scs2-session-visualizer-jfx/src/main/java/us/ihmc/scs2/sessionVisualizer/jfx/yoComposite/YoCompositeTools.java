package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.lang3.StringUtils;

import gnu.trove.list.array.TIntArrayList;
import javafx.util.Pair;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.definition.yoComposite.YoCompositePatternDefinition;
import us.ihmc.scs2.definition.yoComposite.YoCompositePatternListDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoYawPitchRollDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupModel;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ChartTools;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoCompositeTools
{
   public static final String YO_VARIABLE = YoVariable.class.getSimpleName();
   public static final String YO_BOOLEAN = YoBoolean.class.getSimpleName();
   public static final String YO_DOUBLE = YoDouble.class.getSimpleName();
   public static final String YO_INTEGER = YoInteger.class.getSimpleName();
   public static final String YO_LONG = YoLong.class.getSimpleName();
   public static final String YO_TUPLE2D = YoTuple2DDefinition.YoTuple2D;
   public static final String YO_TUPLE3D = YoTuple3DDefinition.YoTuple3D;
   public static final String YO_QUATERNION = YoQuaternionDefinition.YoQuaternion;
   public static final String YO_YAW_PITCH_ROLL = YoYawPitchRollDefinition.YoYawPitchRoll;

   @SuppressWarnings("unchecked")
   private static final Class<? extends YoVariable>[] yoPrimitives = new Class[] {YoDouble.class, YoBoolean.class, YoInteger.class, YoEnum.class, YoLong.class};

   public static String getYoCompositeName(YoCompositePattern definition, List<YoVariable> yoCompositeComponents)
   {
      Set<YoVariable> yoCompositeComponentSet = new HashSet<>(yoCompositeComponents);
      int numberOfComponents = definition.getComponentIdentifiers().length;

      if (numberOfComponents != yoCompositeComponentSet.size())
         return null;

      YoVariable firstYoVariable = yoCompositeComponents.get(0);
      YoCompositeName[] candidateNames = fromComponentIdentifiers(firstYoVariable.getName(), definition.getComponentIdentifiers());

      if (candidateNames == null || candidateNames.length == 0)
         return null;

      Map<String, YoVariable> yoVariableMap = yoCompositeComponentSet.stream().collect(Collectors.toMap(yoVariable -> yoVariable.getName().toLowerCase(),
                                                                                                        Function.identity()));

      for (YoCompositeName candidateName : candidateNames)
      {
         String[] componentNames = toComponentNames(definition.getComponentIdentifiers(), candidateName);
         boolean isCompositeValid = Stream.of(componentNames).map(String::toLowerCase).allMatch(yoVariableMap::containsKey);
         if (!isCompositeValid)
            continue;

         YoVariable[] components = Stream.of(componentNames).map(String::toLowerCase).map(yoVariableMap::remove).toArray(YoVariable[]::new);

         if (areAllOfSameType(components))
         {
            return candidateName.getName();
         }
         break;
      }

      return null;
   }

   public static Map<String, List<YoComposite>> searchYoCompositeLists(YoCompositeCollection collection)
   {
      Map<String, List<YoComposite>> result = new LinkedHashMap<>();

      List<YoComposite> candidates = collection.getYoComposites().stream().filter(yoComposite -> containsInteger(yoComposite.getName(), 0))
                                               .collect(Collectors.toList());
      List<YoComposite> others = collection.getYoComposites();
      others = others.stream().filter(yoComposite -> yoComposite.getName().split("[^0-9]+").length > 0).collect(Collectors.toList());

      for (YoComposite candidate : candidates)
      {
         List<YoCompositeName> listCandidateNames = fromComponentIdentifier(candidate.getName(), "0");

         for (YoCompositeName listCandidateName : listCandidateNames)
         {
            int uniquePrefixLength = candidate.getUniqueName().indexOf(listCandidateName.getPrefix());

            YoCompositeName listCandidateUniqueName;
            if (uniquePrefixLength == 0)
               listCandidateUniqueName = listCandidateName;
            else
               listCandidateUniqueName = new YoCompositeName(candidate.getUniqueName().substring(0, uniquePrefixLength) + listCandidateName.getPrefix(),
                                                             listCandidateName.getSuffix());

            List<YoComposite> possibleOtherElements;

            if (listCandidateUniqueName.getPrefix().isEmpty())
            {
               if (listCandidateUniqueName.getSuffix().isEmpty())
                  continue;

               possibleOtherElements = others.stream().filter(yoComposite -> yoComposite.getUniqueName().endsWith(listCandidateUniqueName.getSuffix()))
                                             .collect(Collectors.toList());
            }
            else if (listCandidateUniqueName.getSuffix().isEmpty())
            {
               possibleOtherElements = others.stream().filter(yoComposite -> yoComposite.getUniqueName().startsWith(listCandidateUniqueName.getPrefix()))
                                             .collect(Collectors.toList());
            }
            else
            {
               possibleOtherElements = others.stream().filter(yoComposite -> yoComposite.getUniqueName().startsWith(listCandidateUniqueName.getPrefix())
                     && yoComposite.getUniqueName().endsWith(listCandidateUniqueName.getSuffix())).collect(Collectors.toList());
            }

            if (!possibleOtherElements.isEmpty())
            {
               List<YoComposite> compositeList = new ArrayList<>();
               compositeList.add(candidate);

               int listIndex = 1;
               boolean foundNextElement = false;

               do
               {
                  foundNextElement = false;

                  for (YoComposite possibleOtherElement : possibleOtherElements)
                  {
                     String indexString = possibleOtherElement.getName()
                                                              .substring(listCandidateName.getPrefix().length(),
                                                                         possibleOtherElement.getName().length() - listCandidateName.getSuffix().length());
                     try
                     {
                        if (Integer.parseInt(indexString) == listIndex)
                        {
                           compositeList.add(possibleOtherElement);
                           foundNextElement = true;
                           break;
                        }
                     }
                     catch (NumberFormatException e)
                     {
                     }
                  }

                  listIndex++;
               }
               while (foundNextElement);

               if (compositeList.size() > 1)
               {
                  result.put(listCandidateUniqueName.getName(), compositeList);
               }
            }
         }
      }

      return result;
   }

   public static boolean containsInteger(String string, int integer)
   {
      String[] numbers = string.split("[^0-9]+");
      for (String number : numbers)
      {
         if (number.isEmpty())
            continue;
         if (Integer.parseInt(number) == integer)
            return true;
      }
      return false;
   }

   public static List<YoComposite> searchYoComposites(YoCompositePattern pattern, YoRegistry registry, Predicate<YoRegistry> registryFilter)
   {
      try
      {
         return searchYoCompositesRecursive(pattern, registry, registryFilter).getKey();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private static Pair<List<YoComposite>, List<YoVariable>> searchYoCompositesRecursive(YoCompositePattern pattern,
                                                                                        YoRegistry registry,
                                                                                        Predicate<YoRegistry> registryFilter)
   {
      if (!registryFilter.test(registry))
         return new Pair<>(Collections.emptyList(), Collections.emptyList());

      Pair<List<YoComposite>, List<YoVariable>> result = new Pair<>(new ArrayList<>(), new ArrayList<>());

      List<YoVariable> yoVariables = registry.getVariables();

      for (Class<? extends YoVariable> yoPrimitive : yoPrimitives)
      {
         List<YoVariable> searchPool = new ArrayList<>();
         for (int i = 0; i < yoVariables.size(); i++)
         {
            YoVariable yoVariable = yoVariables.get(i);

            if (yoPrimitive.isInstance(yoVariable))
               searchPool.add(yoVariable);
         }

         Pair<List<YoComposite>, List<YoVariable>> primitiveResult = searchYoComposites(pattern, searchPool, registry.getNamespace(), false);
         result.getKey().addAll(primitiveResult.getKey());
         result.getValue().addAll(primitiveResult.getValue());
      }

      List<YoRegistry> children = registry.getChildren();

      for (int i = 0; i < children.size(); i++)
      {
         YoRegistry childRegistry = children.get(i);
         Pair<List<YoComposite>, List<YoVariable>> childResult = searchYoCompositesRecursive(pattern, childRegistry, registryFilter);
         result.getKey().addAll(childResult.getKey());
         result.getValue().addAll(childResult.getValue());
      }

      if (pattern.isCrossRegistry())
      {
         // Searching cross-registry
         if (!registry.getChildren().isEmpty())
         {
            Pair<List<YoComposite>, List<YoVariable>> crossRegistryResult = searchYoComposites(pattern, result.getValue(), null, true);
            result.getKey().addAll(crossRegistryResult.getKey());
            result = new Pair<>(result.getKey(), crossRegistryResult.getValue());
         }
      }

      return result;
   }

   private static Pair<List<YoComposite>, List<YoVariable>> searchYoComposites(YoCompositePattern pattern,
                                                                               List<YoVariable> variables,
                                                                               YoNamespace namespace,
                                                                               boolean useUniqueNames)
   {
      variables = variables.stream().filter(variable -> containsAnyIgnoreCase(variable.getName(), pattern.getComponentIdentifiers()))
                           .collect(Collectors.toList());

      if (variables.isEmpty())
         return new Pair<>(Collections.emptyList(), Collections.emptyList());

      List<YoComposite> result = new ArrayList<>();
      List<YoVariable> unresolvedCandidates = new ArrayList<>();

      // Using list to cover the edge case where 2 variables have the same (ignoring the case).
      Map<String, List<NamedObjectHolder<YoVariable>>> variableMap = new LinkedHashMap<>();

      if (useUniqueNames)
      {
         Map<YoVariable, String> variableToUniqueNameMap = computeUniqueNames(variables, v -> v.getNamespace().getSubNames(), YoVariable::getName);

         for (Entry<YoVariable, String> entry : variableToUniqueNameMap.entrySet())
         {
            String variableKey = entry.getValue().toLowerCase();
            List<NamedObjectHolder<YoVariable>> container = variableMap.get(variableKey);

            if (container == null)
            {
               container = new ArrayList<>();
               variableMap.put(variableKey, container);
            }
            container.add(NamedObjectHolder.newUniqueNamedYoVariable(entry.getValue(), entry.getKey()));
         }
      }
      else
      {
         for (YoVariable variable : variables)
         {
            String variableKey = variable.getName().toLowerCase();
            List<NamedObjectHolder<YoVariable>> container = variableMap.get(variableKey);

            if (container == null)
            {
               container = new ArrayList<>();
               variableMap.put(variableKey, container);
            }
            container.add(NamedObjectHolder.newUniqueNamedYoVariable(variable.getName(), variable));
         }
      }

      while (!variableMap.isEmpty())
      {
         String yoVariableName = variableMap.values().iterator().next().get(0).getUniqueName();
         String variableKey = yoVariableName.toLowerCase();

         YoCompositeName[] candidateNames = fromComponentIdentifiers(yoVariableName, pattern.getComponentIdentifiers());

         for (YoCompositeName candidateName : candidateNames)
         {
            String[] componentNames = toComponentNames(pattern.getComponentIdentifiers(), candidateName);
            boolean isCompositeValid = Stream.of(componentNames).map(String::toLowerCase).allMatch(variableMap::containsKey);
            if (!isCompositeValid)
               continue;

            YoVariable[] components = new YoVariable[componentNames.length];

            for (int i = 0; i < componentNames.length; i++)
            {
               String componentName = componentNames[i];
               String componentKey = componentName.toLowerCase();

               List<NamedObjectHolder<YoVariable>> container = variableMap.get(componentKey);

               if (container.size() == 1)
               {
                  components[i] = container.get(0).getOriginalObject();
                  variableMap.remove(componentKey);
               }
               else
               {
                  throw new RuntimeException("Implement this edge case. Name collision: "
                        + EuclidCoreIOTools.getCollectionString("\n\t", "", "\n\t", container, Object::toString));
               }
            }

            if (namespace == null)
               namespace = findCommonNamespace(components);
            result.add(new YoComposite(pattern, candidateName.getName(), namespace, Arrays.asList(components)));

            break;
         }

         List<NamedObjectHolder<YoVariable>> container = variableMap.get(variableKey);

         if (container != null)
         {
            unresolvedCandidates.add(container.remove(0).getOriginalObject());
            if (container.isEmpty())
               variableMap.remove(variableKey);
         }
      }

      return new Pair<>(result, unresolvedCandidates);
   }

   private static YoNamespace findCommonNamespace(YoVariable[] components)
   {
      if (components == null || components.length == 0)
         return null;

      if (components.length == 1)
         return components[0].getNamespace();

      List<String> commonNamespace = components[0].getNamespace().getSubNames();

      for (int i = 1; i < components.length; i++)
      {
         List<String> componentNamespace = components[i].getNamespace().getSubNames();

         for (int j = 0; j < commonNamespace.size(); j++)
         {
            if (!commonNamespace.get(j).equals(componentNamespace.get(j)))
            {
               commonNamespace.subList(0, j + 1);
               break;
            }
         }
      }

      return new YoNamespace(commonNamespace);
   }

   public static boolean areAllOfSameType(Object[] array)
   {
      if (array == null)
         return false;
      if (array.length == 1)
         return true;

      Object firstElement = array[0];

      for (int i = 1; i < array.length; i++)
      {
         if (array[i].getClass() != firstElement.getClass())
            return false;
      }
      return true;
   }

   static boolean containsAnyIgnoreCase(CharSequence str, CharSequence... searchStrs)
   {
      if (searchStrs == null)
         return false;
      else
         return Stream.of(searchStrs).anyMatch(searchStr -> StringUtils.containsIgnoreCase(str, searchStr));
   }

   static YoCompositeName[] fromComponentIdentifiers(String variableName, String[] identifiers)
   {
      if (identifiers == null || identifiers.length == 0)
         return new YoCompositeName[0];

      List<YoCompositeName> candidateNames = fromComponentIdentifier(variableName, identifiers[0]);

      for (int i = 1; i < identifiers.length; i++)
      {
         candidateNames.addAll(fromComponentIdentifier(variableName, identifiers[i]));
      }

      return candidateNames.toArray(new YoCompositeName[candidateNames.size()]);
   }

   static List<YoCompositeName> fromComponentIdentifier(String variableName, String identifier)
   {
      int indexOfAxisName = StringUtils.indexOfIgnoreCase(variableName, identifier);

      if (indexOfAxisName == -1)
         return new ArrayList<>();

      List<YoCompositeName> candidateNames = new ArrayList<>();

      while (indexOfAxisName >= 0)
      {
         String prefix = variableName.substring(0, indexOfAxisName);
         String suffix;

         if (indexOfAxisName + 1 < variableName.length())
            suffix = variableName.substring(indexOfAxisName + identifier.length(), variableName.length());
         else
            suffix = "";

         candidateNames.add(new YoCompositeName(prefix, suffix));

         if (indexOfAxisName + 1 >= variableName.length())
            break;

         indexOfAxisName = StringUtils.indexOfIgnoreCase(variableName, identifier, indexOfAxisName + 1);
      }

      return candidateNames;
   }

   static String[] toComponentNames(String[] componentIdentifiers, YoCompositeName yoCompositeName)
   {
      if (componentIdentifiers == null || componentIdentifiers.length == 0)
         return new String[0];

      String[] componentNames = new String[componentIdentifiers.length];
      for (int i = 0; i < componentNames.length; i++)
         componentNames[i] = yoCompositeName.getPrefix() + componentIdentifiers[i] + yoCompositeName.getSuffix();
      return componentNames;
   }

   public static <T> Map<T, String> computeUniqueNames(Collection<T> nameObjectCollection,
                                                       Function<T, List<String>> namespaceFunction,
                                                       Function<T, String> nameFunction)
   {
      List<NamedObjectHolder<T>> nameObjectHolderList = new ArrayList<>();
      Map<String, List<NamedObjectHolder<T>>> nameToHolderMap = new LinkedHashMap<>();

      for (T namedObject : nameObjectCollection)
      {
         String name = nameFunction.apply(namedObject);
         List<NamedObjectHolder<T>> container = nameToHolderMap.get(name.toLowerCase());

         if (container == null)
         {
            container = new ArrayList<>();
            nameToHolderMap.put(name.toLowerCase(), container);
         }
         NamedObjectHolder<T> namedObjectHolder = new NamedObjectHolder<>(name, namespaceFunction.apply(namedObject), namedObject);
         container.add(namedObjectHolder);
         nameObjectHolderList.add(namedObjectHolder);
      }

      for (Entry<String, List<NamedObjectHolder<T>>> entry : nameToHolderMap.entrySet())
      {
         List<NamedObjectHolder<T>> homonyms = entry.getValue();

         if (homonyms.size() == 1)
         { // Trivial case: the short name is already unique
            homonyms.get(0).uniqueName = homonyms.get(0).name;
         }
         else if (homonyms.size() == 2)
         {
            NamedObjectHolder<T> h1 = homonyms.get(0);
            NamedObjectHolder<T> h2 = homonyms.get(1);

            List<String> namespace1 = h1.namespace;
            List<String> namespace2 = h2.namespace;

            if (namespace1 == null ? namespace2 == null : namespace1.equals(namespace2))
            {
               throw new IllegalArgumentException("Unsupported data structure, two elements have the same fullname: " + h1.originalObject + " and "
                     + h2.originalObject);
            }

            int namespaceIndex1 = namespace1.size() - 1;
            int namespaceIndex2 = namespace2.size() - 1;

            h1.uniqueName = h1.name;
            h2.uniqueName = h2.name;

            do
            {
               if (namespaceIndex1 >= 0)
               {
                  h1.uniqueName = namespace1.get(namespaceIndex1) + "." + h1.uniqueName;
                  namespaceIndex1--;
               }
               if (namespaceIndex2 >= 0)
               {
                  h2.uniqueName = namespace2.get(namespaceIndex2) + "." + h2.uniqueName;
                  namespaceIndex2--;
               }

               if (namespaceIndex1 == 0 && namespaceIndex2 == 0 && h1.uniqueName.toLowerCase().equals(h2.uniqueName.toLowerCase()))
                  throw new IllegalArgumentException("Unsupported data structure, two elements have the same fullname: " + h1.originalObject + " and "
                        + h2.originalObject);
            }
            while (h1.uniqueName.toLowerCase().equals(h2.uniqueName.toLowerCase()));
         }
         else
         {
            TIntArrayList namespaceIndices = new TIntArrayList();
            for (NamedObjectHolder<T> homonym : homonyms)
            {
               homonym.uniqueName = homonym.name;
               namespaceIndices.add(homonym.namespace.size() - 1);
            }

            do
            {
               int numberOfUnmodifiedUniqueNames = 0;

               for (int i = 0; i < homonyms.size(); i++)
               {
                  NamedObjectHolder<T> homonym = homonyms.get(i);
                  int namespaceIndex = namespaceIndices.get(i);

                  if (namespaceIndex >= 0)
                  {
                     homonym.uniqueName = homonym.namespace.get(namespaceIndex) + "." + homonym.uniqueName;
                     namespaceIndices.set(i, namespaceIndex - 1);
                  }
                  else
                  {
                     numberOfUnmodifiedUniqueNames++;
                  }
               }

               if (numberOfUnmodifiedUniqueNames >= 2)
                  throw new IllegalArgumentException("Cannot compute unique names for the homonyms: " + homonyms);

               List<NamedObjectHolder<T>> homonymsToProcess = new ArrayList<>();
               TIntArrayList namespaceIndexOfHomonymsToProcess = new TIntArrayList();

               for (int h1Index = 0; h1Index < homonyms.size();)
               {
                  boolean isH1Unique = true;
                  NamedObjectHolder<T> h1 = homonyms.get(h1Index);

                  for (int h2Index = h1Index + 1; h2Index < homonyms.size();)
                  {
                     NamedObjectHolder<T> h2 = homonyms.get(h2Index);

                     if (h1.uniqueName.toLowerCase().equals(h2.uniqueName.toLowerCase()))
                     {
                        isH1Unique = false;
                        homonymsToProcess.add(homonyms.remove(h2Index));
                        namespaceIndexOfHomonymsToProcess.add(namespaceIndices.removeAt(h2Index));
                     }
                     else
                     {
                        h2Index++;
                     }
                  }

                  if (!isH1Unique)
                  {
                     homonymsToProcess.add(homonyms.remove(h1Index));
                     namespaceIndexOfHomonymsToProcess.add(namespaceIndices.removeAt(h1Index));
                  }
                  else
                  {
                     h1Index++;
                  }
               }

               homonyms = homonymsToProcess;
               namespaceIndices = namespaceIndexOfHomonymsToProcess;
            }
            while (!homonyms.isEmpty());
         }
      }

      return nameObjectHolderList.stream().collect(Collectors.toMap(NamedObjectHolder::getOriginalObject, NamedObjectHolder::getUniqueName));
   }

   private static class NamedObjectHolder<T>
   {
      private String name;
      private String uniqueName;
      private List<String> namespace;
      private T originalObject;

      public static NamedObjectHolder<YoVariable> newUniqueNamedYoVariable(String uniqueName, YoVariable yoVariable)
      {
         NamedObjectHolder<YoVariable> namedObjectHolder = new NamedObjectHolder<>(yoVariable.getName(), yoVariable.getNamespace().getSubNames(), yoVariable);
         namedObjectHolder.uniqueName = uniqueName;
         return namedObjectHolder;
      }

      public NamedObjectHolder(String name, List<String> namespace, T originalObject)
      {
         this.name = name;
         this.namespace = namespace;
         this.originalObject = originalObject;
      }

      public String getUniqueName()
      {
         return uniqueName;
      }

      public T getOriginalObject()
      {
         return originalObject;
      }

      @Override
      public String toString()
      {
         return name + ", unique name: " + uniqueName + ", namespace: " + namespace;
      }
   }

   public static List<YoCompositePattern> toYoCompositePatterns(YoCompositePatternListDefinition definition)
   {
      if (definition.getYoCompositePatterns() == null)
         return Collections.emptyList();
      else
         return definition.getYoCompositePatterns().stream().map(YoCompositeTools::toYoCompositePattern).collect(Collectors.toList());
   }

   public static YoCompositePattern toYoCompositePattern(YoCompositePatternDefinition definition)
   {
      String type = definition.getName();
      boolean crossRegistry = definition.isCrossRegistry();
      String[] componentIdentifiers = definition.getIdentifiers().stream().toArray(String[]::new);
      List<ChartGroupModel> preferredChartConfigurations = definition.getPreferredConfigurations().stream().map(ChartTools::toChartIdentifierList)
                                                                     .collect(Collectors.toList());
      return new YoCompositePattern(type, crossRegistry, componentIdentifiers, preferredChartConfigurations);
   }

   public static YoCompositePatternListDefinition toYoCompositePatternListDefinition(List<YoCompositePattern> yoCompositePatterns)
   {
      YoCompositePatternListDefinition definition = new YoCompositePatternListDefinition();
      definition.setYoCompositePatterns(yoCompositePatterns.stream().map(YoCompositeTools::toYoCompositePatternDefinition).collect(Collectors.toList()));
      return definition;
   }

   public static List<YoCompositePatternDefinition> toYoCompositePatternDefinitions(Collection<? extends YoCompositePattern> yoCompositePatterns)
   {
      return yoCompositePatterns.stream().map(YoCompositeTools::toYoCompositePatternDefinition).collect(Collectors.toList());
   }

   public static YoCompositePatternDefinition toYoCompositePatternDefinition(YoCompositePattern yoCompositePattern)
   {
      YoCompositePatternDefinition definition = new YoCompositePatternDefinition();
      definition.setName(yoCompositePattern.getType());
      definition.setCrossRegistry(yoCompositePattern.isCrossRegistry());
      if (yoCompositePattern.getComponentIdentifiers() != null)
         definition.setIdentifiers(new ArrayList<>(Arrays.asList(yoCompositePattern.getComponentIdentifiers())));
      definition.setPreferredConfigurations(ChartTools.toYoChartGroupModelDefinitions(yoCompositePattern.getPreferredChartConfigurations()));
      return definition;
   }
}

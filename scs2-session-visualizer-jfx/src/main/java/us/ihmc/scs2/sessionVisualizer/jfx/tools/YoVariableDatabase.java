package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.stream.Collectors;

import org.apache.commons.text.similarity.LongestCommonSubsequence;
import org.apache.commons.text.similarity.SimilarityScore;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableDatabase
{
   private static final boolean ENABLE_FUZZY_SEARCH = SessionPropertiesHelper.loadBooleanProperty("scs2.session.gui.yovariable.enablefuzzysearch", false);

   private final YoRegistry rootRegistry;
   private final YoRegistryChangedListener registryChangedListener;
   private final LinkedYoRegistry linkedRootRegistry;
   private final List<YoVariable> allYoVariables = new ArrayList<>();
   private final Map<Class<? extends YoVariable>, List<? extends YoVariable>> allTypedYoVariables = new HashMap<>();

   private final Map<YoNamespace, YoNamespace> changedNamespaceMap = new HashMap<>();

   private final Map<String, YoVariable> previousSearchResults = new HashMap<>();
   private final Map<String, String> fromSearchToBestSubname = new HashMap<>();

   private final SimilarityScore<Double> searchEngine;

   public YoVariableDatabase(YoRegistry rootRegistry, LinkedYoRegistry linkedRootRegistry)
   {
      this.rootRegistry = rootRegistry;
      this.linkedRootRegistry = linkedRootRegistry;
      searchEngine = new SimilarityScore<Double>()
      {
         SimilarityScore<Integer> caseSensitiveSearchEngine = new LongestCommonSubsequence();

         @Override
         public Double apply(CharSequence left, CharSequence right)
         {
            return caseSensitiveSearchEngine.apply(left.toString().toLowerCase(), right.toString().toLowerCase()).doubleValue()
                  / Math.max(left.length(), right.length());
         }
      };

      allYoVariables.addAll(rootRegistry.collectSubtreeVariables());
      allTypedYoVariables.put(YoVariable.class, allYoVariables);

      registryChangedListener = new YoRegistryChangedListener()
      {
         @Override
         public void changed(Change change)
         {
            if (change.wasVariableAdded())
            {
               allYoVariables.add(change.getTargetVariable());
            }

            if (change.wasVariableRemoved())
            {
               allYoVariables.remove(change.getTargetVariable());
            }

            if (change.wasRegistryAdded())
            {
               allYoVariables.addAll(change.getTargetRegistry().collectSubtreeVariables());
            }

            if (change.wasRegistryRemoved())
            {
               allYoVariables.clear();
               allYoVariables.addAll(rootRegistry.collectSubtreeVariables());
            }

            allTypedYoVariables.clear();
            allTypedYoVariables.put(YoVariable.class, allYoVariables);
         }
      };
      rootRegistry.addListener(registryChangedListener);
   }

   public <L extends LinkedYoVariable<T>, T extends YoVariable> L linkYoVariable(T variableToLink, Object initialUser)
   {
      return linkedRootRegistry.linkYoVariable(variableToLink, initialUser);
   }

   public YoVariable searchExact(String fullnameToSearch)
   {
      return rootRegistry.findVariable(fullnameToSearch);
   }

   public YoVariable searchSimilar(String fullnameToSearch, double minScore)
   {
      return searchSimilar(fullnameToSearch, minScore, YoVariable.class);
   }

   @SuppressWarnings("unchecked")
   public <T extends YoVariable> T searchSimilar(String fullnameToSearch, double minScore, Class<? extends T> type)
   {
      if (!ENABLE_FUZZY_SEARCH)
         return null;

      // 1- First see if we've made this exact search previously.
      if (previousSearchResults.containsKey(fullnameToSearch))
         return type.cast(previousSearchResults.get(fullnameToSearch));

      // 2- Update the search's namespace based on moved registries previously identified.
      YoNamespace variableFullname = updateNamespaceToIncludeMove(new YoNamespace(fullnameToSearch));
      YoNamespace variableNamespace = variableFullname.getParent();
      String variableName = variableFullname.getShortName();

      { // 3- The YoVariable may have been renamed and is still in the same registry with a very similar name
         YoRegistry parentRegistry = rootRegistry.findRegistry(variableNamespace);
         if (parentRegistry != null)
         {
            ScoredObject<T> result = searchSimilarInRegistryShallow(variableFullname, minScore, type, parentRegistry);
            if (result != null)
            {
               previousSearchResults.put(fullnameToSearch, result.getObject());
               return result.getObject();
            }
         }
      }

      { // 4- The parent registry of the YoVariable may have been moved.
         List<YoRegistry> candidates = YoSearchTools.filterRegistries(candidate -> candidate.getName().equals(variableNamespace.getShortName()), rootRegistry);

         if (candidates != null && !candidates.isEmpty())
         {
            if (candidates.size() == 1)
            { // Found a single match.
              // Test if the variable can be found exactly
               YoVariable variable = candidates.get(0).getVariable(variableName);

               if (variable != null && type.isInstance(variable))
               {
                  LogTools.info("Detected registry moved from: " + variableNamespace + " to: " + candidates.get(0).getNamespace()
                        + " when searching for variable: " + variableName);
                  previousSearchResults.put(fullnameToSearch, variable);
                  registerRegistryMoved(variableNamespace, candidates.get(0).getNamespace());
                  return type.cast(variable);
               }

               // Search for similar variables, maybe the YoVariable got renamed.
               ScoredObject<T> result = searchSimilarInRegistryShallow(variableFullname, minScore, type, candidates.get(0));
               if (result != null)
               {
                  LogTools.info("Detected registry moved from: " + variableNamespace + " to: " + candidates.get(0).getNamespace()
                        + " when searching for variable: " + variableName);
                  previousSearchResults.put(fullnameToSearch, result.getObject());
                  registerRegistryMoved(variableNamespace, candidates.get(0).getNamespace());
                  return result.getObject();
               }
            }
            else
            { // Got more than 1 match
              // Let's return the first exact match if present.
              // Could be the wrong move in some scenario, update the algorithm as necessary.
               for (YoRegistry candidate : candidates)
               {
                  YoVariable variable = candidate.getVariable(variableName);

                  if (variable != null && type.isInstance(variable))
                  {
                     LogTools.info("Detected registry moved from: " + variableNamespace + " to: " + candidate.getNamespace() + " when searching for variable: "
                           + variableName);
                     previousSearchResults.put(fullnameToSearch, variable);
                     registerRegistryMoved(variableNamespace, candidate.getNamespace());
                     return type.cast(variable);
                  }
               }

               // Couldn't find an exact match, the variable could have be renamed. Search for similar variables in all candidates.
               List<ScoredObject<T>> similarVariables = candidates.stream()
                                                                  .map(candidate -> (ScoredObject<T>) searchSimilarInRegistryShallow(variableFullname,
                                                                                                                                     minScore,
                                                                                                                                     type,
                                                                                                                                     candidate))
                                                                  .filter(Objects::nonNull)
                                                                  .sorted()
                                                                  .collect(Collectors.toList());
               if (!similarVariables.isEmpty())
               {
                  T bestYoVariable = similarVariables.get(0).getObject();
                  double bestScore = similarVariables.get(0).getScore().doubleValue();

                  LogTools.info("Score: " + bestScore + ", query: " + variableFullname + ", result:" + bestYoVariable.getFullNameString());
                  LogTools.info("Detected registry moved from: " + variableNamespace + " to: " + bestYoVariable.getNamespace()
                        + " when searching for variable: " + variableName);
                  registerRegistryMoved(variableNamespace, bestYoVariable.getNamespace());
                  return bestYoVariable;
               }
            }
         }
      }

      // 5- Brute force search for the YoVariable name and see what we get
      List<T> exactMatches = rootRegistry.findVariables(variableName).stream().filter(type::isInstance).map(type::cast).collect(Collectors.toList());
      if (exactMatches.size() == 1)
      { // We have exactly 1 matching variable, let's assume it is the good one.
         LogTools.info("Found single exact name match for query: " + fullnameToSearch + ", result: " + exactMatches.get(0).getFullNameString());
         previousSearchResults.put(fullnameToSearch, exactMatches.get(0));
         return exactMatches.get(0);
      }

      // 6- Used knowledge from past general searches to make guesses at what we could be looking for.
      for (Entry<String, String> entry : fromSearchToBestSubname.entrySet())
      {
         String bestSubname = entry.getKey();
         String searchSubname = entry.getValue();
         if (fullnameToSearch.contains(searchSubname))
         {
            YoVariable candidate = rootRegistry.findVariable(fullnameToSearch.replace(searchSubname, bestSubname));

            if (candidate != null && type.isInstance(candidate))
            {
               LogTools.info("Search for: " + fullnameToSearch + ", found: " + candidate.getFullNameString());
               previousSearchResults.put(fullnameToSearch, candidate);
               return type.cast(candidate);
            }
         }
      }

      // 7- General search for similar variables. Expensive search that scores all variables against our fullname, if the score is good we assume it is what we were looking for.
      List<T> correctTypeVariables = (List<T>) allTypedYoVariables.computeIfAbsent(type,
                                                                                   typeLocal -> allYoVariables.stream()
                                                                                                              .filter(type::isInstance)
                                                                                                              .map(type::cast)
                                                                                                              .collect(Collectors.toList()));

      List<Number> score = new ArrayList<>();
      List<T> searchResult = YoVariableTools.search(correctTypeVariables, YoVariable::getFullNameString, fullnameToSearch, searchEngine, 1, score);

      if (searchResult != null)
      {
         T bestYoVariable = searchResult.get(0);
         String bestFullname = bestYoVariable.getFullNameString();
         
         String commonSuffix = YoGeometryNameTools.getCommonSuffix(fullnameToSearch, bestFullname);
         String searchSubname = fullnameToSearch.substring(0, fullnameToSearch.length() - commonSuffix.length());
         String bestSubname = bestFullname.substring(0, bestFullname.length() - commonSuffix.length());

         String commonPrefix = YoGeometryNameTools.getCommonPrefix(searchSubname, bestSubname);
         searchSubname = fullnameToSearch.substring(commonPrefix.length(), searchSubname.length());
         bestSubname = bestFullname.substring(commonPrefix.length(), bestSubname.length());

         LogTools.info("Score: " + score.get(0) + ", difference: [" + searchSubname + ", " + bestSubname + "], field: " + fullnameToSearch + ", result: "
               + bestFullname);

         if (score.get(0).doubleValue() >= minScore)
         {
            fromSearchToBestSubname.put(bestSubname, searchSubname);
            previousSearchResults.put(fullnameToSearch, bestYoVariable);
            return bestYoVariable;
         }
      }

      return null;
   }

   private YoNamespace updateNamespaceToIncludeMove(YoNamespace variableNamespace)
   {
      for (Entry<YoNamespace, YoNamespace> entry : changedNamespaceMap.entrySet())
      {
         YoNamespace originalNamespace = entry.getKey();

         if (variableNamespace.startsWith(originalNamespace))
         {
            YoNamespace newNamespace = entry.getValue();
            return variableNamespace.removeStart(originalNamespace).prepend(newNamespace);
         }
      }

      return variableNamespace;
   }

   private void registerRegistryMoved(YoNamespace originalNamespace, YoNamespace newNamespace)
   {
      if (!originalNamespace.getShortName().equals(newNamespace.getShortName()))
         return;

      // Maybe it is one of the ancestor of the registry that has been moved.
      // So we're gonna "climb up" the structure until we identify exactly which registry has moved.
      YoNamespace originalParentNamespace = originalNamespace.getParent();
      YoNamespace newParentNamespace = newNamespace.getParent();

      while (originalParentNamespace.getShortName().equals(newParentNamespace.getShortName()))
      {
         originalNamespace = originalParentNamespace;
         newNamespace = newParentNamespace;
         originalParentNamespace = originalParentNamespace.getParent();
         newParentNamespace = newParentNamespace.getParent();
      }

      changedNamespaceMap.put(originalNamespace, newNamespace);
   }

   private <T extends YoVariable> ScoredObject<T> searchSimilarInRegistryShallow(YoNamespace variableFullname,
                                                                                 double minScore,
                                                                                 Class<? extends T> type,
                                                                                 YoRegistry registry)
   {
      List<T> registryVariables = registry.getVariables().stream().filter(type::isInstance).map(type::cast).collect(Collectors.toList());
      List<Number> score = new ArrayList<>();
      List<T> searchResult = YoVariableTools.search(registryVariables, YoVariable::getName, variableFullname.getShortName(), searchEngine, 1, score);
      if (searchResult != null)
      {
         T bestYoVariable = searchResult.get(0);

         LogTools.info("Score: " + score.get(0) + ", query: " + variableFullname + ", result:" + bestYoVariable.getFullNameString());

         if (score.get(0).doubleValue() >= minScore)
            return new ScoredObject<>(bestYoVariable, score.get(0));
      }
      return null;
   }

   public void dispose()
   {
      rootRegistry.removeListener(registryChangedListener);
      allYoVariables.clear();
      allTypedYoVariables.clear();
      changedNamespaceMap.clear();
      previousSearchResults.clear();
      fromSearchToBestSubname.clear();
   }
}
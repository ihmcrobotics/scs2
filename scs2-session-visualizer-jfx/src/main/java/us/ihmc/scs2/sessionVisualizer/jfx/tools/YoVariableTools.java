package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;
import java.util.function.Function;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import org.apache.commons.text.similarity.JaccardSimilarity;
import org.apache.commons.text.similarity.JaroWinklerDistance;
import org.apache.commons.text.similarity.LevenshteinDistance;
import org.apache.commons.text.similarity.LongestCommonSubsequence;
import org.apache.commons.text.similarity.SimilarityScore;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.RegularExpression;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search.SearchEngines;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.buffer.YoBufferVariableEntry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableTools
{
   private static final boolean SORT_YO_VARIABLES = false;

   public static long hashCode(YoRegistry registry)
   {
      long hashCode = 1L;
      for (YoVariable yoVariable : registry.collectSubtreeVariables())
      {
         hashCode = 31L * hashCode + hashCode(yoVariable);
      }
      return hashCode;
   }
   
   public static long hashCode(YoBuffer buffer)
   {
      final int prime = 31;
      long result = 1;
      
      int inPoint = buffer.getInPoint();
      int outPoint = buffer.getOutPoint();
      
      List<YoBufferVariableEntry> entries = buffer.getEntries();
      for(int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry entry = entries.get(i);
         double[] bufferWindow = entry.getBufferWindow(inPoint, outPoint);
         result = prime * result + Arrays.hashCode(bufferWindow);
      }
      
      return result;
   }

   public static long hashCode(YoVariable yoVariable)
   {
      return yoVariable.getFullNameString().hashCode();
   }

   public static <V extends YoVariable> List<V> sortYoVariablesByName(Collection<V> yoVariablesToSort)
   {
      return sortByName(yoVariablesToSort, YoVariable::getName);
   }

   public static <E> List<E> sortByName(Collection<E> collection, Function<E, String> nameGetter)
   {
      return collection.stream().sorted((e1, e2) -> nameGetter.apply(e1).compareTo(nameGetter.apply(e2))).collect(Collectors.toList());
   }

   public static List<YoFrameTuple2D> searchYoTuple2Ds(Collection<? extends YoVariable> yoVariablesToSearch, ReferenceFrame tuplesFrame)
   {
      String axisName = "x";
      List<YoDouble> yoDoubles = yoVariablesToSearch.stream().filter(YoDouble.class::isInstance).map(v -> (YoDouble) v).collect(Collectors.toList());
      List<YoDouble> xCoordinateCandidates = yoDoubles.stream().filter(v -> v.getName().toLowerCase().contains(axisName)).collect(Collectors.toList());

      List<YoFrameTuple2D> foundYoTuple2Ds = new ArrayList<>();

      for (YoDouble xCoordinateCandidate : xCoordinateCandidates)
      {
         String varName = xCoordinateCandidate.getName();
         String varNameLowerCase = varName.toLowerCase();
         String namespace = xCoordinateCandidate.getNamespace().toString();

         int indexOfAxisName = varNameLowerCase.indexOf(axisName);

         while (indexOfAxisName >= 0)
         {
            String prefix = varName.substring(0, indexOfAxisName);
            String suffix;

            if (indexOfAxisName + 1 < varName.length())
               suffix = varName.substring(indexOfAxisName + 1, varName.length());
            else
               suffix = "";

            YoFramePoint2D searchResult = findYoFramePoint2D(namespace, prefix, suffix, yoDoubles, tuplesFrame);

            if (searchResult != null && !doesYoFrameTuple3DExist(namespace, prefix, suffix, yoDoubles)
                  && !doesYoFrameQuaternionExist(namespace, prefix, suffix, yoDoubles))
               foundYoTuple2Ds.add(searchResult);

            if (indexOfAxisName + 1 >= varName.length())
               break;

            indexOfAxisName = varNameLowerCase.indexOf(axisName, indexOfAxisName + 1);
         }
      }

      return foundYoTuple2Ds;
   }

   public static List<YoFrameTuple3D> searchYoTuple3Ds(Collection<? extends YoVariable> yoVariablesToSearch, ReferenceFrame tuplesFrame)
   {
      String axisName = "x";
      List<YoDouble> yoDoubles = yoVariablesToSearch.stream().filter(YoDouble.class::isInstance).map(v -> (YoDouble) v).collect(Collectors.toList());
      List<YoDouble> xCoordinateCandidates = yoDoubles.stream().filter(v -> v.getName().toLowerCase().contains(axisName)).collect(Collectors.toList());

      List<YoFrameTuple3D> foundYoTuple3Ds = new ArrayList<>();

      for (YoDouble xCoordinateCandidate : xCoordinateCandidates)
      {
         String varName = xCoordinateCandidate.getName();
         String varNameLowerCase = varName.toLowerCase();
         String namespace = xCoordinateCandidate.getNamespace().toString();

         int indexOfAxisName = varNameLowerCase.indexOf(axisName);

         while (indexOfAxisName >= 0)
         {
            String prefix = varName.substring(0, indexOfAxisName);
            String suffix;

            if (indexOfAxisName + 1 < varName.length())
               suffix = varName.substring(indexOfAxisName + 1, varName.length());
            else
               suffix = "";

            YoFramePoint3D searchResult = findYoFramePoint3D(namespace, prefix, suffix, yoDoubles, tuplesFrame);

            if (searchResult != null && !doesYoFrameQuaternionExist(namespace, prefix, suffix, yoDoubles))
               foundYoTuple3Ds.add(searchResult);

            if (indexOfAxisName + 1 >= varName.length())
               break;

            indexOfAxisName = varNameLowerCase.indexOf(axisName, indexOfAxisName + 1);
         }
      }

      return foundYoTuple3Ds;
   }

   public static List<YoFrameQuaternion> searchYoQuaternions(Collection<? extends YoVariable> yoVariablesToSearch, ReferenceFrame tuplesFrame)
   {
      String axisName = "x";
      List<YoDouble> yoDoubles = yoVariablesToSearch.stream().filter(YoDouble.class::isInstance).map(v -> (YoDouble) v).collect(Collectors.toList());
      List<YoDouble> xCoordinateCandidates = yoDoubles.stream().filter(v -> v.getName().toLowerCase().contains(axisName)).collect(Collectors.toList());

      List<YoFrameQuaternion> foundYoTuple3Ds = new ArrayList<>();

      for (YoDouble xCoordinateCandidate : xCoordinateCandidates)
      {
         String varName = xCoordinateCandidate.getName();
         String varNameLowerCase = varName.toLowerCase();
         String namespace = xCoordinateCandidate.getNamespace().toString();

         int indexOfAxisName = varNameLowerCase.indexOf(axisName);

         while (indexOfAxisName >= 0)
         {
            String prefix = varName.substring(0, indexOfAxisName);
            String suffix;

            if (indexOfAxisName + 1 < varName.length())
               suffix = varName.substring(indexOfAxisName + 1, varName.length());
            else
               suffix = "";

            YoFrameQuaternion searchResult = findYoFrameQuaternion(namespace, prefix, suffix, yoDoubles, tuplesFrame);

            if (searchResult != null)
               foundYoTuple3Ds.add(searchResult);

            if (indexOfAxisName + 1 >= varName.length())
               break;

            indexOfAxisName = varNameLowerCase.indexOf(axisName, indexOfAxisName + 1);
         }
      }

      return foundYoTuple3Ds;
   }

   public static boolean doesYoFrameTuple2DExist(String namespace, String prefix, String suffix, Collection<? extends YoVariable> yoVariablesToSearch)
   {
      return findYoFramePoint2D(namespace, prefix, suffix, yoVariablesToSearch, null) != null;
   }

   public static boolean doesYoFrameTuple2DExist(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      return findYoFramePoint2D(namespace, prefix, suffix, yoVariableHolder, null) != null;
   }

   public static boolean doesYoFrameTuple3DExist(String namespace, String prefix, String suffix, Collection<? extends YoVariable> yoVariablesToSearch)
   {
      return findYoFramePoint3D(namespace, prefix, suffix, yoVariablesToSearch, null) != null;
   }

   public static boolean doesYoFrameTuple3DExist(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      return findYoFramePoint3D(namespace, prefix, suffix, yoVariableHolder, null) != null;
   }

   public static boolean doesYoFrameQuaternionExist(String namespace, String prefix, String suffix, Collection<? extends YoVariable> yoVariablesToSearch)
   {
      return findYoFrameQuaternion(namespace, prefix, suffix, yoVariablesToSearch, null) != null;
   }

   public static boolean doesYoFrameQuaternionExist(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameQuaternion(namespace, prefix, suffix, yoVariableHolder, null) != null;
   }

   public static YoFramePoint2D findYoFramePoint2D(String namespace, String prefix, String suffix, Collection<? extends YoVariable> yoVariablesToSearch,
                                                   ReferenceFrame tupleFrame)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariablesToSearch);
      if (x == null)
         return null;
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariablesToSearch);
      if (y == null)
         return null;
      return new YoFramePoint2D(x, y, tupleFrame);
   }

   public static YoFramePoint2D findYoFramePoint2D(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder, ReferenceFrame tupleFrame)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariableHolder);
      if (x == null)
         return null;
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariableHolder);
      if (y == null)
         return null;
      return new YoFramePoint2D(x, y, tupleFrame);
   }

   public static YoFrameVector2D findYoFrameVector2D(String namespace, String prefix, String suffix, Collection<? extends YoVariable> yoVariablesToSearch,
                                                     ReferenceFrame tupleFrame)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariablesToSearch);
      if (x == null)
         return null;
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariablesToSearch);
      if (y == null)
         return null;
      return new YoFrameVector2D(x, y, tupleFrame);
   }

   public static YoFrameVector2D findYoFrameVector2D(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder,
                                                     ReferenceFrame tupleFrame)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariableHolder);
      if (x == null)
         return null;
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariableHolder);
      if (y == null)
         return null;
      return new YoFrameVector2D(x, y, tupleFrame);
   }

   public static YoFramePoint3D findYoFramePoint3D(String namespace, String prefix, String suffix, Collection<? extends YoVariable> yoVariablesToSearch,
                                                   ReferenceFrame tupleFrame)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariablesToSearch);
      if (x == null)
         return null;
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariablesToSearch);
      if (y == null)
         return null;
      YoDouble z = findYoDouble(namespace, YoGeometryNameTools.createZName(prefix, suffix), yoVariablesToSearch);
      if (z == null)
         return null;
      return new YoFramePoint3D(x, y, z, tupleFrame);
   }

   public static YoFramePoint3D findYoFramePoint3D(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder, ReferenceFrame tupleFrame)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariableHolder);
      if (x == null)
         return null;
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariableHolder);
      if (y == null)
         return null;
      YoDouble z = findYoDouble(namespace, YoGeometryNameTools.createZName(prefix, suffix), yoVariableHolder);
      if (z == null)
         return null;
      return new YoFramePoint3D(x, y, z, tupleFrame);
   }

   public static YoFrameVector3D findYoFrameVector3D(String namespace, String prefix, String suffix, Collection<? extends YoVariable> yoVariablesToSearch,
                                                     ReferenceFrame tupleFrame)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariablesToSearch);
      if (x == null)
         return null;
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariablesToSearch);
      if (y == null)
         return null;
      YoDouble z = findYoDouble(namespace, YoGeometryNameTools.createZName(prefix, suffix), yoVariablesToSearch);
      if (z == null)
         return null;
      return new YoFrameVector3D(x, y, z, tupleFrame);
   }

   public static YoFrameVector3D findYoFrameVector3D(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder,
                                                     ReferenceFrame tupleFrame)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariableHolder);
      if (x == null)
         return null;
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariableHolder);
      if (y == null)
         return null;
      YoDouble z = findYoDouble(namespace, YoGeometryNameTools.createZName(prefix, suffix), yoVariableHolder);
      if (z == null)
         return null;
      return new YoFrameVector3D(x, y, z, tupleFrame);
   }

   public static YoFrameQuaternion findYoFrameQuaternion(String namespace, String prefix, String suffix, Collection<? extends YoVariable> yoVariablesToSearch,
                                                         ReferenceFrame tupleFrame)
   {
      YoDouble qx = findYoDouble(namespace, YoGeometryNameTools.createQxName(prefix, suffix), yoVariablesToSearch);
      if (qx == null)
         return null;
      YoDouble qy = findYoDouble(namespace, YoGeometryNameTools.createQyName(prefix, suffix), yoVariablesToSearch);
      if (qy == null)
         return null;
      YoDouble qz = findYoDouble(namespace, YoGeometryNameTools.createQzName(prefix, suffix), yoVariablesToSearch);
      if (qz == null)
         return null;
      YoDouble qs = findYoDouble(namespace, YoGeometryNameTools.createQsName(prefix, suffix), yoVariablesToSearch);
      if (qs == null)
         return null;
      return new YoFrameQuaternion(qx, qy, qz, qs, tupleFrame);
   }

   public static YoFrameQuaternion findYoFrameQuaternion(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder,
                                                         ReferenceFrame tupleFrame)
   {
      YoDouble qx = findYoDouble(namespace, YoGeometryNameTools.createQxName(prefix, suffix), yoVariableHolder);
      if (qx == null)
         return null;
      YoDouble qy = findYoDouble(namespace, YoGeometryNameTools.createQyName(prefix, suffix), yoVariableHolder);
      if (qy == null)
         return null;
      YoDouble qz = findYoDouble(namespace, YoGeometryNameTools.createQzName(prefix, suffix), yoVariableHolder);
      if (qz == null)
         return null;
      YoDouble qs = findYoDouble(namespace, YoGeometryNameTools.createQsName(prefix, suffix), yoVariableHolder);
      if (qs == null)
         return null;
      return new YoFrameQuaternion(qx, qy, qz, qs, tupleFrame);
   }

   public static YoDouble findYoDouble(String namespace, String name, YoVariableHolder yoVariableHolder)
   {
      return findYoVariable(namespace, name, YoDouble.class, yoVariableHolder);
   }

   public static YoDouble findYoDouble(String namespace, String name, Collection<? extends YoVariable> yoVariablesToSearch)
   {
      return findYoVariable(namespace, name, YoDouble.class, yoVariablesToSearch);
   }

   public static YoInteger findYoInteger(String namespace, String name, YoVariableHolder yoVariableHolder)
   {
      return findYoVariable(namespace, name, YoInteger.class, yoVariableHolder);
   }

   public static YoInteger findYoInteger(String namespace, String name, Collection<? extends YoVariable> yoVariablesToSearch)
   {
      return findYoVariable(namespace, name, YoInteger.class, yoVariablesToSearch);
   }

   public static YoBoolean findYoBoolean(String namespace, String name, YoVariableHolder yoVariableHolder)
   {
      return findYoVariable(namespace, name, YoBoolean.class, yoVariableHolder);
   }

   public static YoBoolean findYoBoolean(String namespace, String name, Collection<? extends YoVariable> yoVariablesToSearch)
   {
      return findYoVariable(namespace, name, YoBoolean.class, yoVariablesToSearch);
   }

   @SuppressWarnings("unchecked")
   public static <T extends YoVariable> T findYoVariable(String namespace, String name, Class<T> clazz, Collection<? extends YoVariable> yoVariablesToSearch)
   {
      YoVariable uncheckedVariable = yoVariablesToSearch.stream().filter(v -> v.getName().equals(name))
                                                        .filter(v -> v.getNamespace().toString().equals(namespace)).findFirst().orElse(null);
      if (uncheckedVariable == null)
         return null;
      if (!clazz.isInstance(uncheckedVariable))
         return null;
      return (T) uncheckedVariable;
   }

   @SuppressWarnings("unchecked")
   public static <T extends YoVariable> T findYoVariable(String namespace, String name, Class<T> clazz, YoVariableHolder yoVariableHolder)
   {
      YoVariable uncheckedVariable = yoVariableHolder.findVariable(namespace, name);
      if (uncheckedVariable == null)
         return null;
      if (!clazz.isInstance(uncheckedVariable))
         return null;
      return (T) uncheckedVariable;
   }

   public static <T> List<T> search(Collection<T> collectionToSearch, Function<T, String> stringConverter, String searchQuery,
                                    SimilarityScore<? extends Number> searchEngine, int maxResults)
   {
      return search(collectionToSearch, stringConverter, searchQuery, searchEngine, maxResults, Collectors.toList(), null);
   }

   public static <T> List<T> search(Collection<T> collectionToSearch, Function<T, String> stringConverter, String searchQuery,
                                    SimilarityScore<? extends Number> searchEngine, int maxResults, List<Number> scoresToPack)
   {
      return search(collectionToSearch, stringConverter, searchQuery, searchEngine, maxResults, Collectors.toList(), scoresToPack);
   }

   public static <T, R> R search(Collection<T> collectionToSearch, Function<T, String> stringConverter, String searchQuery,
                                 SimilarityScore<? extends Number> searchEngine, int maxResults, Collector<T, ?, R> collector)
   {
      return search(collectionToSearch, stringConverter, searchQuery, searchEngine, maxResults, collector, null);
   }

   public static <T, R> R search(Collection<T> collectionToSearch, Function<T, String> stringConverter, String searchQuery,
                                 SimilarityScore<? extends Number> searchEngine, int maxResults, Collector<T, ?, R> collector, List<Number> scoresToPack)
   {
      if (collectionToSearch == null || collectionToSearch.isEmpty())
         return null;

      PriorityQueue<ScoredObject<T>> scoredItemQueue = new PriorityQueue<>(collectionToSearch.size());

      for (T itemToScore : collectionToSearch)
      {
         if (Thread.interrupted())
         {
            System.out.println("Search interrupted 1");
            return null;
         }

         Number score = searchEngine.apply(stringConverter.apply(itemToScore), searchQuery);
         if (score.doubleValue() > 0.0)
            scoredItemQueue.add(new ScoredObject<T>(itemToScore, score));
      }

      List<ScoredObject<T>> scoredItems = scoredItemQueue.stream().limit(maxResults).collect(Collectors.toList());

      int startIndex = 0;
      int endIndex = 0;

      while (endIndex < scoredItems.size() - 1)
      {
         if (Thread.interrupted())
         {
            System.out.println("Search interrupted 2");
            return null;
         }

         for (int i = startIndex;; i++)
         {
            if (i > scoredItems.size() - 2)
            {
               endIndex = i;
               break;
            }

            ScoredObject<T> scoredItem = scoredItems.get(i);
            ScoredObject<T> nextScoredItem = scoredItems.get(i + 1);

            if (!scoredItem.getScore().equals(nextScoredItem.getScore()))
            {
               endIndex = i;
               break;
            }
         }

         if (Thread.interrupted())
         {
            System.out.println("Search interrupted 3");
            return null;
         }

         if (SORT_YO_VARIABLES)
         {
            if (endIndex - startIndex > 1)
               Collections.sort(scoredItems.subList(startIndex, endIndex + 1),
                                (o1, o2) -> stringConverter.apply(o1.getObject()).toLowerCase().compareTo(stringConverter.apply(o2.getObject()).toLowerCase()));
         }

         startIndex = endIndex + 1;
      }

      if (scoresToPack != null)
      {
         scoresToPack.clear();

         for (ScoredObject<T> scoredObject : scoredItems)
         {
            scoresToPack.add(scoredObject.getScore());
         }
      }

      return scoredItems.stream().map(ScoredObject::getObject).collect(collector);
   }

   public static SimilarityScore<? extends Number> fromSearchEnginesEnum(SearchEngines searchEngine)
   {
      switch (searchEngine)
      {
         case DEFAULT:
            return new SimilarityScore<Number>()
            {
               @Override
               public Number apply(CharSequence left, CharSequence right)
               {
                  return RegularExpression.check(left.toString(), right.toString()) ? 1.0 : 0.0;
               }
            };
         case JACCARD:
            return new JaccardSimilarity();
         case COSINE:
            return null;
         case HAMMING:
            return new JaroWinklerDistance();
         case LEVENSHTEIN:
            return new LevenshteinDistance();
         case SUBSEQUENCE:
            return new LongestCommonSubsequence();
         default:
            throw new UnsupportedOperationException("Unsupported search method: " + searchEngine);
      }
   }
}

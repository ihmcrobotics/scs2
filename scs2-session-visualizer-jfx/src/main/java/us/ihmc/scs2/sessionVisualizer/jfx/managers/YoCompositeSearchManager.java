package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_BOOLEAN;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_DOUBLE;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_INTEGER;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_LONG;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_QUATERNION;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_TUPLE2D;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_TUPLE3D;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_VARIABLE;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.YO_YAW_PITCH_ROLL;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools.searchYoComposites;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.ConcurrentModificationException;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.function.Predicate;

import javax.xml.bind.JAXBException;

import com.google.common.base.CaseFormat;

import javafx.application.Platform;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.FXCollections;
import javafx.collections.MapChangeListener;
import javafx.collections.ObservableMap;
import javafx.collections.ObservableSet;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupModel;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIdentifier;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositePattern;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoCompositeSearchManager implements Manager
{
   private final YoCompositePattern yoVariablePattern = YoCompositePattern.singleton(YO_VARIABLE);
   private final YoCompositePattern yoBooleanPattern = YoCompositePattern.singleton(YO_BOOLEAN);
   private final YoCompositePattern yoDoublePattern = YoCompositePattern.singleton(YO_DOUBLE);
   private final YoCompositePattern yoIntegerPattern = YoCompositePattern.singleton(YO_INTEGER);
   private final YoCompositePattern yoLongPattern = YoCompositePattern.singleton(YO_LONG);

   private final Set<YoCompositePattern> primitivePatterns = new LinkedHashSet<>(Arrays.asList(yoVariablePattern,
                                                                                               yoBooleanPattern,
                                                                                               yoDoublePattern,
                                                                                               yoIntegerPattern,
                                                                                               yoLongPattern));
   private final Map<YoCompositePattern, Class<? extends YoVariable>> primitivePatternToClass = new HashMap<>();

   private final Property<YoCompositeCollection> yoTuple2DCollection = new SimpleObjectProperty<>(this, "yoTuple2DCollectionProperty", null);
   private final Property<YoCompositeCollection> yoTuple3DCollection = new SimpleObjectProperty<>(this, "yoTuple3DCollectionProperty", null);
   private final Property<YoCompositeCollection> yoQuaternionCollection = new SimpleObjectProperty<>(this, "yoQuaternionCollectionProperty", null);
   private final Property<YoCompositeCollection> yoYawPitchRollCollection = new SimpleObjectProperty<>(this, "yoYawPitchRollCollectionProperty", null);

   private final List<YoCompositePattern> defaultCompositePatterns = new ArrayList<>();

   private final ObservableSet<YoCompositePattern> yoCompositePatterns = FXCollections.observableSet(new LinkedHashSet<>());
   private final ObservableSet<YoCompositePattern> customYoCompositePatterns = FXCollections.observableSet(new LinkedHashSet<>());
   private final ObservableMap<String, YoCompositePattern> typeToCompositePattern = FXCollections.observableMap(new LinkedHashMap<>());
   private final ObservableMap<String, Property<YoCompositeCollection>> typeToCompositeCollection = FXCollections.observableMap(new LinkedHashMap<>());

   private final Map<YoCompositePattern, Map<String, List<YoComposite>>> listOfYoCompositeMaps = new HashMap<>();

   private final YoRegistryChangedListener rootRegistryChangeListener = change -> refreshYoCompositesInBackground();

   private final Property<Boolean> includeSCS2YoVariables;
   private final YoManager yoManager;
   private final BackgroundExecutorManager backgroundExecutorManager;
   /** Using this map as a set to keep track of the ongoing searches. */
   private final ConcurrentHashMap<YoCompositePattern, Object> activeSearches = new ConcurrentHashMap<>();

   private volatile boolean isSessionActive = false;

   public YoCompositeSearchManager(JavaFXMessager messager,
                                   SessionVisualizerTopics topics,
                                   YoManager yoManager,
                                   BackgroundExecutorManager backgroundExecutorManager)
   {
      this.yoManager = yoManager;
      this.backgroundExecutorManager = backgroundExecutorManager;
      List<YoCompositePattern> compositePatterns;
      try
      {
         compositePatterns = XMLTools.loadYoCompositePatterns(SessionVisualizerIOTools.getConfigurationResource(SessionVisualizerIOTools.DEFAULT_YO_COMPOSITE_PATTERNS_FILE));
      }
      catch (JAXBException | IOException e)
      {
         throw new RuntimeException("Failed to load the default " + YoCompositePattern.class.getSimpleName() + "s.", e);
      }

      typeToCompositePattern.addListener((MapChangeListener<String, YoCompositePattern>) change ->
      {
         if (change.wasAdded())
            yoCompositePatterns.add(change.getValueAdded());
         if (change.wasRemoved())
            yoCompositePatterns.remove(change.getValueRemoved());
      });

      primitivePatternToClass.put(yoVariablePattern, YoVariable.class);
      primitivePatternToClass.put(yoBooleanPattern, YoBoolean.class);
      primitivePatternToClass.put(yoDoublePattern, YoDouble.class);
      primitivePatternToClass.put(yoIntegerPattern, YoInteger.class);
      primitivePatternToClass.put(yoLongPattern, YoLong.class);

      for (YoCompositePattern primitivePattern : primitivePatterns)
      {
         String namePrefix = CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_CAMEL, primitivePattern.getType());
         Property<YoCompositeCollection> collection = new SimpleObjectProperty<>(this, namePrefix + "CollectionProperty", null);
         typeToCompositeCollection.put(primitivePattern.getType(), collection);
         typeToCompositePattern.put(primitivePattern.getType(), primitivePattern);
         primitivePattern.getPreferredChartConfigurations().add(new ChartGroupModel("single", Collections.singletonList(new ChartIdentifier(0, 0))));
      }

      for (YoCompositePattern compositePattern : compositePatterns)
      {
         typeToCompositePattern.put(compositePattern.getType(), compositePattern);
      }

      typeToCompositeCollection.put(YO_TUPLE2D, yoTuple2DCollection);
      typeToCompositeCollection.put(YO_TUPLE3D, yoTuple3DCollection);
      typeToCompositeCollection.put(YO_QUATERNION, yoQuaternionCollection);
      typeToCompositeCollection.put(YO_YAW_PITCH_ROLL, yoYawPitchRollCollection);
      defaultCompositePatterns.addAll(typeToCompositePattern.values());

      typeToCompositePattern.addListener((MapChangeListener<String, YoCompositePattern>) change ->
      {
         if (change.wasAdded())
            customYoCompositePatterns.add(change.getValueAdded());
         if (change.wasRemoved())
            customYoCompositePatterns.remove(change.getValueRemoved());
      });

      messager.registerTopicListener(topics.getYoCompositePatternLoadRequest(), this::loadYoCompositePatternFromFile);
      messager.registerTopicListener(topics.getYoCompositePatternSaveRequest(), this::saveYoCompositePatternToFile);
      messager.registerTopicListener(topics.getYoCompositeRefreshAll(), m -> refreshYoCompositesInBackground());
      includeSCS2YoVariables = messager.createPropertyInput(topics.getShowSCS2YoVariables(), false);
      includeSCS2YoVariables.addListener((o, oldValue, newValue) -> refreshYoCompositesInBackground());
   }

   @Override
   public void startSession(Session session)
   {
      LogTools.info("Searching default YoComposite.");
      isSessionActive = true;
      typeToCompositePattern.values().forEach(this::searchYoCompositeNow);
      yoManager.getRootRegistry().addListener(rootRegistryChangeListener);
      LogTools.info("Initialized default YoComposite.");
   }

   @Override
   public void stopSession()
   {
      isSessionActive = false;
      yoManager.getRootRegistry().removeListener(rootRegistryChangeListener);
      typeToCompositePattern.entrySet().removeIf(entry -> !defaultCompositePatterns.contains(entry.getValue()));
      typeToCompositeCollection.entrySet().removeIf(entry -> !typeToCompositePattern.containsKey(entry.getKey()));

      for (YoCompositePattern pattern : defaultCompositePatterns)
         typeToCompositeCollection.get(pattern.getType()).setValue(null);

      customYoCompositePatterns.clear();
      listOfYoCompositeMaps.clear();
   }

   @Override
   public boolean isSessionLoaded()
   {
      for (YoCompositePattern pattern : defaultCompositePatterns)
      {
         if (typeToCompositeCollection.get(pattern.getType()).getValue() == null)
            return false;
      }
      return true;
   }

   public void refreshYoCompositesInBackground()
   {
      typeToCompositePattern.values().forEach(this::searchYoCompositeInBackground);
   }

   public void searchYoCompositeInBackground(YoCompositePattern pattern)
   {
      backgroundExecutorManager.queueTaskToExecuteInBackground(this, () -> searchYoCompositeNow(pattern));
   }

   public void searchYoCompositeNow(YoCompositePattern pattern)
   {
      if (activeSearches.containsKey(pattern))
      { // We have an active search, let's reschedule this search.
         searchYoCompositeInBackground(pattern);
         return;
      }

      activeSearches.put(pattern, Object.class);
      try
      {
         YoRegistry rootRegistry = yoManager.getRootRegistry();

         if (!isSessionActive || rootRegistry == null)
            return; // Stopping the session

         Predicate<YoRegistry> registryFilter;

         if (includeSCS2YoVariables.getValue())
            registryFilter = reg -> true;
         else
            registryFilter = reg -> !reg.getNamespace().equals(Session.SESSION_INTERNAL_NAMESPACE);

         Class<? extends YoVariable> primitiveClass = primitivePatternToClass.get(pattern);
         String type = pattern.getType();

         if (primitiveClass != null)
         {
            YoCompositeCollection collection;
            try
            {
               collection = new YoCompositeCollection(pattern, collectPrimitiveYoComposites(pattern, primitiveClass, rootRegistry, registryFilter));
            }
            catch (ConcurrentModificationException e)
            {
               // If we have a concurrent modification, then it means we have another search scheduled waiting. Let's return.
               return;
            }
            catch (Exception e)
            {
               e.printStackTrace();
               return;
            }

            if (isSessionActive)
               JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> typeToCompositeCollection.get(type).setValue(collection));
         }
         else
         {
            List<YoComposite> result;
            try
            {
               result = searchYoComposites(pattern, rootRegistry, registryFilter);
            }
            catch (ConcurrentModificationException e)
            {
               // If we have a concurrent modification, then it means we have another search scheduled waiting. Let's return.
               return;
            }
            catch (Exception e)
            {
               if (isSessionActive)
                  e.printStackTrace();
               return;
            }

            if (result != null)
            {
               YoCompositeCollection collection = new YoCompositeCollection(pattern, result);

               if (isSessionActive)
               {
                  JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
                  {
                     Property<YoCompositeCollection> property = typeToCompositeCollection.get(type);

                     if (property == null)
                     {
                        String propertyName = type + "CollectionProperty";
                        property = new SimpleObjectProperty<>(this, propertyName, null);
                        typeToCompositeCollection.put(type, property);
                        typeToCompositePattern.put(type, pattern);
                     }

                     property.setValue(collection);
                  });
               }
            }
         }
      }
      finally
      {
         activeSearches.remove(pattern);
      }
   }

   private static List<YoComposite> collectPrimitiveYoComposites(YoCompositePattern pattern,
                                                                 Class<? extends YoVariable> primitiveClass,
                                                                 YoRegistry start,
                                                                 Predicate<YoRegistry> registryFilter)
   {
      return collectPrimitiveYoComposites(pattern, primitiveClass, start, registryFilter, new ArrayList<>());
   }

   private static List<YoComposite> collectPrimitiveYoComposites(YoCompositePattern pattern,
                                                                 Class<? extends YoVariable> primitiveClass,
                                                                 YoRegistry start,
                                                                 Predicate<YoRegistry> registryFilter,
                                                                 List<YoComposite> compositesToPack)
   {
      if (registryFilter.test(start))
      {
         for (YoVariable variable : start.getVariables())
         {
            if (primitiveClass.isInstance(variable))
               compositesToPack.add(new YoComposite(pattern, variable));
         }

         for (YoRegistry child : start.getChildren())
            collectPrimitiveYoComposites(pattern, primitiveClass, child, registryFilter, compositesToPack);
      }

      return compositesToPack;
   }

   public void discardYoComposite(String typeToDiscard)
   {
      discardYoComposite(getPatternFromType(typeToDiscard));
   }

   public void discardYoComposite(YoCompositePattern patternToDiscard)
   {
      if (patternToDiscard == null || !customYoCompositePatterns.contains(patternToDiscard))
         return;

      typeToCompositePattern.remove(patternToDiscard.getType());
      Property<YoCompositeCollection> collectionProperty = typeToCompositeCollection.remove(patternToDiscard.getType());
      if (collectionProperty != null)
         collectionProperty.setValue(null);
      listOfYoCompositeMaps.remove(patternToDiscard);
   }

   private void loadYoCompositePatternFromFile(File file)
   {
      if (file == null)
         return;
      LogTools.info("Loading file: " + file);
      try
      {
         List<YoCompositePattern> newPatterns = XMLTools.loadYoCompositePatterns(new FileInputStream(file));
         newPatterns.forEach(this::searchYoCompositeInBackground);
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   public void saveYoCompositePatternToFile(File file)
   {
      if (!Platform.isFxApplicationThread())
         throw new IllegalStateException("Save must only be used from the FX Application Thread");

      if (file == null)
         return;
      LogTools.info("Saving file: " + file);
      ArrayList<YoCompositePattern> patternsToExport = new ArrayList<>(typeToCompositePattern.values());
      patternsToExport.removeAll(defaultCompositePatterns);

      try
      {
         XMLTools.saveYoCompositePatterns(new FileOutputStream(file), patternsToExport);
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   public void requestSearchListOfYoComposites(String compositeType, Consumer<Map<String, List<YoComposite>>> callback)
   {
      requestSearchListOfYoComposites(getPatternFromType(compositeType), callback);
   }

   public void requestSearchListOfYoComposites(YoCompositePattern compositeDefintion, Consumer<Map<String, List<YoComposite>>> callback)
   {
      if (listOfYoCompositeMaps.containsKey(compositeDefintion))
      {
         callback.accept(listOfYoCompositeMaps.get(compositeDefintion));
         return;
      }

      backgroundExecutorManager.queueTaskToExecuteInBackground(this, () ->
      {
         Map<String, List<YoComposite>> result = YoCompositeTools.searchYoCompositeLists(getCollectionFromType(compositeDefintion.getType()));

         if (result != null)
         {
            JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
            {
               listOfYoCompositeMaps.put(compositeDefintion, result);
               callback.accept(result);
            });
         }
      });
   }

   public ObservableMap<String, Property<YoCompositeCollection>> typeToCompositeCollection()
   {
      return typeToCompositeCollection;
   }

   public YoComposite getYoComposite(String type, String fullname)
   {
      YoCompositeCollection collectionFromType = getCollectionFromType(type);
      if (collectionFromType != null)
         return collectionFromType.getYoCompositeFromFullname(fullname);
      else
         return null;
   }

   public YoCompositeCollection getCollectionFromType(String type)
   {
      Property<YoCompositeCollection> property = typeToCompositeCollection.get(type);
      if (property != null)
         return property.getValue();
      else
         return null;
   }

   public ObservableSet<YoCompositePattern> yoCompositePatterns()
   {
      return yoCompositePatterns;
   }

   public ObservableSet<YoCompositePattern> customYoCompositePatterns()
   {
      return customYoCompositePatterns;
   }

   public YoCompositePattern getPatternFromType(String type)
   {
      return typeToCompositePattern.get(type);
   }

   public YoCompositeCollection getYoVariableCollection()
   {
      return getCollectionFromType(yoVariablePattern.getType());
   }

   public YoCompositeCollection getYoBooleanCollection()
   {
      return getCollectionFromType(yoBooleanPattern.getType());
   }

   public YoCompositeCollection getYoDoubleCollection()
   {
      return getCollectionFromType(yoDoublePattern.getType());
   }

   public YoCompositeCollection getYoIntegerCollection()
   {
      return getCollectionFromType(yoIntegerPattern.getType());
   }

   public YoCompositeCollection getYoLongCollection()
   {
      return getCollectionFromType(yoLongPattern.getType());
   }

   public YoCompositeCollection getYoTuple2DCollection()
   {
      return yoTuple2DCollection.getValue();
   }

   public YoCompositeCollection getYoTuple3DCollection()
   {
      return yoTuple3DCollection.getValue();
   }

   public YoCompositeCollection getYoQuaternionCollection()
   {
      return yoQuaternionCollection.getValue();
   }

   public YoCompositeCollection getYoYawPitchRollCollection()
   {
      return yoYawPitchRollCollection.getValue();
   }
}

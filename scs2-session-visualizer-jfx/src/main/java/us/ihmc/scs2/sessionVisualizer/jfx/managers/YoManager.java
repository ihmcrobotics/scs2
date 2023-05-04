package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.LongProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleLongProperty;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoBooleanProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoEnumAsStringProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoIntegerProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoLongProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableDatabase;
import us.ihmc.scs2.sharedMemory.LinkedBufferProperties;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.interfaces.LinkedYoVariableFactory;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoManager extends ObservedAnimationTimer implements Manager
{
   private static final boolean DEFAULT_ENABLE_FUZZY_SEARCH = SessionPropertiesHelper.loadBooleanProperty("scs2.session.gui.yovariable.enablefuzzysearch", false);

   private final LongProperty rootRegistryChangeCounter = new SimpleLongProperty(this, "rootRegistryChangeCounter", 0);
   private final BooleanProperty enableFuzzyYoSearch = new SimpleBooleanProperty(this, "enableFuzzySearch", DEFAULT_ENABLE_FUZZY_SEARCH);
   private final YoRegistryChangedListener counterUpdater = changer -> rootRegistryChangeCounter.set(rootRegistryChangeCounter.get() + 1);

   private YoRegistry rootRegistry;
   private LinkedYoRegistry linkedRootRegistry;
   private LinkedBufferProperties linkedBufferProperties;
   private LinkedYoVariableFactory linkedYoVariableFactory;

   private boolean updatingYoVariables = true;

   private YoVariableDatabase rootRegistryDatabase = null;

   public YoManager()
   {
      enableFuzzyYoSearch.addListener((o, oldValue, newValue) -> {
         if (rootRegistryDatabase != null)
            rootRegistryDatabase.setEnableFuzzySearch(newValue);
      });
   }

   @Override
   public void handleImpl(long now)
   {
      if (linkedRootRegistry != null && !updatingYoVariables)
         linkedRootRegistry.pull();
   }

   @Override
   public void startSession(Session session)
   {
      LogTools.info("Linking YoVariables");
      rootRegistry = new YoRegistry(SimulationSession.ROOT_REGISTRY_NAME);
      linkedYoVariableFactory = session.getLinkedYoVariableFactory();
      linkedRootRegistry = linkedYoVariableFactory.newLinkedYoRegistry(rootRegistry);
      linkedBufferProperties = linkedYoVariableFactory.newLinkedBufferProperties();

      updatingYoVariables = true;
      rootRegistry.addListener(counterUpdater);
      rootRegistryChangeCounter.set(rootRegistryChangeCounter.get() + 1);
      rootRegistryDatabase = new YoVariableDatabase(rootRegistry, linkedRootRegistry);
      rootRegistryDatabase.setEnableFuzzySearch(enableFuzzyYoSearch.get());
      updatingYoVariables = false;
      LogTools.info("UI linked YoVariables created");
      start();
   }

   @Override
   public void stopSession()
   {
      rootRegistryDatabase.dispose();
      rootRegistryDatabase = null;
      linkedYoVariableFactory = null;
      linkedRootRegistry = null;
      linkedBufferProperties = null;
      rootRegistry.removeListener(counterUpdater);
      rootRegistry.clear();
      rootRegistry = null;
      rootRegistryChangeCounter.set(rootRegistryChangeCounter.get() + 1);
   }

   @Override
   public boolean isSessionLoaded()
   {
      return linkedRootRegistry != null && !updatingYoVariables;
   }

   public BooleanProperty enableFuzzyYoSearchProperty()
   {
      return enableFuzzyYoSearch;
   }

   public LinkedYoRegistry newLinkedYoRegistry(YoRegistry registry)
   {
      return linkedYoVariableFactory.newLinkedYoRegistry(registry);
   }

   public LinkedYoVariable<?> newLinkedYoVariable(YoVariable yoVariable, Object initialUser)
   {
      return linkedYoVariableFactory.newLinkedYoVariable(yoVariable, initialUser);
   }

   public YoRegistry getRootRegistry()
   {
      return rootRegistry;
   }

   public YoVariableDatabase getRootRegistryDatabase()
   {
      return rootRegistryDatabase;
   }

   public boolean isUpdatingYoVariables()
   {
      return updatingYoVariables;
   }

   public LinkedYoRegistry getLinkedRootRegistry()
   {
      return linkedRootRegistry;
   }

   public LinkedBufferProperties newLinkedBufferProperties()
   {
      if (linkedYoVariableFactory == null)
         return null;
      else
         return linkedYoVariableFactory.newLinkedBufferProperties();
   }

   public LongProperty rootRegistryChangeCounter()
   {
      return rootRegistryChangeCounter;
   }

   public int getBufferSize()
   {
      if (linkedBufferProperties == null)
         return -1;
      if (linkedBufferProperties.peekCurrentBufferProperties() == null)
         return -1;
      return linkedBufferProperties.peekCurrentBufferProperties().getSize();
   }

   public YoDoubleProperty newYoDoubleProperty(String variableName)
   {
      if (rootRegistry == null)
      {
         LogTools.error("No active session.");
         return null;
      }

      YoDouble variable = findYoVariable(YoDouble.class, variableName);
      if (variable == null)
      {
         LogTools.error("Could not find variable from name: {}", variableName);
         return null;
      }

      YoDoubleProperty property = new YoDoubleProperty(variable);
      property.setLinkedBuffer(linkedRootRegistry.linkYoVariable(variable, property));
      return property;
   }

   public YoIntegerProperty newYoIntegerProperty(String variableName)
   {
      if (rootRegistry == null)
      {
         LogTools.error("No active session.");
         return null;
      }

      YoInteger variable = findYoVariable(YoInteger.class, variableName);
      if (variable == null)
      {
         LogTools.error("Could not find variable from name: {}", variableName);
         return null;
      }

      YoIntegerProperty property = new YoIntegerProperty(variable);
      property.setLinkedBuffer(linkedRootRegistry.linkYoVariable(variable, property));
      return property;
   }

   public YoLongProperty newYoLongProperty(String variableName)
   {
      if (rootRegistry == null)
      {
         LogTools.error("No active session.");
         return null;
      }

      YoLong variable = findYoVariable(YoLong.class, variableName);
      if (variable == null)
      {
         LogTools.error("Could not find variable from name: {}", variableName);
         return null;
      }

      YoLongProperty property = new YoLongProperty(variable);
      property.setLinkedBuffer(linkedRootRegistry.linkYoVariable(variable, property));
      return property;
   }

   public YoBooleanProperty newYoBooleanProperty(String variableName)
   {
      if (rootRegistry == null)
      {
         LogTools.error("No active session.");
         return null;
      }

      YoBoolean variable = findYoVariable(YoBoolean.class, variableName);
      if (variable == null)
      {
         LogTools.error("Could not find variable from name: {}", variableName);
         return null;
      }

      YoBooleanProperty property = new YoBooleanProperty(variable);
      property.setLinkedBuffer(linkedRootRegistry.linkYoVariable(variable, property));
      return property;
   }

   public <E extends Enum<E>> YoEnumAsStringProperty<E> newYoEnumProperty(String variableName)
   {
      if (rootRegistry == null)
      {
         LogTools.error("No active session.");
         return null;
      }

      @SuppressWarnings("unchecked")
      YoEnum<E> variable = findYoVariable(YoEnum.class, variableName);
      if (variable == null)
      {
         LogTools.error("Could not find variable from name: {}", variableName);
         return null;
      }

      YoEnumAsStringProperty<E> property = new YoEnumAsStringProperty<>(variable);
      property.setLinkedBuffer(linkedRootRegistry.linkYoVariable(variable, property));
      return property;
   }

   @SuppressWarnings("unchecked")
   private <T extends YoVariable> T findYoVariable(Class<T> type, String variableName)
   {
      int separatorIndex = variableName.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      String namespaceEnding = separatorIndex == -1 ? null : variableName.substring(0, separatorIndex);
      String name = separatorIndex == -1 ? variableName : variableName.substring(separatorIndex + 1);
      return (T) YoSearchTools.findFirstVariable(namespaceEnding, name, type::isInstance, rootRegistry);
   }
}

package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;

public class YoRegistryBuffer
{
   private final YoRegistry rootRegistry;
   private final YoVariableBufferList yoVariableBuffers = new YoVariableBufferList();
   private final Map<String, YoVariableBuffer<?>> yoVariableFullnameToBufferMap = new HashMap<>();
   private final YoBufferPropertiesReadOnly properties;
   private final YoRegistryChangedListener registryBufferUpdater;

   /** The size of a single buffer frame in bytes. */
   private long registryMemorySize;

   private final ReentrantLock lock = new ReentrantLock();

   public YoRegistryBuffer(YoRegistry rootRegistry, YoBufferPropertiesReadOnly properties)
   {
      this.rootRegistry = rootRegistry;
      this.properties = properties;

      for (YoVariable yoVariable : rootRegistry.collectSubtreeVariables())
         registerNewYoVariable(yoVariable);

      registryBufferUpdater = (change) ->
      {
         if (change.wasVariableAdded())
            registerNewYoVariable(change.getTargetVariable());
         if (change.wasRegistryAdded())
            registerNewYoVariables(change.getTargetRegistry().collectSubtreeVariables());

         // A registry/variable removed on the backend side (e.g. a robot being replaced) must also drop its buffer
         // entry here - otherwise a same-named replacement variable is silently skipped by registerNewYoVariable's
         // "if the full name is already registered" guard, permanently reusing the old (now dead) variable's buffer.
         //
         // As with LinkedYoRegistry's equivalent fix: by the time this fires, the removed variable's own registry
         // link (and so its getNamespace()) is already cleared, so the full name must be rebuilt from
         // getTargetParentRegistry() (still attached, unaffected by the removal) instead of the removed
         // variable's/registry's own state.
         if (change.wasVariableRemoved())
         {
            YoRegistry parentRegistry = change.getTargetParentRegistry();
            if (parentRegistry != null)
               unregisterYoVariable(computeFullName(parentRegistry, change.getTargetVariable().getName()));
         }

         if (change.wasRegistryRemoved())
         {
            // The removed registry's own getNamespace() is just as unreliable here as a removed variable's (its
            // parent link was already cleared too) - and that unreliability would otherwise cascade to every
            // descendant's getNamespace() as well. So rebuild each descendant variable's full name structurally,
            // walking .getChildren()/.getVariables() (untouched by the removal) from the one known-safe namespace
            // (the still-attached parent's), instead of trusting .getNamespace() anywhere inside the removed subtree.
            YoRegistry parentRegistry = change.getTargetParentRegistry();
            if (parentRegistry != null)
               unregisterSubtree(change.getTargetRegistry(), parentRegistry.getNamespace().append(change.getTargetRegistry().getName()));
         }
      };

      this.rootRegistry.addListener(registryBufferUpdater);
   }

   private void registerNewYoVariables(Collection<? extends YoVariable> yoVariables)
   {
      for (YoVariable yoVariable : yoVariables)
      {
         registerNewYoVariable(yoVariable);
      }
   }

   private void registerNewYoVariable(YoVariable yoVariable)
   {
      String fullName = yoVariable.getFullNameString();

      if (yoVariableFullnameToBufferMap.containsKey(fullName))
         return;

      YoVariableBuffer<?> yoVariableBuffer = YoVariableBuffer.newYoVariableBuffer(yoVariable, properties);

      lock.lock();
      try
      {
         yoVariableBuffers.add(yoVariableBuffer);
         yoVariableFullnameToBufferMap.put(fullName, yoVariableBuffer);
         registryMemorySize += yoVariableBuffer.getVariableMemorySize();
      }
      finally
      {
         lock.unlock();
      }
   }

   private static String computeFullName(YoRegistry parentRegistry, String variableName)
   {
      return parentRegistry.getNamespace().append(variableName).getName();
   }

   private void unregisterSubtree(YoRegistry registry, YoNamespace registryNamespace)
   {
      for (YoVariable variable : registry.getVariables())
         unregisterYoVariable(registryNamespace.append(variable.getName()).getName());
      for (YoRegistry child : registry.getChildren())
         unregisterSubtree(child, registryNamespace.append(child.getName()));
   }

   /**
    * Only drops the full-name mapping - {@link YoVariableBufferList} is append-only (its {@code remove} methods all
    * throw {@link UnsupportedOperationException}), so the old buffer stays in {@link #yoVariableBuffers} and keeps
    * being read/written every tick, just no longer reachable by name. That's a harmless, bounded amount of waste
    * (one extra buffer per removed variable, not per tick) - the actual bug this fixes is a *same-named replacement*
    * variable being permanently skipped by {@link #registerNewYoVariable}'s "already registered" guard.
    */
   private void unregisterYoVariable(String fullName)
   {
      lock.lock();
      try
      {
         yoVariableFullnameToBufferMap.remove(fullName);
      }
      finally
      {
         lock.unlock();
      }
   }

   /**
    * Returns the size of the registry in bytes.
    * <p>
    * The size of the registry is the sum of the size of all the variables it contains. It can be used to estimate the memory size of a single frame in the
    * buffer.
    * </p>
    *
    * @return the size of the registry in bytes.
    */
   public long getRegistryMemorySize()
   {
      return registryMemorySize;
   }

   public void resizeBuffer(int from, int length)
   {
      yoVariableBuffers.resizeBuffer(from, length);
   }

   public void fillBuffer(boolean zeroFill, int from, int length)
   {
      yoVariableBuffers.fillBuffer(zeroFill, from, length);
   }

   public void writeBuffer()
   {
      writeBufferAt(properties.getCurrentIndex());
   }

   public void writeBufferAt(int index)
   {
      yoVariableBuffers.writeBufferAt(index);
   }

   public void readBuffer()
   {
      readBufferAt(properties.getCurrentIndex());
   }

   public void readBufferAt(int index)
   {
      yoVariableBuffers.readBufferAt(index);
   }

   public List<YoVariableBuffer<?>> getYoVariableBuffers()
   {
      return yoVariableBuffers;
   }

   public YoVariableBuffer<?> findYoVariableBuffer(YoVariable yoVariable)
   {
      return yoVariableFullnameToBufferMap.get(yoVariable.getFullNameString());
   }

   public YoVariableBuffer<?> findOrCreateYoVariableBuffer(YoVariable yoVariable)
   {
      String variableFullName = yoVariable.getFullNameString();
      YoVariableBuffer<?> yoVariableBuffer = yoVariableFullnameToBufferMap.get(variableFullName);

      if (yoVariableBuffer == null)
      {
         YoRegistry registry = SharedMemoryTools.ensurePathExists(rootRegistry, yoVariable.getNamespace());
         YoVariable duplicate = registry.getVariable(yoVariable.getName());
         if (duplicate == null)
            duplicate = yoVariable.duplicate(registry);

         yoVariableBuffer = YoVariableBuffer.newYoVariableBuffer(duplicate, properties);
         yoVariableBuffers.add(yoVariableBuffer);
         yoVariableFullnameToBufferMap.put(variableFullName, yoVariableBuffer);
      }

      return yoVariableBuffer;
   }

   LinkedYoRegistry newLinkedYoRegistry(YoRegistry registryToLink)
   {
      return new LinkedYoRegistry(registryToLink, this);
   }

   LinkedYoRegistry newLinkedYoRegistry()
   {
      return new LinkedYoRegistry(new YoRegistry(rootRegistry.getName()), this);
   }

   ReentrantLock getLock()
   {
      return lock;
   }

   public YoRegistry getRootRegistry()
   {
      return rootRegistry;
   }

   public YoBufferPropertiesReadOnly getProperties()
   {
      return properties;
   }

   public void dispose()
   {
      rootRegistry.removeListener(registryBufferUpdater);
      yoVariableBuffers.dispose();
      yoVariableFullnameToBufferMap.clear();
   }
}

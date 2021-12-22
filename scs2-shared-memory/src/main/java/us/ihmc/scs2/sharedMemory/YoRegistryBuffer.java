package us.ihmc.scs2.sharedMemory;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoRegistryBuffer
{
   private final YoRegistry rootRegistry;
   private final YoVariableBufferList yoVariableBuffers = new YoVariableBufferList();
   private final Map<String, YoVariableBuffer<?>> yoVariableFullnameToBufferMap = new HashMap<>();
   private final YoBufferPropertiesReadOnly properties;
   private final YoRegistryChangedListener registryBufferUpdater;

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
      }
      finally
      {
         lock.unlock();
      }
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

package us.ihmc.scs2.sharedMemory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

@SuppressWarnings({"rawtypes", "unchecked"})
public class LinkedYoRegistry extends LinkedBuffer
{
   private final YoRegistry rootRegistry;
   private final YoRegistryBuffer yoRegistryBuffer;

   private final ReentrantLock lock;
   private final LinkedBufferArray linkedYoVariables = new LinkedBufferArray();
   private final Map<YoVariable, LinkedYoVariable> linkedYoVariableMap = new HashMap<>();
   private final List<PushRequestListener> listeners = new ArrayList<>();
   private final PushRequestListener pushRequestForwarder = target -> listeners.forEach(listener -> listener.pushRequested(this));

   private YoRegistryChangedListener rootRegistryListener;
   private YoRegistryChangedListener bufferRootRegistryListener;
   private YoRegistry bufferRootRegistry;

   private boolean isDisposed = false;

   LinkedYoRegistry(YoRegistry rootRegistry, YoRegistryBuffer yoRegistryBuffer)
   {
      this.rootRegistry = rootRegistry;
      this.yoRegistryBuffer = yoRegistryBuffer;
      lock = yoRegistryBuffer.getLock();
      setup();
   }

   private void setup()
   {
      linkedYoVariables.addChangeListener(change ->
      {
         if (change.getTarget() instanceof LinkedYoVariable)
         {
            LinkedYoVariable<?> target = (LinkedYoVariable<?>) change.getTarget();
            if (change.wasLinkedBufferAdded())
               linkedYoVariableMap.put(target.getLinkedYoVariable(), target);
            if (change.wasLinkedBufferRemoved())
               linkedYoVariableMap.remove(target.getLinkedYoVariable());
         }
      });

      bufferRootRegistry = yoRegistryBuffer.getRootRegistry().findRegistry(rootRegistry.getNamespace());
      SharedMemoryTools.duplicateMissingYoVariablesInTarget(bufferRootRegistry, rootRegistry);

      bufferRootRegistryListener = change ->
      {
         lock.lock();
         try
         {
            if (change.wasVariableAdded())
            {
               YoVariable newBufferVariable1 = change.getTargetVariable();
               YoRegistry registry1 = SharedMemoryTools.ensurePathExists(rootRegistry, newBufferVariable1.getNamespace());
               if (registry1.getVariable(newBufferVariable1.getName()) == null)
                  newBufferVariable1.duplicate(registry1);
            }

            if (change.wasRegistryAdded())
            {
               for (YoVariable newBufferVariable2 : change.getTargetRegistry().collectSubtreeVariables())
               {
                  YoRegistry registry2 = SharedMemoryTools.ensurePathExists(rootRegistry, newBufferVariable2.getNamespace());
                  if (registry2.getVariable(newBufferVariable2.getName()) == null)
                     newBufferVariable2.duplicate(registry2);
               }
            }
         }
         finally
         {
            lock.unlock();
         }
      };
      bufferRootRegistry.addListener(bufferRootRegistryListener);

      rootRegistryListener = change ->
      {
         lock.lock();
         try
         {
            if (change.wasVariableAdded())
               yoRegistryBuffer.findOrCreateYoVariableBuffer(change.getTargetVariable());
            if (change.wasRegistryAdded())
               change.getTargetRegistry().collectSubtreeVariables().forEach(var -> yoRegistryBuffer.findOrCreateYoVariableBuffer(var));
         }
         finally
         {
            lock.unlock();
         }
      };
      rootRegistry.addListener(rootRegistryListener);
   }

   public <L extends LinkedYoVariable<T>, T extends YoVariable> L linkYoVariable(T variableToLink)
   {
      return linkYoVariable(variableToLink, null);
   }

   public <L extends LinkedYoVariable<T>, T extends YoVariable> L linkYoVariable(T variableToLink, Object initialUser)
   {
      if (isDisposed)
         return null;

      LinkedYoVariable linkedYoVariable = linkedYoVariableMap.get(variableToLink);

      if (linkedYoVariable == null)
      {
         YoVariableBuffer yoVariableBuffer = yoRegistryBuffer.findYoVariableBuffer(variableToLink);
         linkedYoVariable = yoVariableBuffer.newLinkedYoVariable(variableToLink, initialUser);
         linkedYoVariable.addPushRequestListener(pushRequestForwarder);
         linkedYoVariables.add(linkedYoVariable);
      }

      return (L) linkedYoVariable;
   }

   /** {@inheritDoc} */
   // Operation for the buffer consumers only.
   @Override
   public void push()
   {
      if (isDisposed)
         return;

      lock.lock();
      try
      {
         linkedYoVariables.push();
      }
      finally
      {
         lock.unlock();
      }
   }

   /** {@inheritDoc} */
   @Override
   public boolean pull()
   {
      if (isDisposed)
         return false;

      lock.lock();
      try
      {
         return linkedYoVariables.pull();
      }
      finally
      {
         lock.unlock();
      }
   }

   /** {@inheritDoc} */
   // Operation for the buffer manager only.
   @Override
   boolean processPush(boolean writeBuffer)
   {
      if (isDisposed)
         return false;

      lock.lock();
      try
      {
         return linkedYoVariables.processPush(writeBuffer);
      }
      finally
      {
         lock.unlock();
      }
   }

   /** {@inheritDoc} */
   // Operation for the buffer manager only.
   @Override
   void flushPush()
   {
      if (isDisposed)
         return;

      lock.lock();
      try
      {
         linkedYoVariables.flushPush();
      }
      finally
      {
         lock.unlock();
      }
   }

   @Override
   void addPushRequestListener(PushRequestListener listener)
   {
      if (isDisposed)
         return;

      listeners.add(listener);
   }

   @Override
   boolean removePushRequestListener(PushRequestListener listener)
   {
      if (isDisposed)
         return false;

      return listeners.remove(listener);
   }

   /** {@inheritDoc} */
   // Operation for the buffer manager only.
   @Override
   void prepareForPull()
   {
      if (isDisposed)
         return;

      lock.lock();
      try
      {
         linkedYoVariables.prepareForPull();
      }
      finally
      {
         lock.unlock();
      }
   }

   /** {@inheritDoc} */
   // Operation for the buffer manager only.
   @Override
   boolean hasRequestPending()
   {
      if (isDisposed)
         return false;

      lock.lock();

      try
      {
         return linkedYoVariables.hasRequestPending();
      }
      finally
      {
         lock.unlock();
      }
   }

   public YoRegistry getRootRegistry()
   {
      return rootRegistry;
   }

   @Override
   public void dispose()
   {
      if (isDisposed)
         return;

      isDisposed = true;
      linkedYoVariables.dispose();
      linkedYoVariableMap.clear();
      listeners.clear();
      rootRegistry.removeListener(rootRegistryListener);
      bufferRootRegistry.removeListener(bufferRootRegistryListener);
      rootRegistryListener = null;
      bufferRootRegistryListener = null;
      bufferRootRegistry = null;
   }
}

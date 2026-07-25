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
   // Assigned in the constructor (rather than here) since the lambda body reads the `lock` field, which isn't
   // assigned yet at this point in field-initializer order.
   private final PushRequestListener pushRequestForwarder;

   private YoRegistryChangedListener rootRegistryListener;
   private YoRegistryChangedListener bufferRootRegistryListener;
   private YoRegistry bufferRootRegistry;

   // Only ever read/written while holding `lock` - the lock's own happens-before guarantee is what makes dispose()
   // visible to every other method here, so this doesn't need to be volatile.
   private boolean isDisposed = false;

   LinkedYoRegistry(YoRegistry rootRegistry, YoRegistryBuffer yoRegistryBuffer)
   {
      this.rootRegistry = rootRegistry;
      this.yoRegistryBuffer = yoRegistryBuffer;
      lock = yoRegistryBuffer.getLock();
      pushRequestForwarder = target ->
      {
         lock.lock();
         try
         {
            listeners.forEach(listener -> listener.pushRequested(this));
         }
         finally
         {
            lock.unlock();
         }
      };

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

            // A registry/variable removed on the backend side (e.g. a robot being replaced) must also be removed
            // from this mirror - otherwise it lingers here forever, disconnected from any backend updates, and its
            // name permanently blocks a same-named replacement from ever being linked (the add-handling above only
            // duplicates a variable "if it doesn't already exist").
            //
            // Note: by the time this listener runs, the removed variable/registry has already been detached from
            // its parent (its own getNamespace()/getParent() are no longer reliable, and can even be null - e.g.
            // when a whole registry subtree is destroy()'d, each of its variables individually fires a "variable
            // removed" event from within that same detach). getTargetParentRegistry() is unaffected by this since
            // it refers to the (still attached) former parent, so look the mirror up from there instead.
            if (change.wasVariableRemoved())
            {
               YoRegistry backendParent = change.getTargetParentRegistry();
               YoRegistry mirrorParent = backendParent == null ? null : rootRegistry.findRegistry(backendParent.getNamespace());
               if (mirrorParent != null)
               {
                  YoVariable mirrorVariable = mirrorParent.getVariable(change.getTargetVariable().getName());
                  if (mirrorVariable != null)
                  {
                     unlinkYoVariable(mirrorVariable);
                     mirrorVariable.destroy();
                  }
               }
            }

            if (change.wasRegistryRemoved())
            {
               YoRegistry backendParent = change.getTargetParentRegistry();
               YoRegistry mirrorParent = backendParent == null ? null : rootRegistry.findRegistry(backendParent.getNamespace());
               YoRegistry mirrorRegistry = mirrorParent == null ? null : mirrorParent.getChild(change.getTargetRegistry().getName());
               if (mirrorRegistry != null)
               {
                  mirrorRegistry.collectSubtreeVariables().forEach(this::unlinkYoVariable);
                  mirrorParent.removeChild(mirrorRegistry);
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

   /**
    * Disposes and detaches the {@link LinkedYoVariable} (if any) previously created for {@code mirrorVariable} via
    * {@link #linkYoVariable}, so it stops being tracked (and pulled/pushed) once the variable itself is about to be
    * removed from this mirror.
    */
   private void unlinkYoVariable(YoVariable mirrorVariable)
   {
      LinkedYoVariable<?> linkedYoVariable = linkedYoVariableMap.get(mirrorVariable);
      if (linkedYoVariable != null)
      {
         linkedYoVariables.remove(linkedYoVariable);
         linkedYoVariable.dispose();
      }
   }

   public <L extends LinkedYoVariable<T>, T extends YoVariable> L linkYoVariable(T variableToLink)
   {
      return linkYoVariable(variableToLink, null);
   }

   public <L extends LinkedYoVariable<T>, T extends YoVariable> L linkYoVariable(T variableToLink, Object initialUser)
   {
      lock.lock();
      try
      {
         if (isDisposed)
            return null;

         LinkedYoVariable linkedYoVariable = linkedYoVariableMap.get(variableToLink);

         if (linkedYoVariable == null)
         {
            YoVariableBuffer yoVariableBuffer = yoRegistryBuffer.findYoVariableBuffer(variableToLink);
            // variableToLink can be a mirror variable that has since been destroy()'d (e.g. an old UI control still
            // holding a reference to a robot's variable across a reload/replace) - it no longer resolves to a buffer,
            // so there is nothing to link it to.
            if (yoVariableBuffer == null)
               return null;
            linkedYoVariable = yoVariableBuffer.newLinkedYoVariable(variableToLink, initialUser);
            linkedYoVariable.addPushRequestListener(pushRequestForwarder);
            linkedYoVariables.add(linkedYoVariable);
         }

         return (L) linkedYoVariable;
      }
      finally
      {
         lock.unlock();
      }
   }

   /** {@inheritDoc} */
   // Operation for the buffer consumers only.
   @Override
   public void push()
   {
      lock.lock();
      try
      {
         if (isDisposed)
            return;

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
      lock.lock();
      try
      {
         if (isDisposed)
            return false;
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
      lock.lock();
      try
      {
         if (isDisposed)
            return false;
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
      lock.lock();
      try
      {
         if (isDisposed)
            return;
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
      lock.lock();
      try
      {
         if (isDisposed)
            return;
         listeners.add(listener);
      }
      finally
      {
         lock.unlock();
      }
   }

   @Override
   boolean removePushRequestListener(PushRequestListener listener)
   {
      lock.lock();
      try
      {
         if (isDisposed)
            return false;
         return listeners.remove(listener);
      }
      finally
      {
         lock.unlock();
      }
   }

   /** {@inheritDoc} */
   // Operation for the buffer manager only.
   @Override
   void prepareForPull()
   {
      lock.lock();
      try
      {
         if (isDisposed)
            return;
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
      lock.lock();

      try
      {
         if (isDisposed)
            return false;
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
      lock.lock();
      try
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
      finally
      {
         lock.unlock();
      }
   }
}

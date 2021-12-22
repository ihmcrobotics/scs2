package us.ihmc.scs2.sharedMemory;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class LinkedYoVariable<T extends YoVariable> extends LinkedBuffer
{
   protected final T linkedYoVariable;
   protected final YoVariableBuffer<T> buffer;

   protected PushRequest<T> pushRequestToProcess;
   protected PullRequest<T> pullRequest;

   protected BufferSampleRequest bufferSampleRequest;
   @SuppressWarnings("rawtypes")
   protected BufferSample bufferSample;

   private final List<PushRequestListener> pushRequestListeners = new ArrayList<>();
   private final Set<Object> users = new HashSet<>();

   private boolean isDisposed = false;

   @SuppressWarnings({"rawtypes", "unchecked"})
   static LinkedYoVariable newLinkedYoVariable(YoVariable yoVariableToLink, YoVariableBuffer<?> buffer)
   {
      if (yoVariableToLink instanceof YoBoolean)
         return new LinkedYoBoolean((YoBoolean) yoVariableToLink, (YoBooleanBuffer) buffer);
      if (yoVariableToLink instanceof YoDouble)
         return new LinkedYoDouble((YoDouble) yoVariableToLink, (YoDoubleBuffer) buffer);
      if (yoVariableToLink instanceof YoInteger)
         return new LinkedYoInteger((YoInteger) yoVariableToLink, (YoIntegerBuffer) buffer);
      if (yoVariableToLink instanceof YoLong)
         return new LinkedYoLong((YoLong) yoVariableToLink, (YoLongBuffer) buffer);
      if (yoVariableToLink instanceof YoEnum)
         return new LinkedYoEnum<>((YoEnum) yoVariableToLink, (YoEnumBuffer) buffer);

      throw new UnsupportedOperationException("Unsupported YoVariable type: " + yoVariableToLink.getClass().getSimpleName());
   }

   LinkedYoVariable(T yoVariable, YoVariableBuffer<T> buffer)
   {
      Objects.requireNonNull(buffer, "Cannot create a linked YoVariable without a buffer.");

      this.linkedYoVariable = yoVariable;
      this.buffer = buffer;
   }

   public void requestEntireBuffer()
   {
      requestBufferWindow(-1, -1);
   }

   public void requestActiveBufferOnly()
   {
      requestBufferWindow(-1, -2);
   }

   public void requestBufferStartingFrom(int from)
   {
      requestBufferWindow(from, -1);
   }

   public void requestBufferWindow(int from, int length)
   {
      if (isDisposed)
         return;

      bufferSampleRequest = new BufferSampleRequest(from, length);
   }

   @Override
   void addPushRequestListener(PushRequestListener listener)
   {
      if (isDisposed)
         return;

      pushRequestListeners.add(listener);
   }

   @Override
   boolean removePushRequestListener(PushRequestListener listener)
   {
      return pushRequestListeners.remove(listener);
   }

   @Override
   public void push()
   {
      if (isDisposed)
         return;

      pushRequestToProcess = toPushRequest();
      pushRequestListeners.forEach(listener -> listener.pushRequested(this));
   }

   @Override
   boolean processPush(boolean writeBuffer)
   {
      if (isDisposed || pushRequestToProcess == null)
         return false;

      PushRequest<T> push = pushRequestToProcess;
      pushRequestToProcess = null;

      boolean modified = push.push();

      if (modified && writeBuffer)
         buffer.writeBuffer();

      return modified;
   }

   @Override
   void flushPush()
   {
      pushRequestToProcess = null;
   }

   @Override
   void prepareForPull()
   {
      if (isDisposed)
         return;

      pullRequest = toPullRequest();
      consumeBufferSampleRequest();
   }

   private void consumeBufferSampleRequest()
   {
      if (isDisposed || bufferSampleRequest == null)
         return;

      BufferSampleRequest localRequest = bufferSampleRequest;
      bufferSampleRequest = null;

      int from = localRequest.getFrom();
      int length = localRequest.getLength();

      YoBufferPropertiesReadOnly properties = buffer.getProperties();

      if (length > properties.getSize())
         return;

      if (from >= properties.getSize())
         return;

      if (length == -1)
      {
         if (from == -1)
         {
            from = 0;
            length = properties.getSize();
         }
         else if (from >= 0)
         {
            length = SharedMemoryTools.computeSubLength(from, properties.getOutPoint(), properties.getSize());
         }
      }
      else if (length == -2 && from == -1)
      {
         from = properties.getInPoint();
         length = properties.getActiveBufferLength();
      }

      if (length == 0)
         return;

      if (from < 0 || from >= properties.getSize() || length < 0 || length > properties.getSize())
         throw new IllegalArgumentException("Invalid request: from = " + from + ", length = " + length);

      bufferSample = buffer.copy(from, length, properties.copy());
   }

   @Override
   public boolean pull()
   {
      if (isDisposed)
         return false;

      PullRequest<T> pull = pullRequest;
      pullRequest = null;

      if (pull != null)
      {
         pull.pull();
      }

      return pull != null;
   }

   @Override
   public boolean hasRequestPending()
   {
      return bufferSampleRequest != null;
   }

   public boolean isRequestedBufferSampleAvailable()
   {
      return bufferSample != null;
   }

   @SuppressWarnings("rawtypes")
   public BufferSample pollRequestedBufferSample()
   {
      if (isDisposed)
         return null;

      BufferSample localSample = bufferSample;
      bufferSample = null;
      return localSample;
   }

   abstract PullRequest<T> toPullRequest();

   abstract PushRequest<T> toPushRequest();

   public T getLinkedYoVariable()
   {
      return linkedYoVariable;
   }

   YoVariableBuffer<T> getBuffer()
   {
      return buffer;
   }

   public void addUser(Object user)
   {
      users.add(user);
   }

   public boolean removeUser(Object user)
   {
      return users.remove(user);
   }

   @Override
   public boolean isActive()
   {
      return !isDisposed && !users.isEmpty();
   }

   @Override
   public void dispose()
   {
      if (isDisposed)
         return;

      isDisposed = true;
      pushRequestToProcess = null;
      pullRequest = null;
      bufferSampleRequest = null;
      bufferSample = null;
      pushRequestListeners.clear();
      users.clear();
   }
}
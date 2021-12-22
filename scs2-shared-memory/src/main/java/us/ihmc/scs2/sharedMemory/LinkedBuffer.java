package us.ihmc.scs2.sharedMemory;

public abstract class LinkedBuffer
{
   /**
    * Polls new data from the buffer, typically used to update a linked variable.
    * <p>
    * Operation for the buffer consumers only.
    * </p>
    * 
    * @return {@code true} if the pull actually changed data for the buffer consumer.
    */
   public abstract boolean pull();

   /**
    * Creates request for modifying the buffer when possible. This is typically used to push the value
    * of a linked {@code YoVariable} that has been changed in a buffer consumer thread.
    * <p>
    * Operation for the buffer consumers only.
    * </p>
    */
   public abstract void push();

   /**
    * Called by the buffer manager to apply requested changes from the linked variables and registries
    * in the buffer.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    * 
    * @param writeBuffer when {@code true} any actual modification is written in both at the current
    *                    reading index in the buffer and in the buffer's {@code YoVariable}. When
    *                    {@code false}, the value is pushed only to the buffer's {@code YoVariable},
    *                    the buffer remains unchanged.
    */
   abstract boolean processPush(boolean writeBuffer);

   /**
    * Discards any push request registered to this linked buffer.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    */
   abstract void flushPush();

   /**
    * Called by the buffer manager when wrapping writing operations on the buffers, this is where next
    * data available to the linked variables and registries is stored.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    */
   abstract void prepareForPull();

   /**
    * Tests whether a buffer consumer is still waiting for a buffer sample.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    * 
    * @return {@code true} if there is still a {@link BufferSampleRequest} awaiting to be processed,
    *         {@code false} otherwise.
    */
   abstract boolean hasRequestPending();

   boolean isActive()
   {
      return true;
   }

   void addPushRequestListener(PushRequestListener listener)
   {
   }

   boolean removePushRequestListener(PushRequestListener listener)
   {
      return false;
   }

   public abstract void dispose();

   public static interface PushRequestListener
   {
      void pushRequested(LinkedBuffer target);
   }
}

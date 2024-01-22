package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.LinkedYoVariableFactory;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferProcessor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Predicate;

/**
 * {@code YoSharedBuffer} allows to control read/write on {@code YoVariable} buffers of a session.
 * It is meant to be used by a single thread that is responsible for writing from the
 * {@code YoVariable}s into the internal buffers via {@link #writeBuffer()}, or reading from them to
 * update the {@code YoVariable}s.
 * <p>
 * For operations on the buffer from other threads, that is read/write or even modifying the list of
 * {@code YoVariable}s to record, this class can be shared via the interface
 * {@code LinkedYoVariableFactory}. The interface restricts the API to the creation of linked
 * registries, variable, or buffer properties. These linked elements represent a thread-safe barrier
 * with their own internal memory which allow to make read/write request on this
 * {@code YoSharedBuffer}. These requests will then be consumed on the owner thread of the
 * {@code YoSharedBuffer} and will trigger a notification to the linked elements, when needed, that
 * for instance new data is available.
 * </p>
 * <p>
 * The owner of the {@code YoSharedBuffer} is in the following named as the <i>buffer manager</i>
 * while the threads owning linked elements are referred to as <i>buffer consumers</i>.
 * </p>
 * <p>
 * Here are 3 typical use-case for a {@code YoSharedBuffer} from the buffer manager's perspective:
 * <ol>
 * <li>The buffer manager is computing data and storing it over time:
 *
 * <pre>
 * yoSharedBuffer.processLinkedPushRequests(); // Apply linked requests if desired, skip this to only give read-only access to the buffer consumers.
 * // Do calculation and write result in yoVariables
 * yoSharedBuffer.writeBuffer();
 * yoSharedBuffer.prepareLinkedBuffersForPull(); // Allow the buffer consumers to read the new data.
 * yoSharedBuffer.incrementBufferIndex(true); // Increment the current read/write index in the buffer.
 * </pre>
 *
 * <li>The buffer manager is playing back some pre-recorded data from the buffer:
 *
 * <pre>
 * yoSharedBuffer.readBuffer(); // Load values for the yoVariables from the buffer.
 * yoSharedBuffer.prepareLinkedBuffersForPull(); // Allow the buffer consumers to read the new data.
 * yoSharedBuffer.incrementBufferIndex(false, stepSizePerPlaybackTick); // Step forward in the buffer. stepSizePerPlaybackTick is typically greater than 0 to read forward, to play backward use a negative value.
 * </pre>
 *
 * <li>The buffer manager is paused, i.e. the buffer index is not changing, but it authorizes
 * modifications from the buffer consumers:
 *
 * <pre>
 * boolean isBufferModified = yoSharedBuffer.processLinkedPushRequests(); // Apply requested changes.
 * if (isBufferModified)
 *    yoSharedBuffer.writeBuffer(); // Write the changes in the buffer (they were only applied onto the yoVariables).
 * // The following assumes the buffer manager can handle requests for changing the current reading index in the buffer.
 * // This type of requests cannot be achieved via the linked buffer interface.
 * boolean hasConsumerChangedReadingIndex = yoSharedBuffer.setCurrentIndex(userRequestedIndex);
 * if (hasConsumerChangedReadingIndex)
 *    yoSharedBuffer.readBuffer(); // In case the reading index has been modified.
 * if (hasConsumerChangedReadingIndex || isBufferModified)
 *    yoSharedBuffer.prepareLinkedBuffersForPull(); // Make sure the buffer consumers have the updated data.
 * </pre>
 * </ol>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class YoSharedBuffer implements LinkedYoVariableFactory
{
   private final YoRegistryBuffer registryBuffer;

   private final ReentrantLock linkedBuffersLock = new ReentrantLock();
   private final LinkedBufferArray linkedBuffers = new LinkedBufferArray();
   private final ConcurrentLinkedQueue<LinkedBufferProperties> linkedBufferProperties = new ConcurrentLinkedQueue<>();

   private final YoBufferProperties properties = new YoBufferProperties();

   private boolean isDisposed = false;

   public YoSharedBuffer(YoRegistry rootRegistry, int initialBufferSize)
   {
      properties.setSize(initialBufferSize);
      registryBuffer = new YoRegistryBuffer(rootRegistry, properties);
   }

   /**
    * Consumes a request for cropping the size of the buffers, i.e. resizing the buffers to only keep
    * the part that is in between the {@code from} and {@code to} points as defined in the given
    * {@code request}.
    * <p>
    * This operation does not update the {@code YoVariable}'s value, to do so a call to
    * {@link #readBuffer()} is needed after calling this method in the case the index was successfully
    * set.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param request request defining the new size as well as the part of the buffer to preserve.
    */
   public void cropBuffer(CropBufferRequest request)
   {
      if (isDisposed)
         return;

      int newSize = request.getCroppedSize(properties.getSize());
      registryBuffer.resizeBuffer(request.getFrom(), newSize);
      properties.setSize(newSize);
      properties.setInPointIndex(0);
      properties.setOutPointIndex(newSize - 1);
      properties.setCurrentIndexUnsafe(0);
   }

   /**
    * Consumes a request for filling in the buffer with either zeros or the current value stored in
    * each {@link YoVariable}.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param request request defining the region of the buffer to fill in.
    */
   public void fillBuffer(FillBufferRequest request)
   {
      if (isDisposed)
         return;

      registryBuffer.fillBuffer(request.getZeroFill(), request.getFrom(), request.getFilledSize(properties.getSize()));
   }

   /**
    * Resize the buffer. This is typically used to increase its size.
    * <p>
    * When resizing, the buffer attempts to preserve the active part, i.e. data between the in and out
    * points, and the position of the current index relative to the in and out points.
    * </p>
    * <p>
    * This operation does not update the {@code YoVariable}'s value, to do so a call to
    * {@link #readBuffer()} is needed after calling this method in the case the buffer was successfully
    * resized.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param newSize the new size for this buffer.
    * @return {@code true} if the buffer was resized, {@code false} otherwise.
    */
   public boolean resizeBuffer(int newSize)
   {
      if (isDisposed)
         return false;

      if (newSize == properties.getSize() || newSize <= 0)
         return false;

      int oldCurrentIndex = properties.getCurrentIndex();
      int oldInPoint = properties.getInPoint();
      int oldOutPoint = properties.getOutPoint();

      int startBufferCopyIndex = oldInPoint;

      int oldSize = properties.getSize();

      int newOutPoint, newInPoint, newCurrentIndex;

      if (newSize < properties.getActiveBufferLength())
      { // Cannot keep all of the active buffer, keeping data closest to out-point.
         startBufferCopyIndex = SharedMemoryTools.computeFromIndex(oldOutPoint, newSize, oldSize);

         newInPoint = 0;
         newOutPoint = newSize - 1;
         if (SharedMemoryTools.isInsideBounds(oldCurrentIndex, SharedMemoryTools.computeFromIndex(oldOutPoint, newSize, oldSize), oldOutPoint, oldSize))
            newCurrentIndex = newOutPoint - SharedMemoryTools.computeSubLength(oldCurrentIndex, oldOutPoint, oldSize) + 1;
         else
            newCurrentIndex = newOutPoint;
      }
      else
      {
         newInPoint = 0;
         newOutPoint = properties.getActiveBufferLength() - 1;
         if (properties.isIndexBetweenBounds(oldCurrentIndex))
            newCurrentIndex = SharedMemoryTools.computeSubLength(oldInPoint, oldCurrentIndex, oldSize) - 1;
         else
            newCurrentIndex = newOutPoint;
      }

      registryBuffer.resizeBuffer(startBufferCopyIndex, newSize);

      properties.setSize(newSize);
      properties.setInPointIndex(newInPoint);
      properties.setOutPointIndex(newOutPoint);
      properties.setCurrentIndexUnsafe(newCurrentIndex);
      return true;
   }

   /**
    * Returns the size of a single frame in the buffer in bytes.
    *
    * @return the size of a single frame in the buffer in bytes.
    */
   public long getSingleBufferFrameMemorySize()
   {
      return registryBuffer.getRegistryMemorySize();
   }

   /**
    * Changes the current position in the buffer.
    * <p>
    * This operation does not update the {@code YoVariable}'s value, to do so a call to
    * {@link #readBuffer()} is needed after calling this method in the case the index was successfully
    * set.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param newIndex the new position in the buffer.
    * @return {@code true} if the index was changed successfully, {@code false} otherwise.
    */
   public boolean setCurrentIndex(int newIndex)
   {
      return properties.setCurrentIndex(newIndex);
   }

   /**
    * Changes the in-point index.
    * <p>
    * The in and out points are used to define a interval of interest in the buffer. They usually start
    * at zero and as data is being recorded into the buffer, the out point is being pushed such that
    * the interval represents the set of indices for which the data has been written.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param newInPoint the new index for the in point.
    * @return {@code true} if the index has changed, {@code false} otherwise.
    */
   public boolean setInPoint(int newInPoint)
   {
      return properties.setInPointIndex(newInPoint);
   }

   /**
    * Changes the out-point index.
    * <p>
    * The in and out points are used to define a interval of interest in the buffer. They usually start
    * at zero and as data is being recorded into the buffer, the out point is being pushed such that
    * the interval represents the set of indices for which the data has been written.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param newOutPoint the new index for the out point.
    * @return {@code true} if the index has changed, {@code false} otherwise.
    */
   public boolean setOutPoint(int newOutPoint)
   {
      return properties.setOutPointIndex(newOutPoint);
   }

   /**
    * Applies the changes requested from the difference linked variables and registries to the buffers.
    * <p>
    * If {@code writeBuffer == false} and a change of value of a {@code YoVariable} is requested, it is
    * applied to the {@code YoVariable} not the buffer. So when this operation results in a actual
    * modification, i.e. this methods returns {@code true}, the buffers need to be updated by calling
    * {@link #writeBuffer()}.
    * </p>
    * <p>
    * If {@code writeBuffer == true} any change will be applied to both the corresponding
    * {@code YoVariable} and buffer, no need to call {@link #writeBuffer()}.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param writeBuffer when {@code true} any actual modification is written in both at the current
    *                    reading index in the buffer and in the buffer's {@code YoVariable}. When
    *                    {@code false}, the value is pushed only to the buffer's {@code YoVariable},
    *                    the buffer remains unchanged.
    * @return {@code true} if this operation actually resulted in at least one modification,
    *       {@code false} otherwise.
    */
   public boolean processLinkedPushRequests(boolean writeBuffer)
   {
      if (isDisposed)
         return false;

      linkedBuffersLock.lock();
      try
      {
         return linkedBuffers.processPush(writeBuffer);
      }
      finally
      {
         linkedBuffersLock.unlock();
      }
   }

   /**
    * Discards all push requested from buffer consumers for all the internal buffer.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    */
   public void flushLinkedPushRequests()
   {
      if (isDisposed)
         return;

      linkedBuffersLock.lock();
      try
      {
         linkedBuffers.flushPush();
      }
      finally
      {
         linkedBuffersLock.unlock();
      }
   }

   /**
    * Reads the buffers at the current index, i.e. {@code properties.getCurrentIndex()}, and update
    * their respective {@code YoVariable}.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    */
   public void readBuffer()
   {
      if (isDisposed)
         return;

      registryBuffer.readBuffer();
   }

   /**
    * Writes into the buffers at the current index, i.e. {@code properties.getCurrentIndex()}, the
    * value of their respective {@code YoVariable}.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    */
   public void writeBuffer()
   {
      if (isDisposed)
         return;

      registryBuffer.writeBuffer();
   }

   /**
    * Packs the values of this buffer's {@code YoVariable}s to be available to the linked variables.
    * <p>
    * Operation for the buffer manager only.
    * </p>
    */
   public void prepareLinkedBuffersForPull()
   {
      if (isDisposed)
         return;

      linkedBuffersLock.lock();
      try
      {
         linkedBuffers.prepareForPull();
      }
      finally
      {
         linkedBuffersLock.unlock();
      }

      linkedBufferProperties.forEach(LinkedBufferProperties::prepareForPull);
   }

   /**
    * Indicates whether a buffer consumer is still awaiting for a response.
    * <p>
    * When a buffer consumer is awaiting for response, {@link #prepareLinkedBuffersForPull()} should be
    * called to resolve the request.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @return {@code true} if at least one request is still pending, {@code false} otherwise.
    */
   public boolean hasRequestPending()
   {
      if (isDisposed)
         return false;

      linkedBuffersLock.lock();
      try
      {
         return linkedBuffers.hasRequestPending();
      }
      finally
      {
         linkedBuffersLock.unlock();
      }
   }

   /**
    * Increments the position of the current read index by 1.
    * <p>
    * The two following scenarios should be considered:
    * <ol>
    * <li>The buffer manager is currently recording data into the buffer. In such case,
    * {@code updateBufferBounds} should be {@code true}, allowing the reading index to push the
    * out-point back.
    * <li>The buffer manager is currently playing back data from the buffer. In such case,
    * {@code updateBufferBounds} should be {@code false}, when the current reading index is about to
    * pass the out-point, it is sent to the in-point causing the playback to cycle through the data
    * that is in between the in and out points.
    * </ol>
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param updateBufferBounds when {@code true} the out-point will automatically be pushed back by
    *                           the current index. Usually {@code true} when the buffer manager is
    *                           writing into the buffer and usually {@code false} when data is being
    *                           played back from the buffer.
    * @return the new position of the reading index.
    */
   public int incrementBufferIndex(boolean updateBufferBounds)
   {
      return properties.incrementIndex(updateBufferBounds);
   }

   /**
    * Increments the position of the current read index by {@code stepSize}.
    * <p>
    * The two following scenarios should be considered:
    * <ol>
    * <li>The buffer manager is currently recording data into the buffer. In such case,
    * {@code updateBufferBounds} should be {@code true}, allowing the reading index to push the
    * out-point back.
    * <li>The buffer manager is currently playing back data from the buffer. In such case,
    * {@code updateBufferBounds} should be {@code false}, when the current reading index is about to
    * pass the out-point, it is sent to the in-point causing the playback to cycle through the data
    * that is in between the in and out points.
    * </ol>
    * </p>
    * <p>
    * Note that if the given {@code stepSize} is negative, the reading position index is decremented
    * instead.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param updateBufferBounds when {@code true} the out-point will automatically be pushed back by
    *                           the current index. Usually {@code true} when the buffer manager is
    *                           writing into the buffer and usually {@code false} when data is being
    *                           played back from the buffer.
    * @param stepSize           the size of the increment to apply to the reading position.
    * @return the new position of the reading index.
    */
   public int incrementBufferIndex(boolean updateBufferBounds, int stepSize)
   {
      return properties.incrementIndex(updateBufferBounds, stepSize);
   }

   /**
    * Decrements the position of the current read index by {@code stepSize}.
    * <p>
    * This methods is usually used when playing back data from the buffer in reverse.
    * </ol>
    * </p>
    * <p>
    * Note that if the given {@code stepSize} is negative, the reading position index is incremented
    * instead.
    * </p>
    * <p>
    * Operation for the buffer manager only.
    * </p>
    *
    * @param stepSize the size of the decrement to apply to the reading position.
    * @return the new position of the reading index.
    */
   public int decrementBufferIndex(int stepSize)
   {
      return properties.decrementIndex(stepSize);
   }

   /**
    * Applies a function to the buffer from the in-point to the out-point.
    *
    * @param processor the function to apply to the buffer.
    */
   public void applyProcessor(YoBufferProcessor processor)
   {
      if (isDisposed)
         return;

      linkedBuffersLock.lock();
      try
      {
         int initialIndex = properties.getCurrentIndex();
         int length = properties.getActiveBufferLength();
         processor.initialize(getRootRegistry());

         if (processor.goForward())
         {
            int startIndex = properties.getInPoint();
            int endIndex = properties.getOutPoint();
            setCurrentIndex(startIndex);

            for (int i = 0; i < length; i++)
            {
               readBuffer();
               processor.process(startIndex, endIndex, properties.getCurrentIndex());
               writeBuffer();
               incrementBufferIndex(false);
            }
         }
         else
         {
            int startIndex = properties.getOutPoint();
            int endIndex = properties.getInPoint();
            setCurrentIndex(startIndex);

            for (int i = 0; i < length; i++)
            {
               readBuffer();
               processor.process(startIndex, endIndex, properties.getCurrentIndex());
               writeBuffer();
               decrementBufferIndex(1);
            }
         }

         setCurrentIndex(initialIndex);
         readBuffer();
      }
      finally
      {
         linkedBuffersLock.unlock();
      }
   }

   /**
    * Gets the read-only reference to this buffer properties.
    * <p>
    * Properties include information about the current reading index, in point, out point, and buffer
    * size.
    * </p>
    *
    * @return the properties of this buffer.
    */
   public YoBufferPropertiesReadOnly getProperties()
   {
      return properties;
   }

   /**
    * Gets the root registry this buffer was created for.
    *
    * @return the root registry.
    */
   public YoRegistry getRootRegistry()
   {
      return registryBuffer.getRootRegistry();
   }

   /**
    * Gets the internal buffer holder.
    *
    * @return the root registry buffer listing the buffer for every {@code YoVariable}.
    */
   public YoRegistryBuffer getRegistryBuffer()
   {
      return registryBuffer;
   }

   /**
    * Writes all the yoVariable buffers to the given {@code outputStream} using a non-compressed format
    * where each variable is stored as on line.
    *
    * @param outputStream the stream to write to.
    */
   public void exportDataASCII(OutputStream outputStream)
   {
      exportDataASCII(outputStream, null, null);
   }

   /**
    * Writes all the yoVariable buffers to the given {@code outputStream} using a non-compressed format
    * where each variable is stored as one line.
    *
    * @param outputStream   the stream to write to.
    * @param variableFilter a filter to downselect the {@link YoVariable}s to be exported. A
    *                       {@link YoVariable} is exported if the given predicate returns {@code true}.
    *                       Can be {@code null}, if so all variable are exported.
    * @param registryFilter a filter to downselect the {@link YoRegistry}s to be exported. A
    *                       {@link YoRegistry} and its descendants are skipped if the predicate returns
    *                       {@code false}.
    */
   public void exportDataASCII(OutputStream outputStream, Predicate<YoVariable> variableFilter, Predicate<YoRegistry> registryFilter)
   {
      SharedMemoryIOTools.exportDataASCII(this, outputStream, variableFilter, registryFilter);
   }

   /**
    * Writes all the yoVariable buffers to the given {@code outputStream} using a non-compressed format
    * where each variable is stored as one column and columns are separated by a comma.
    *
    * @param outputStream the stream to write to.
    */
   public void exportDataCSV(OutputStream outputStream)
   {
      exportDataCSV(outputStream, null, null);
   }

   /**
    * Writes all the yoVariable buffers to the given {@code outputStream} using a non-compressed format
    * where each variable is stored as one column and columns are separated by a comma.
    *
    * @param outputStream   the stream to write to.
    * @param variableFilter a filter to downselect the {@link YoVariable}s to be exported. A
    *                       {@link YoVariable} is exported if the given predicate returns {@code true}.
    *                       Can be {@code null}, if so all variable are exported.
    * @param registryFilter a filter to downselect the {@link YoRegistry}s to be exported. A
    *                       {@link YoRegistry} and its descendants are skipped if the predicate returns
    *                       {@code false}.
    */
   public void exportDataCSV(OutputStream outputStream, Predicate<YoVariable> variableFilter, Predicate<YoRegistry> registryFilter)
   {
      SharedMemoryIOTools.exportDataCSV(this, outputStream, variableFilter, registryFilter);
   }

   /**
    * Writes all the yoVariable buffers to the given {@code file} using Matlab data structure and file
    * format.
    *
    * @param file the file to write to.
    */
   public void exportDataMatlab(File file) throws IOException
   {
      exportDataMatlab(file, null, null);
   }

   /**
    * Writes all the yoVariable buffers to the given {@code file} using Matlab data structure and file
    * format.
    *
    * @param file           the file to write to.
    * @param variableFilter a filter to downselect the {@link YoVariable}s to be exported. A
    *                       {@link YoVariable} is exported if the given predicate returns {@code true}.
    *                       Can be {@code null}, if so all variable are exported.
    * @param registryFilter a filter to downselect the {@link YoRegistry}s to be exported. A
    *                       {@link YoRegistry} and its descendants are skipped if the predicate returns
    *                       {@code false}.
    */
   public void exportDataMatlab(File file, Predicate<YoVariable> variableFilter, Predicate<YoRegistry> registryFilter) throws IOException
   {
      SharedMemoryIOTools.exportDataMatlab(this, file, variableFilter, registryFilter);
   }

   public void dispose()
   {
      if (isDisposed)
         return;

      try
      {
         linkedBuffersLock.lock();
         isDisposed = true;
         registryBuffer.dispose();
         linkedBuffers.dispose();
         linkedBufferProperties.forEach(l -> l.dispose());
         linkedBufferProperties.clear();
      }
      finally
      {
         linkedBuffersLock.unlock();
      }
   }

   @Override
   public LinkedYoRegistry newLinkedYoRegistry(YoRegistry registryToLink)
   {
      if (isDisposed)
         return null;

      LinkedYoRegistry linkedYoRegistry = registryBuffer.newLinkedYoRegistry(registryToLink);
      linkedBuffersLock.lock();
      try
      {
         linkedBuffers.add(linkedYoRegistry);
      }
      finally
      {
         linkedBuffersLock.unlock();
      }
      return linkedYoRegistry;
   }

   @Override
   public LinkedYoRegistry newLinkedYoRegistry()
   {
      if (isDisposed)
         return null;

      LinkedYoRegistry linkedYoRegistry = registryBuffer.newLinkedYoRegistry();
      linkedBuffersLock.lock();
      try
      {
         linkedBuffers.add(linkedYoRegistry);
      }
      finally
      {
         linkedBuffersLock.unlock();
      }
      return linkedYoRegistry;
   }

   @Override
   public LinkedBufferProperties newLinkedBufferProperties()
   {
      if (isDisposed)
         return null;

      LinkedBufferProperties linkedBufferProperties = new LinkedBufferProperties(properties);
      this.linkedBufferProperties.add(linkedBufferProperties);
      return linkedBufferProperties;
   }

   @Override
   public LinkedYoVariable<?> newLinkedYoVariable(YoVariable variableToLink, Object initialUser)
   {
      if (isDisposed)
         return null;

      LinkedYoVariable<?> linkedYoVariable = LinkedYoVariable.newLinkedYoVariable(variableToLink,
                                                                                  registryBuffer.findYoVariableBuffer(variableToLink),
                                                                                  initialUser);
      linkedBuffersLock.lock();
      try
      {
         linkedBuffers.add(linkedYoVariable);
      }
      finally
      {
         linkedBuffersLock.unlock();
      }
      return linkedYoVariable;
   }
}

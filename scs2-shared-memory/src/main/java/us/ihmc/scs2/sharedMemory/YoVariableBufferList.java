package us.ihmc.scs2.sharedMemory;

import java.util.AbstractList;
import java.util.Arrays;
import java.util.Collection;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class YoVariableBufferList extends AbstractList<YoVariableBuffer<?>>
{
   private int size = 0;
   private YoVariableBuffer<?>[] yoVariableBuffers = new YoVariableBuffer[8];

   /*
    * writeBufferAt(int) is called every tick for every registered variable, so its call site needs to stay
    * monomorphic: iterating the single yoVariableBuffers array (a mix of 5 concrete YoVariableBuffer subtypes)
    * forces a megamorphic virtual dispatch per element that the JIT cannot inline. These type-segregated
    * buckets let writeBufferAt/readBufferAt loop over one concrete type at a time instead.
    */
   private YoDoubleBuffer[] doubleBuffers = new YoDoubleBuffer[0];
   private YoBooleanBuffer[] booleanBuffers = new YoBooleanBuffer[0];
   private YoIntegerBuffer[] integerBuffers = new YoIntegerBuffer[0];
   private YoLongBuffer[] longBuffers = new YoLongBuffer[0];
   private YoEnumBuffer<?>[] enumBuffers = new YoEnumBuffer[0];
   private int doubleSize = 0, booleanSize = 0, integerSize = 0, longSize = 0, enumSize = 0;

   /*
    * Dedicated, statically-partitioned worker pool for writeBufferAt, sized off Runtime.availableProcessors()
    * so it adapts to whatever machine this runs on. This replaces a per-call Arrays.stream(...).parallel(),
    * which re-splits the array via the (contended, shared) common ForkJoinPool on every single tick - overkill
    * given each element's work is a single array store.
    */
   private ExecutorService workerPool;
   private WriteWorker[] writeWorkers = new WriteWorker[0];
   private Future<?>[] writeFutures = new Future<?>[0];
   private volatile int writeIndex;

   @Override
   public int size()
   {
      return size;
   }

   @Override
   public YoVariableBuffer<?>[] toArray()
   {
      return Arrays.copyOf(yoVariableBuffers, size);
   }

   @Override
   public boolean add(YoVariableBuffer<?> e)
   {
      size++;
      ensureCapacity(size);
      yoVariableBuffers[size - 1] = e;
      addToTypedBucket(e);
      rebuildWriteWorkers();
      return true;
   }

   private void addToTypedBucket(YoVariableBuffer<?> e)
   {
      if (e instanceof YoDoubleBuffer b)
      {
         doubleBuffers = ensureTypedCapacity(doubleBuffers, doubleSize + 1);
         doubleBuffers[doubleSize++] = b;
      }
      else if (e instanceof YoBooleanBuffer b)
      {
         booleanBuffers = ensureTypedCapacity(booleanBuffers, booleanSize + 1);
         booleanBuffers[booleanSize++] = b;
      }
      else if (e instanceof YoIntegerBuffer b)
      {
         integerBuffers = ensureTypedCapacity(integerBuffers, integerSize + 1);
         integerBuffers[integerSize++] = b;
      }
      else if (e instanceof YoLongBuffer b)
      {
         longBuffers = ensureTypedCapacity(longBuffers, longSize + 1);
         longBuffers[longSize++] = b;
      }
      else if (e instanceof YoEnumBuffer<?> b)
      {
         enumBuffers = ensureTypedCapacity(enumBuffers, enumSize + 1);
         enumBuffers[enumSize++] = b;
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported buffer type: " + e.getClass().getSimpleName());
      }
   }

   private static YoDoubleBuffer[] ensureTypedCapacity(YoDoubleBuffer[] array, int minCapacity)
   {
      return minCapacity <= array.length ? array : Arrays.copyOf(array, growTypedCapacity(array.length, minCapacity));
   }

   private static YoBooleanBuffer[] ensureTypedCapacity(YoBooleanBuffer[] array, int minCapacity)
   {
      return minCapacity <= array.length ? array : Arrays.copyOf(array, growTypedCapacity(array.length, minCapacity));
   }

   private static YoIntegerBuffer[] ensureTypedCapacity(YoIntegerBuffer[] array, int minCapacity)
   {
      return minCapacity <= array.length ? array : Arrays.copyOf(array, growTypedCapacity(array.length, minCapacity));
   }

   private static YoLongBuffer[] ensureTypedCapacity(YoLongBuffer[] array, int minCapacity)
   {
      return minCapacity <= array.length ? array : Arrays.copyOf(array, growTypedCapacity(array.length, minCapacity));
   }

   private static YoEnumBuffer<?>[] ensureTypedCapacity(YoEnumBuffer<?>[] array, int minCapacity)
   {
      return minCapacity <= array.length ? array : Arrays.copyOf(array, growTypedCapacity(array.length, minCapacity));
   }

   private static int growTypedCapacity(int oldCapacity, int minCapacity)
   {
      int newCapacity = oldCapacity == 0 ? 8 : oldCapacity + (oldCapacity >> 1);
      return Math.max(newCapacity, minCapacity);
   }

   @Override
   public YoVariableBuffer<?> get(int index)
   {
      return yoVariableBuffers[index];
   }

   public void resizeBuffer(int from, int length)
   {
      for (int i = 0; i < size; i++)
      {
         yoVariableBuffers[i].resizeBuffer(from, length);
      }
   }

   public void fillBuffer(boolean zeroFill, int from, int length)
   {
      if (length <= 0)
         return;

      for (int i = 0; i < size; i++)
      {
         yoVariableBuffers[i].fillBuffer(zeroFill, from, length);
      }
   }

   public void writeBufferAt(int index)
   {
      if (writeWorkers.length <= 1)
      {
         // Small registry, or no multi-core benefit available: skip the pool entirely.
         for (int i = 0; i < doubleSize; i++)
            doubleBuffers[i].writeBufferAt(index);
         for (int i = 0; i < booleanSize; i++)
            booleanBuffers[i].writeBufferAt(index);
         for (int i = 0; i < integerSize; i++)
            integerBuffers[i].writeBufferAt(index);
         for (int i = 0; i < longSize; i++)
            longBuffers[i].writeBufferAt(index);
         for (int i = 0; i < enumSize; i++)
            enumBuffers[i].writeBufferAt(index);
         return;
      }

      writeIndex = index;

      for (int i = 0; i < writeWorkers.length; i++)
         writeFutures[i] = workerPool.submit(writeWorkers[i]);

      try
      {
         for (Future<?> future : writeFutures)
            future.get();
      }
      catch (InterruptedException e)
      {
         Thread.currentThread().interrupt();
         throw new RuntimeException(e);
      }
      catch (ExecutionException e)
      {
         throw new RuntimeException(e.getCause());
      }
   }

   public void readBufferAt(int index)
   {
      for (int i = 0; i < size; i++)
      {
         yoVariableBuffers[i].readBufferAt(index);
      }
   }

   /**
    * Rebuilds the fixed worker pool (sized off the host's available cores, not a hardcoded number so this
    * scales across whatever machine it runs on) and re-partitions the typed buckets evenly across workers.
    * Only called from {@link #add(YoVariableBuffer)}, so this cost is paid when a variable is registered, not
    * per tick.
    */
   private void rebuildWriteWorkers()
   {
      int workerCount = Math.max(1, Runtime.getRuntime().availableProcessors() - 1);

      if (workerPool == null && workerCount > 1)
      {
         workerPool = Executors.newFixedThreadPool(workerCount, runnable ->
         {
            Thread thread = new Thread(runnable, "YoVariableBufferList-writer");
            thread.setDaemon(true);
            return thread;
         });
      }

      if (workerCount <= 1)
      {
         writeWorkers = new WriteWorker[0];
         writeFutures = new Future<?>[0];
         return;
      }

      WriteWorker[] workers = new WriteWorker[workerCount];

      for (int w = 0; w < workerCount; w++)
      {
         WriteWorker worker = new WriteWorker();
         worker.doubleFrom = partitionBound(doubleSize, workerCount, w);
         worker.doubleTo = partitionBound(doubleSize, workerCount, w + 1);
         worker.booleanFrom = partitionBound(booleanSize, workerCount, w);
         worker.booleanTo = partitionBound(booleanSize, workerCount, w + 1);
         worker.integerFrom = partitionBound(integerSize, workerCount, w);
         worker.integerTo = partitionBound(integerSize, workerCount, w + 1);
         worker.longFrom = partitionBound(longSize, workerCount, w);
         worker.longTo = partitionBound(longSize, workerCount, w + 1);
         worker.enumFrom = partitionBound(enumSize, workerCount, w);
         worker.enumTo = partitionBound(enumSize, workerCount, w + 1);
         workers[w] = worker;
      }

      writeWorkers = workers;
      writeFutures = new Future<?>[workerCount];
   }

   private static int partitionBound(int total, int parts, int part)
   {
      return (int) ((long) total * part / parts);
   }

   private final class WriteWorker implements Runnable
   {
      int doubleFrom, doubleTo;
      int booleanFrom, booleanTo;
      int integerFrom, integerTo;
      int longFrom, longTo;
      int enumFrom, enumTo;

      @Override
      public void run()
      {
         int index = writeIndex;

         YoDoubleBuffer[] doubles = doubleBuffers;
         for (int i = doubleFrom; i < doubleTo; i++)
            doubles[i].writeBufferAt(index);

         YoBooleanBuffer[] booleans = booleanBuffers;
         for (int i = booleanFrom; i < booleanTo; i++)
            booleans[i].writeBufferAt(index);

         YoIntegerBuffer[] integers = integerBuffers;
         for (int i = integerFrom; i < integerTo; i++)
            integers[i].writeBufferAt(index);

         YoLongBuffer[] longs = longBuffers;
         for (int i = longFrom; i < longTo; i++)
            longs[i].writeBufferAt(index);

         YoEnumBuffer<?>[] enums = enumBuffers;
         for (int i = enumFrom; i < enumTo; i++)
            enums[i].writeBufferAt(index);
      }
   }

   public void dispose()
   {
      for (int i = 0; i < yoVariableBuffers.length; i++)
      {
         if (yoVariableBuffers[i] != null)
         {
            yoVariableBuffers[i].dispose();
            yoVariableBuffers[i] = null;
         }
      }
      yoVariableBuffers = null;
      doubleBuffers = null;
      booleanBuffers = null;
      integerBuffers = null;
      longBuffers = null;
      enumBuffers = null;
      writeWorkers = new WriteWorker[0];
      writeFutures = new Future<?>[0];

      if (workerPool != null)
      {
         workerPool.shutdown();
         workerPool = null;
      }
   }

   @Override
   public boolean remove(Object o)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean addAll(Collection<? extends YoVariableBuffer<?>> c)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean addAll(int index, Collection<? extends YoVariableBuffer<?>> c)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean removeAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean retainAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void clear()
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public YoVariableBuffer<?> set(int index, YoVariableBuffer<?> element)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void add(int index, YoVariableBuffer<?> element)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public YoVariableBuffer<?> remove(int index)
   {
      throw new UnsupportedOperationException();
   }

   protected void rangeCheck(int index)
   {
      if (index >= size)
         throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size);
      positiveIndexCheck(index);
   }

   protected void positiveIndexCheck(int index)
   {
      if (index < 0)
         throw new IndexOutOfBoundsException("Index cannot be negative: " + index);
   }

   private static final int MAX_ARRAY_SIZE = Integer.MAX_VALUE - 8;

   protected void ensureCapacity(int minCapacity)
   {
      if (minCapacity <= yoVariableBuffers.length)
         return;

      int previousArraySize = yoVariableBuffers.length;
      int newArraySize = previousArraySize + (previousArraySize >> 1);
      if (newArraySize - minCapacity < 0)
         newArraySize = minCapacity;
      if (newArraySize - MAX_ARRAY_SIZE > 0)
         newArraySize = checkWithMaxCapacity(minCapacity);

      yoVariableBuffers = Arrays.copyOf(yoVariableBuffers, newArraySize);
   }

   private static int checkWithMaxCapacity(int minCapacity)
   {
      if (minCapacity < 0) // overflow
         throw new OutOfMemoryError();
      return (minCapacity > MAX_ARRAY_SIZE) ? Integer.MAX_VALUE : MAX_ARRAY_SIZE;
   }
}

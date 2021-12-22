package us.ihmc.scs2.sharedMemory;

import java.util.AbstractList;
import java.util.Arrays;
import java.util.Collection;

public class YoVariableBufferList extends AbstractList<YoVariableBuffer<?>>
{
   private int size = 0;
   private YoVariableBuffer<?>[] yoVariableBuffers = new YoVariableBuffer[8];

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
      return true;
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
      Arrays.stream(yoVariableBuffers, 0, size).parallel().forEach(buffer -> buffer.writeBufferAt(index));
//      for (int i = 0; i < size; i++)
//      {
//         yoVariableBuffers[i].writeBufferAt(index);
//      }
   }

   public void readBufferAt(int index)
   {
      for (int i = 0; i < size; i++)
      {
         yoVariableBuffers[i].readBufferAt(index);
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

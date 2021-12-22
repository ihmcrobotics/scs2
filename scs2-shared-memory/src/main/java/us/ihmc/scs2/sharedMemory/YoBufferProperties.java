package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

public class YoBufferProperties implements YoBufferPropertiesReadOnly
{
   private int inPoint = 0;
   private int outPoint = 0;

   private int currentIndex = 0;
   private int size = 0;

   public YoBufferProperties()
   {
   }

   public YoBufferProperties(int index, int size)
   {
      setSize(size);
      setCurrentIndexUnsafe(index);
   }

   public YoBufferProperties(YoBufferPropertiesReadOnly other)
   {
      set(other);
   }

   @Override
   public int getSize()
   {
      return size;
   }

   @Override
   public int getCurrentIndex()
   {
      return currentIndex;
   }

   @Override
   public int getInPoint()
   {
      return inPoint;
   }

   @Override
   public int getOutPoint()
   {
      return outPoint;
   }

   public int incrementIndex(boolean updateBufferBounds)
   {
      currentIndex = SharedMemoryTools.increment(currentIndex, 1, getSize());

      if (updateBufferBounds)
      {
         // The out-point is always at the last writing index.
         outPoint = currentIndex;

         if (outPoint == inPoint)
         { // Push the in-point
            inPoint = SharedMemoryTools.increment(inPoint, 1, getSize());
         }
      }
      else if (!isIndexBetweenBounds(currentIndex))
      { // The current index has to remain within the bounds when reading.
         currentIndex = inPoint;
      }
      return currentIndex;
   }

   public int decrementIndex()
   {
      currentIndex = SharedMemoryTools.decrement(currentIndex, 1, getSize());

      if (!isIndexBetweenBounds(currentIndex))
      { // The current index has to remain within the bounds when reading.
         currentIndex = outPoint;
      }
      return currentIndex;
   }

   public int incrementIndex(boolean updateBufferBounds, int stepSize)
   {
      if (stepSize < 0)
         return decrementIndex(-stepSize);

      for (int i = 0; i < stepSize; i++)
         incrementIndex(updateBufferBounds);
      return currentIndex;
   }

   public int decrementIndex(int stepSize)
   {
      if (stepSize < 0)
         return incrementIndex(false, -stepSize);

      for (int i = 0; i < stepSize; i++)
         decrementIndex();
      return currentIndex;
   }

   public void set(YoBufferPropertiesReadOnly other)
   {
      inPoint = other.getInPoint();
      outPoint = other.getOutPoint();
      currentIndex = other.getCurrentIndex();
      size = other.getSize();
   }

   public boolean setCurrentIndex(int newCurrentIndex)
   {
      if (newCurrentIndex < 0 || newCurrentIndex >= size || newCurrentIndex == currentIndex)
         return false;
      currentIndex = newCurrentIndex;
      return true;
   }

   public void setCurrentIndexUnsafe(int currentIndex)
   {
      this.currentIndex = currentIndex;
   }

   public void setSize(int size)
   {
      if (size <= 0)
         throw new IllegalArgumentException("Buffer size has to be strictly greater than 0.");
      this.size = size;
   }

   public boolean setInPointIndex(int newInPointIndex)
   {
      if (newInPointIndex < 0 || newInPointIndex >= size || newInPointIndex == inPoint)
         return false;
      inPoint = newInPointIndex;
      return true;
   }

   public boolean setOutPointIndex(int newOutPointIndex)
   {
      if (newOutPointIndex < 0 || newOutPointIndex >= size || newOutPointIndex == outPoint)
         return false;
      outPoint = newOutPointIndex;
      return true;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof YoBufferPropertiesReadOnly)
         return equals((YoBufferPropertiesReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return "Size " + size + ", index " + currentIndex + ", in-point " + inPoint + ", out-point " + outPoint;
   }
}

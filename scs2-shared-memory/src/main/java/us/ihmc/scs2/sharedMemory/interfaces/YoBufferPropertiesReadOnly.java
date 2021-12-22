package us.ihmc.scs2.sharedMemory.interfaces;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

public interface YoBufferPropertiesReadOnly
{
   int getSize();

   int getCurrentIndex();

   int getInPoint();

   int getOutPoint();

   default int getActiveBufferLength()
   {
      return SharedMemoryTools.computeSubLength(getInPoint(), getOutPoint(), getSize());
   }

   default boolean isIndexBetweenBounds(int indexToCheck)
   {
      return SharedMemoryTools.isInsideBounds(indexToCheck, getInPoint(), getOutPoint(), getSize());
   }

   default YoBufferPropertiesReadOnly copy()
   {
      int size = getSize();
      int currentIndex = getCurrentIndex();
      int inPoint = getInPoint();
      int outPoint = getOutPoint();

      return new YoBufferPropertiesReadOnly()
      {
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
      };
   }

   default boolean equals(YoBufferPropertiesReadOnly other)
   {
      if (other == null)
         return false;
      if (getCurrentIndex() != other.getCurrentIndex())
         return false;
      if (getSize() != other.getSize())
         return false;
      if (getInPoint() != other.getInPoint())
         return false;
      if (getOutPoint() != other.getOutPoint())
         return false;
      return true;
   }
}

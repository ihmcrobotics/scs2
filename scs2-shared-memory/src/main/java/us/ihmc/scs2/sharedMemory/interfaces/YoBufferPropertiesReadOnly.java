package us.ihmc.scs2.sharedMemory.interfaces;

import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Read-only interface gathering the general properties of a {@link YoSharedBuffer}.
 */
public interface YoBufferPropertiesReadOnly
{
   /**
    * Gets the buffer capacity, i.e. the total the number of values that the buffer can store for each
    * {@link YoVariable}.
    * 
    * @return the buffer size.
    */
   int getSize();

   /**
    * Gets the current reading/writing position in the buffer.
    * 
    * @return the current buffer index.
    */
   int getCurrentIndex();

   /**
    * Gets the first index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * 
    * @return the in-point index.
    */
   int getInPoint();

   /**
    * Gets the last index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * 
    * @return the out-point index.
    */
   int getOutPoint();

   /**
    * Gets the size of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * 
    * @return the length of the active part of the buffer.
    */
   default int getActiveBufferLength()
   {
      return SharedMemoryTools.computeSubLength(getInPoint(), getOutPoint(), getSize());
   }

   /**
    * Tests whether the given index is located in the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * 
    * @param indexToCheck the query.
    * @return {@code true} if the index is in the active part of the buffer, {@code false} if in the
    *         inactive part of the buffer or out of the buffer.
    */
   default boolean isIndexBetweenBounds(int indexToCheck)
   {
      return SharedMemoryTools.isInsideBounds(indexToCheck, getInPoint(), getOutPoint(), getSize());
   }

   /**
    * Creates a copy of this.
    * 
    * @return a copy of this.
    */
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

   /**
    * Compares this to other on a per property basis.
    * 
    * @param other the other set of properties to compare against this.
    * @return {@code true} if the two set of properties are considered equal, {@code false} otherwise.
    */
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

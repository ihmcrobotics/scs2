package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

/**
 * This class contains the properties required to request cropping a session's buffer.
 * 
 * @see YoSharedBuffer#cropBuffer(CropBufferRequest)
 */
public class CropBufferRequest
{
   private final int from;
   private final int to;

   /**
    * Creates a new request for cropping a buffer.
    * 
    * @param from the first index of data to keep (inclusive).
    * @param to   the last index of data to keep (inclusive).
    */
   public CropBufferRequest(int from, int to)
   {
      this.from = from;
      this.to = to;
   }

   /**
    * Copy constructor.
    * 
    * @param other the other request to make a copy of.
    */
   public CropBufferRequest(CropBufferRequest other)
   {
      from = other.from;
      to = other.to;
   }

   /**
    * Gets the first index of data keep (inclusive).
    * 
    * @return the from index.
    */
   public int getFrom()
   {
      return from;
   }

   /**
    * Gets the last index of data keep (inclusive).
    * 
    * @return the to index.
    */
   public int getTo()
   {
      return to;
   }

   /**
    * Calculates the size that the buffer will be after the crop.
    * 
    * @param originalBufferSize the buffer size before cropping.
    * @return the buffer size after cropping.
    */
   public int getCroppedSize(int originalBufferSize)
   {
      return SharedMemoryTools.computeSubLength(from, to, originalBufferSize);
   }

   /**
    * Tests on a per-component basis if the two crop requests are equal.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof CropBufferRequest)
      {
         CropBufferRequest other = (CropBufferRequest) object;
         if (from != other.from)
            return false;
         if (to != other.to)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * Human readable summary of this request.
    */
   @Override
   public String toString()
   {
      return "from: " + from + ", to: " + to;
   }
}

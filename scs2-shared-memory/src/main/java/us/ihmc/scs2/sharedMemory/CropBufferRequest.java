package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

public class CropBufferRequest
{
   private final int from, to;

   public CropBufferRequest(int from, int to)
   {
      this.from = from;
      this.to = to;
   }

   public CropBufferRequest(CropBufferRequest other)
   {
      from = other.from;
      to = other.to;
   }

   public int getFrom()
   {
      return from;
   }

   public int getTo()
   {
      return to;
   }

   public int getCroppedSize(int originalBufferSize)
   {
      return SharedMemoryTools.computeSubLength(from, to, originalBufferSize);
   }

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

   @Override
   public String toString()
   {
      return "from: " + from + ", to: " + to;
   }
}

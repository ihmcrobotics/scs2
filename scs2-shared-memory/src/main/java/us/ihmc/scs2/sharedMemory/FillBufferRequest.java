package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

public class FillBufferRequest
{
   private final boolean zeroFill;
   private final int from, to;

   public FillBufferRequest(boolean zeroFill, int from, int to)
   {
      this.zeroFill = zeroFill;
      this.from = from;
      this.to = to;
   }

   public boolean getZeroFill()
   {
      return zeroFill;
   }

   public int getFrom()
   {
      return from;
   }

   public int getTo()
   {
      return to;
   }

   public int getFilledSize(int originalBufferSize)
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
      else if (object instanceof FillBufferRequest)
      {
         FillBufferRequest other = (FillBufferRequest) object;
         if (zeroFill != other.zeroFill)
            return false;
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
      return "zeroFill: " + zeroFill + ", from: " + from + ", to: " + to;
   }
}

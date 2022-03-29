package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class can be used to request to fill in the buffer with "0"s or current {@link YoVariable}
 * values.
 */
public class FillBufferRequest
{
   private final boolean zeroFill;
   private final int from, to;

   /**
    * Creates a new request to fill in buffer data.
    * 
    * @param zeroFill {@code true} to fill in with zeros, {@code false} to fill in using the current
    *                 value stored in each {@link YoVariable}.
    * @param from     the first index of the fill in (inclusive).
    * @param to       the last index of the fill in (inclusive).
    */
   public FillBufferRequest(boolean zeroFill, int from, int to)
   {
      this.zeroFill = zeroFill;
      this.from = from;
      this.to = to;
   }

   /**
    * Whether to fill in with zeros or the current {@link YoVariable} values.
    * 
    * @return {@code true} to fill in with zeros, {@code false} to fill in using the current value
    *         stored in each {@link YoVariable}.
    */
   public boolean getZeroFill()
   {
      return zeroFill;
   }

   /**
    * Gets the first index of the fill in (inclusive).
    * 
    * @return the from index.
    */
   public int getFrom()
   {
      return from;
   }

   /**
    * Gets the last index of the fill in (inclusive).
    * 
    * @return the to index.
    */
   public int getTo()
   {
      return to;
   }

   /**
    * Calculates the size in the buffer to be filled in.
    * 
    * @param originalBufferSize the size of the buffer on which the fill in is to be performed.
    * @return the size to be filled in.
    */
   public int getFilledSize(int originalBufferSize)
   {
      return SharedMemoryTools.computeSubLength(from, to, originalBufferSize);
   }

   /**
    * Tests on a per-component basis if the two requests are equal.
    */
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

   /**
    * Human readable summary of this request.
    */
   @Override
   public String toString()
   {
      return "zeroFill: " + zeroFill + ", from: " + from + ", to: " + to;
   }
}

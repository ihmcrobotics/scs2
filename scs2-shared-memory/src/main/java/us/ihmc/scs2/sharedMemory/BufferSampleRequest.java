package us.ihmc.scs2.sharedMemory;

public class BufferSampleRequest
{
   private final int from;
   private final int length;

   public BufferSampleRequest(int from, int length)
   {
      this.from = from;
      this.length = length;
   }

   public BufferSampleRequest(BufferSampleRequest other)
   {
      from = other.from;
      length = other.length;
   }

   public int getFrom()
   {
      return from;
   }

   public int getLength()
   {
      return length;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof BufferSampleRequest)
      {
         BufferSampleRequest other = (BufferSampleRequest) object;
         if (from != other.from)
            return false;
         if (length != other.length)
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
      return "from " + from + ", length " + length;
   }
}

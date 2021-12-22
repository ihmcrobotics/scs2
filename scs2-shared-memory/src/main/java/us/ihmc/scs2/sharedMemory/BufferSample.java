package us.ihmc.scs2.sharedMemory;

import java.util.Arrays;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

public class BufferSample<D>
{
   private final int from, to;
   private final D sample;
   private final int sampleLength;
   private final YoBufferPropertiesReadOnly bufferProperties;

   public BufferSample(int from, D sample, int sampleLength, YoBufferPropertiesReadOnly bufferProperties)
   {
      this.from = from;
      this.sample = sample;
      this.sampleLength = sampleLength;
      this.bufferProperties = bufferProperties;

      to = SharedMemoryTools.computeToIndex(from, sampleLength, bufferProperties.getSize());
   }

   public BufferSample(BufferSample<D> other)
   {
      this.from = other.from;
      this.sample = other.sample;
      this.sampleLength = other.sampleLength;
      this.bufferProperties = other.bufferProperties;

      to = SharedMemoryTools.computeToIndex(from, sampleLength, bufferProperties.getSize());
   }

   public int getFrom()
   {
      return from;
   }

   public int getTo()
   {
      return to;
   }

   public D getSample()
   {
      return sample;
   }

   public int getSampleLength()
   {
      return sampleLength;
   }

   public YoBufferPropertiesReadOnly getBufferProperties()
   {
      return bufferProperties;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof BufferSample)
      {
         BufferSample<?> other = (BufferSample<?>) object;

         if (from != other.from)
            return false;
         if (sampleLength != other.sampleLength)
            return false;
         if (!bufferProperties.equals(other.bufferProperties))
            return false;
         return sampleEquals(sample, other.sample);
      }
      else
      {
         return false;
      }
   }

   static boolean sampleEquals(Object expected, Object actual)
   {
      if (expected == null)
         return actual == null;
      if (actual == null)
         return false;
      if (expected == actual)
         return true;

      if (!expected.getClass().isArray())
         throw new IllegalArgumentException("The argument \"expected\" is not an array.");
      if (!actual.getClass().isArray())
         throw new IllegalArgumentException("The argument \"actual\" is not an array.");

      if (expected.getClass() != actual.getClass())
         return false;
      if (expected.getClass().getComponentType() == boolean.class)
         return Arrays.equals((boolean[]) expected, (boolean[]) actual);
      if (expected.getClass().getComponentType() == double.class)
         return Arrays.equals((double[]) expected, (double[]) actual);
      if (expected.getClass().getComponentType() == int.class)
         return Arrays.equals((int[]) expected, (int[]) actual);
      if (expected.getClass().getComponentType() == long.class)
         return Arrays.equals((long[]) expected, (long[]) actual);
      if (expected.getClass().getComponentType() == byte.class)
         return Arrays.equals((byte[]) expected, (byte[]) actual);
      throw new IllegalStateException("Unhandled array type: " + expected.getClass().getComponentType());
   }

   @Override
   public String toString()
   {
      return "from: " + from + ", sample length: " + sampleLength + ", type: " + sample.getClass().getComponentType();
   }
}

package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;

public class BufferSampleRequestTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testGeneral()
   {
      Random random = new Random(645323);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int from = random.nextInt(100000);
         int length = random.nextInt(100000);
         BufferSampleRequest bufferSampleRequest = new BufferSampleRequest(from, length);
         assertEquals(from, bufferSampleRequest.getFrom());
         assertEquals(length, bufferSampleRequest.getLength());
      }
   }

   @Test
   public void testEquals()
   {
      Random random = new Random(232);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int from = random.nextInt(100000);
         int length = random.nextInt(100000);

         BufferSampleRequest bufferSampleRequest = new BufferSampleRequest(from, length);

         assertFalse(bufferSampleRequest.equals(null));
         assertFalse(bufferSampleRequest.equals(new ArrayList<>()));

         assertTrue(bufferSampleRequest.equals(bufferSampleRequest));
         assertTrue(bufferSampleRequest.equals(new BufferSampleRequest(bufferSampleRequest)));

         assertFalse(bufferSampleRequest.equals(new BufferSampleRequest(from + 1, length)));
         assertFalse(bufferSampleRequest.equals(new BufferSampleRequest(from, length + 1)));
      }
   }
}

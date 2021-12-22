package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;

public class CropBufferRequestTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testGetters()
   {
      Random random = new Random(7624);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(10000);
         int from = random.nextInt(size);
         int to = random.nextInt(size);
         CropBufferRequest cropBufferRequest = new CropBufferRequest(from, to);
         assertEquals(from, cropBufferRequest.getFrom());
         assertEquals(to, cropBufferRequest.getTo());

         // Use naive approach to measure the length
         int index = cropBufferRequest.getFrom();
         int expectedLength = 1;

         while (index != cropBufferRequest.getTo())
         {
            index++;
            if (index >= size)
               index = 0;
            expectedLength++;
         }

         assertEquals(expectedLength, cropBufferRequest.getCroppedSize(size));
      }
   }

   @Test
   public void testEquals()
   {
      Random random = new Random(96);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(10000);
         int from = random.nextInt(size);
         int to = random.nextInt(size);
         CropBufferRequest request = new CropBufferRequest(from, to);

         assertFalse(request.equals(null));
         assertFalse(request.equals(new ArrayList<>()));

         assertTrue(request.equals(request));
         assertTrue(request.equals(new CropBufferRequest(request)));
         assertFalse(request.equals(new CropBufferRequest(from + 1, to)));
         assertFalse(request.equals(new CropBufferRequest(from, to + 1)));
      }
   }
}

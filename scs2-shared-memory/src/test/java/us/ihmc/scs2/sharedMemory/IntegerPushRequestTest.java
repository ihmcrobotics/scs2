package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class IntegerPushRequestTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testPush()
   {
      Random random = new Random(3466);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoInteger bufferYoInteger = SharedMemoryRandomTools.nextYoInteger(random, new YoRegistry("Dummy"));
         int valueToPush = random.nextInt();

         int currentValue = bufferYoInteger.getValue();

         IntegerPushRequest pushRequest = new IntegerPushRequest(valueToPush, bufferYoInteger);
         assertEquals(currentValue, bufferYoInteger.getValue());

         pushRequest.push();
         assertEquals(valueToPush, bufferYoInteger.getValue());

         assertFalse(pushRequest.push());

         pushRequest = new IntegerPushRequest(valueToPush + 1, bufferYoInteger);
         assertTrue(pushRequest.push());
      }
   }
}

package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoublePushRequestTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testPush()
   {
      Random random = new Random(3466);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDouble bufferYoDouble = SharedMemoryRandomTools.nextYoDouble(random, new YoRegistry("Dummy"));
         double valueToPush = random.nextDouble();

         double currentValue = bufferYoDouble.getValue();

         DoublePushRequest pushRequest = new DoublePushRequest(valueToPush, bufferYoDouble);
         assertEquals(currentValue, bufferYoDouble.getValue());

         pushRequest.push();
         assertEquals(valueToPush, bufferYoDouble.getValue());

         assertFalse(pushRequest.push());

         pushRequest = new DoublePushRequest(valueToPush + 1, bufferYoDouble);
         assertTrue(pushRequest.push());
      }
   }
}

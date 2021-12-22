package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class BooleanPushRequestTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testPush()
   {
      Random random = new Random(3466);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBoolean bufferYoBoolean = SharedMemoryRandomTools.nextYoBoolean(random, new YoRegistry("Dummy"));
         boolean valueToPush = random.nextBoolean();

         boolean currentValue = bufferYoBoolean.getValue();

         BooleanPushRequest pushRequest = new BooleanPushRequest(valueToPush, bufferYoBoolean);
         assertEquals(currentValue, bufferYoBoolean.getValue());

         pushRequest.push();
         assertEquals(valueToPush, bufferYoBoolean.getValue());

         assertFalse(pushRequest.push());

         pushRequest = new BooleanPushRequest(!valueToPush, bufferYoBoolean);
         assertTrue(pushRequest.push());
      }
   }
}

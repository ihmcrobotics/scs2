package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class LongPullRequestTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void test()
   {
      Random random = new Random(453465);
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         YoLong variableToUpdate = SharedMemoryRandomTools.nextYoLong(random, new YoRegistry("Dummy"));
         long initialValue = variableToUpdate.getValue();
         long valueToPull = random.nextLong();
         LongPullRequest pullRequest = new LongPullRequest(variableToUpdate, valueToPull);

         assertEquals(initialValue, variableToUpdate.getValue());
         assertEquals(valueToPull, pullRequest.getValueToPull());
         pullRequest.pull();
         assertEquals(valueToPull, variableToUpdate.getValue());
      }
   }

}

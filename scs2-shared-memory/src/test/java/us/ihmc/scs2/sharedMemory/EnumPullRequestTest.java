package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class EnumPullRequestTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void test()
   {
      Random random = new Random(453465);
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         YoEnum<?> variableToUpdate = SharedMemoryRandomTools.nextYoEnum(random, new YoRegistry("Dummy"));
         int initialValue = variableToUpdate.getOrdinal();
         int valueToPull = random.nextInt(variableToUpdate.getEnumSize());
         EnumPullRequest<?> pullRequest = new EnumPullRequest<>(variableToUpdate, valueToPull);

         assertEquals(initialValue, variableToUpdate.getOrdinal());
         assertEquals(valueToPull, pullRequest.getValueToPull());
         pullRequest.pull();
         assertEquals(valueToPull, variableToUpdate.getOrdinal());
      }
   }

}

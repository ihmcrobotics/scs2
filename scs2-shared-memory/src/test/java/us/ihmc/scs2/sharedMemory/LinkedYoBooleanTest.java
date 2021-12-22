package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class LinkedYoBooleanTest extends LinkedYoVariableTest<YoBoolean>
{
   @Override
   YoBoolean copy(YoBoolean original)
   {
      YoBoolean copy = new YoBoolean(original.getName() + "Copy", new YoRegistry("Dummy"));
      copy.set(original.getValue());
      return copy;
   }

   @Override
   YoBoolean nextYoVariable(Random random, int iteration)
   {
      return SharedMemoryRandomTools.nextYoBoolean(random, new YoRegistry("Dummy"));
   }

   @Test
   public void testConstructor()
   {
      Random random = new Random(76267);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBoolean linkedVariable = SharedMemoryRandomTools.nextYoBoolean(random, new YoRegistry("Dummy"));
         YoBooleanBuffer buffer = SharedMemoryRandomTools.nextYoBooleanBuffer(random, new YoRegistry("Dummy"));
         LinkedYoBoolean linkedYoBoolean = new LinkedYoBoolean(linkedVariable, buffer);

         assertTrue(linkedVariable == linkedYoBoolean.getLinkedYoVariable());
         assertTrue(buffer == linkedYoBoolean.getBuffer());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBoolean linkedVariable = SharedMemoryRandomTools.nextYoBoolean(random, new YoRegistry("Dummy"));
         YoBooleanBuffer buffer = SharedMemoryRandomTools.nextYoBooleanBuffer(random, new YoRegistry("Dummy"));
         LinkedYoBoolean linkedYoBoolean = (LinkedYoBoolean) LinkedYoVariable.newLinkedYoVariable(linkedVariable, buffer);

         assertTrue(linkedVariable == linkedYoBoolean.getLinkedYoVariable());
         assertTrue(buffer == linkedYoBoolean.getBuffer());
      }
   }

   @Test
   public void testToPullRequest()
   {
      Random random = new Random(349785);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBoolean linkedVariable = SharedMemoryRandomTools.nextYoBoolean(random, new YoRegistry("Dummy"));
         YoBooleanBuffer buffer = SharedMemoryRandomTools.nextYoBooleanBuffer(random, new YoRegistry("Dummy"));
         LinkedYoBoolean linkedYoBoolean = new LinkedYoBoolean(linkedVariable, buffer);

         BooleanPullRequest pullRequest = linkedYoBoolean.toPullRequest();
         assertTrue(linkedVariable == pullRequest.getVariableToUpdate());
         assertEquals(buffer.getYoVariable().getValue(), pullRequest.getValueToPull());

         pullRequest.pull();
         assertEquals(linkedVariable.getValue(), buffer.getYoVariable().getValue());
      }
   }

   @Test
   public void testToPushRequest()
   {
      Random random = new Random(349785);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBoolean linkedVariable = SharedMemoryRandomTools.nextYoBoolean(random, new YoRegistry("Dummy"));
         YoBooleanBuffer buffer = SharedMemoryRandomTools.nextYoBooleanBuffer(random, new YoRegistry("Dummy"));
         LinkedYoBoolean linkedYoBoolean = new LinkedYoBoolean(linkedVariable, buffer);

         BooleanPushRequest pullRequest = linkedYoBoolean.toPushRequest();
         assertTrue(buffer.getYoVariable() == pullRequest.getVariableToUpdate());
         assertEquals(linkedVariable.getValue(), pullRequest.getValueToPush());

         pullRequest.push();
         assertEquals(linkedVariable.getValue(), buffer.getYoVariable().getValue());
      }
   }
}

package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class LinkedYoIntegerTest extends LinkedYoVariableTest<YoInteger>
{
   @Override
   YoInteger copy(YoInteger original)
   {
      YoInteger copy = new YoInteger(original.getName() + "Copy", new YoRegistry("Dummy"));
      copy.set(original.getValue());
      return copy;
   }

   @Override
   YoInteger nextYoVariable(Random random, int iteration)
   {
      return SharedMemoryRandomTools.nextYoInteger(random, new YoRegistry("Dummy"));
   }

   @Test
   public void testConstructor()
   {
      Random random = new Random(76267);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoInteger linkedVariable = SharedMemoryRandomTools.nextYoInteger(random, new YoRegistry("Dummy"));
         YoIntegerBuffer buffer = SharedMemoryRandomTools.nextYoIntegerBuffer(random, new YoRegistry("Dummy"));
         LinkedYoInteger linkedYoInteger = new LinkedYoInteger(linkedVariable, buffer, null);

         assertTrue(linkedVariable == linkedYoInteger.getLinkedYoVariable());
         assertTrue(buffer == linkedYoInteger.getBuffer());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoInteger linkedVariable = SharedMemoryRandomTools.nextYoInteger(random, new YoRegistry("Dummy"));
         YoIntegerBuffer buffer = SharedMemoryRandomTools.nextYoIntegerBuffer(random, new YoRegistry("Dummy"));
         LinkedYoInteger linkedYoInteger = (LinkedYoInteger) LinkedYoVariable.newLinkedYoVariable(linkedVariable, buffer);

         assertTrue(linkedVariable == linkedYoInteger.getLinkedYoVariable());
         assertTrue(buffer == linkedYoInteger.getBuffer());
      }
   }

   @Test
   public void testToPullRequest()
   {
      Random random = new Random(349785);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoInteger linkedVariable = SharedMemoryRandomTools.nextYoInteger(random, new YoRegistry("Dummy"));
         YoIntegerBuffer buffer = SharedMemoryRandomTools.nextYoIntegerBuffer(random, new YoRegistry("Dummy"));
         LinkedYoInteger linkedYoInteger = new LinkedYoInteger(linkedVariable, buffer, null);

         IntegerPullRequest pullRequest = linkedYoInteger.toPullRequest();
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
         YoInteger linkedVariable = SharedMemoryRandomTools.nextYoInteger(random, new YoRegistry("Dummy"));
         YoIntegerBuffer buffer = SharedMemoryRandomTools.nextYoIntegerBuffer(random, new YoRegistry("Dummy"));
         LinkedYoInteger linkedYoInteger = new LinkedYoInteger(linkedVariable, buffer, null);

         IntegerPushRequest pullRequest = linkedYoInteger.toPushRequest();
         assertTrue(buffer.getYoVariable() == pullRequest.getVariableToUpdate());
         assertEquals(linkedVariable.getValue(), pullRequest.getValueToPush());

         pullRequest.push();
         assertEquals(linkedVariable.getValue(), buffer.getYoVariable().getValue());
      }
   }
}

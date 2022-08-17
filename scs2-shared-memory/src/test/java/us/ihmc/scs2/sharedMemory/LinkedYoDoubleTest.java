package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LinkedYoDoubleTest extends LinkedYoVariableTest<YoDouble>
{
   @Override
   YoDouble copy(YoDouble original)
   {
      YoDouble copy = new YoDouble(original.getName() + "Copy", new YoRegistry("Dummy"));
      copy.set(original.getValue());
      return copy;
   }

   @Override
   YoDouble nextYoVariable(Random random, int iteration)
   {
      return SharedMemoryRandomTools.nextYoDouble(random, new YoRegistry("Dummy"));
   }

   @Test
   public void testConstructor()
   {
      Random random = new Random(76267);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDouble linkedVariable = SharedMemoryRandomTools.nextYoDouble(random, new YoRegistry("Dummy"));
         YoDoubleBuffer buffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         LinkedYoDouble linkedYoDouble = new LinkedYoDouble(linkedVariable, buffer, null);

         assertTrue(linkedVariable == linkedYoDouble.getLinkedYoVariable());
         assertTrue(buffer == linkedYoDouble.getBuffer());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDouble linkedVariable = SharedMemoryRandomTools.nextYoDouble(random, new YoRegistry("Dummy"));
         YoDoubleBuffer buffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         LinkedYoDouble linkedYoDouble = (LinkedYoDouble) LinkedYoVariable.newLinkedYoVariable(linkedVariable, buffer);

         assertTrue(linkedVariable == linkedYoDouble.getLinkedYoVariable());
         assertTrue(buffer == linkedYoDouble.getBuffer());
      }
   }

   @Test
   public void testToPullRequest()
   {
      Random random = new Random(349785);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDouble linkedVariable = SharedMemoryRandomTools.nextYoDouble(random, new YoRegistry("Dummy"));
         YoDoubleBuffer buffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         LinkedYoDouble linkedYoDouble = new LinkedYoDouble(linkedVariable, buffer, null);

         DoublePullRequest pullRequest = linkedYoDouble.toPullRequest();
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
         YoDouble linkedVariable = SharedMemoryRandomTools.nextYoDouble(random, new YoRegistry("Dummy"));
         YoDoubleBuffer buffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         LinkedYoDouble linkedYoDouble = new LinkedYoDouble(linkedVariable, buffer, null);

         DoublePushRequest pullRequest = linkedYoDouble.toPushRequest();
         assertTrue(buffer.getYoVariable() == pullRequest.getVariableToUpdate());
         assertEquals(linkedVariable.getValue(), pullRequest.getValueToPush());

         pullRequest.push();
         assertEquals(linkedVariable.getValue(), buffer.getYoVariable().getValue());
      }
   }
}

package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class LinkedYoVariableTest<T extends YoVariable>
{
   protected static final int ITERATIONS = 1000;

   abstract T copy(T original);

   abstract T nextYoVariable(Random random, int iteration);

   @SuppressWarnings("unchecked")
   @Test
   public void testPullSequence()
   {
      Random random = new Random(8907543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T linkedVariable = nextYoVariable(random, i);
         YoVariableBuffer<T> buffer = (YoVariableBuffer<T>) SharedMemoryRandomTools.nextYoVariableBuffer(random, nextYoVariable(random, i));
         LinkedYoVariable<T> linkedYoVariable = (LinkedYoVariable<T>) LinkedYoVariable.newLinkedYoVariable(linkedVariable, buffer);

         // Single prepare-pull happens before user actually grabs an update.
         T prePullLinkedValue = copy(linkedVariable);
         T prePullBufferValue = copy(buffer.getYoVariable());
         linkedYoVariable.prepareForPull(); // Called from the buffer manager, internally stores the buffer's variable's value to be pulled later from the user thread.
         assertYoEquals(prePullLinkedValue, linkedVariable);
         assertYoEquals(prePullBufferValue, buffer.getYoVariable());
         SharedMemoryRandomTools.randomizeYoVariable(random, buffer.getYoVariable()); // Simulates that the buffer manager is pursuing its changes on the buffer before the user has pulled its update.
         linkedYoVariable.pull(); // The user is actually grabbing an update.
         assertYoEquals(linkedVariable, prePullBufferValue);

         // Multiple prepare-pulls happen before user actually grabs an update.
         prePullLinkedValue = copy(linkedVariable);
         prePullBufferValue = copy(buffer.getYoVariable());

         T expectedUpdate = copy(prePullBufferValue);

         for (int j = 0; j < random.nextInt(20) + 2; j++)
         {
            SharedMemoryRandomTools.randomizeYoVariable(random, buffer.getYoVariable());
            linkedYoVariable.prepareForPull();
            expectedUpdate = copy(buffer.getYoVariable());
         }

         linkedYoVariable.pull(); // The user is actually grabbing an update.
         assertYoEquals(linkedVariable, expectedUpdate);

         // No prepare-pull, the user's variable remains constant regardless of what changes are being done on the buffer.
         prePullLinkedValue = copy(linkedVariable);
         for (int j = 0; j < random.nextInt(20) + 1; j++)
         {
            SharedMemoryRandomTools.randomizeYoVariable(random, buffer.getYoVariable());
            linkedYoVariable.pull();
            assertYoEquals(linkedVariable, prePullLinkedValue);
         }
      }
   }

   @SuppressWarnings("unchecked")
   @Test
   public void testPushSequence()
   {
      Random random = new Random(8234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T linkedVariable = nextYoVariable(random, i);
         YoVariableBuffer<T> buffer = (YoVariableBuffer<T>) SharedMemoryRandomTools.nextYoVariableBuffer(random, nextYoVariable(random, i));
         LinkedYoVariable<T> linkedYoVariable = (LinkedYoVariable<T>) LinkedYoVariable.newLinkedYoVariable(linkedVariable, buffer);

         // Single push before the before the buffer manager actually applies it.
         T prePushLinkedValue = copy(linkedVariable);
         T prePushBufferValue = copy(buffer.getYoVariable());
         linkedYoVariable.push(); // Called from the user to request a modification in the buffer.
         assertYoEquals(prePushLinkedValue, linkedVariable);
         assertYoEquals(prePushBufferValue, buffer.getYoVariable());
         SharedMemoryRandomTools.randomizeYoVariable(random, linkedVariable); // Simulates that the user is further modifying his local variable.
         linkedYoVariable.processPush(false); // The buffer manager is actually applying the push.
         assertYoEquals(prePushLinkedValue, buffer.getYoVariable());

         // Multiple pushes happen before the buffer manager is able to apply its first one.
         prePushLinkedValue = copy(linkedVariable);
         prePushBufferValue = copy(buffer.getYoVariable());

         T expectedUpdate = copy(prePushLinkedValue);

         for (int j = 0; j < random.nextInt(20) + 2; j++)
         {
            SharedMemoryRandomTools.randomizeYoVariable(random, linkedVariable);
            linkedYoVariable.push();
            expectedUpdate = copy(linkedVariable);
         }

         linkedYoVariable.processPush(false); // The buffer manager is actually applying the requested change.
         assertYoEquals(expectedUpdate, buffer.getYoVariable());

         // No push, the buffer's variable does not change in value even if the linked variable is changing.
         prePushBufferValue = copy(buffer.getYoVariable());
         for (int j = 0; j < random.nextInt(20) + 1; j++)
         {
            SharedMemoryRandomTools.randomizeYoVariable(random, linkedVariable);
            linkedYoVariable.processPush(false);
            assertYoEquals(prePushBufferValue, buffer.getYoVariable());
         }
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   @Test
   public void testBufferSampleRequestSequence()
   {
      Random random = new Random(345780);

      { // Test invalid requests throw an exception
         T linkedVariable = nextYoVariable(random, 0);
         YoVariableBuffer<T> buffer = (YoVariableBuffer<T>) SharedMemoryRandomTools.nextYoVariableBuffer(random, nextYoVariable(random, 0));
         LinkedYoVariable<T> linkedYoVariable = (LinkedYoVariable<T>) LinkedYoVariable.newLinkedYoVariable(linkedVariable, buffer);
         YoBufferPropertiesReadOnly properties = buffer.getProperties();

         linkedYoVariable.requestBufferWindow(-3, 1);
         assertThrows(IllegalArgumentException.class, () -> linkedYoVariable.prepareForPull());

         linkedYoVariable.requestBufferWindow(0, -2);
         assertThrows(IllegalArgumentException.class, () -> linkedYoVariable.prepareForPull());

         linkedYoVariable.requestBufferWindow(0, -3);
         assertThrows(IllegalArgumentException.class, () -> linkedYoVariable.prepareForPull());

         linkedYoVariable.requestBufferWindow(properties.getSize(), 1);
         assertDoesNotThrow(() -> linkedYoVariable.prepareForPull());

         linkedYoVariable.requestBufferWindow(0, properties.getSize() + 1);
         assertDoesNotThrow(() -> linkedYoVariable.prepareForPull());

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         T linkedVariable = nextYoVariable(random, i);
         YoVariableBuffer<T> buffer = (YoVariableBuffer<T>) SharedMemoryRandomTools.nextYoVariableBuffer(random, nextYoVariable(random, i));
         LinkedYoVariable<T> linkedYoVariable = (LinkedYoVariable<T>) LinkedYoVariable.newLinkedYoVariable(linkedVariable, buffer);
         YoBufferPropertiesReadOnly properties = buffer.getProperties();

         // Without a request, no buffer sample should be available.
         for (int j = 0; j < random.nextInt(20) + 1; j++)
         {
            linkedYoVariable.prepareForPull();
            linkedYoVariable.pull();
            assertFalse(linkedYoVariable.hasRequestPending());
            assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());
            assertNull(linkedYoVariable.pollRequestedBufferSample());
         }

         // Request entire buffer.
         linkedYoVariable.requestEntireBuffer();
         assertTrue(linkedYoVariable.hasRequestPending());
         assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());
         linkedYoVariable.prepareForPull();
         assertFalse(linkedYoVariable.hasRequestPending());
         assertTrue(linkedYoVariable.isRequestedBufferSampleAvailable());
         BufferSample bufferSample = linkedYoVariable.pollRequestedBufferSample();
         assertFalse(linkedYoVariable.hasRequestPending());
         assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());

         assertEquals(0, bufferSample.getFrom());
         assertEquals(properties.getSize() - 1, bufferSample.getTo());
         assertEquals(properties.getSize(), bufferSample.getSampleLength());
         assertEquals(buffer.copy(0, properties.getSize(), properties.copy()), bufferSample);

         // Request the active part of the buffer.
         linkedYoVariable.requestActiveBufferOnly();
         assertTrue(linkedYoVariable.hasRequestPending());
         assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());
         linkedYoVariable.prepareForPull();
         assertFalse(linkedYoVariable.hasRequestPending());
         assertTrue(linkedYoVariable.isRequestedBufferSampleAvailable());
         bufferSample = linkedYoVariable.pollRequestedBufferSample();
         assertFalse(linkedYoVariable.hasRequestPending());
         assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());

         assertEquals(properties.getInPoint(), bufferSample.getFrom());
         assertEquals(properties.getOutPoint(), bufferSample.getTo());
         assertEquals(properties.getActiveBufferLength(), bufferSample.getSampleLength());
         assertEquals(buffer.copy(properties.getInPoint(), properties.getActiveBufferLength(), properties.copy()), bufferSample);

         // Request a part of the buffer starting from a given index to the latest out-point
         int start = random.nextInt(properties.getSize());
         linkedYoVariable.requestBufferStartingFrom(start);
         assertTrue(linkedYoVariable.hasRequestPending());
         assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());
         linkedYoVariable.prepareForPull();
         assertFalse(linkedYoVariable.hasRequestPending());
         assertTrue(linkedYoVariable.isRequestedBufferSampleAvailable());
         bufferSample = linkedYoVariable.pollRequestedBufferSample();
         assertFalse(linkedYoVariable.hasRequestPending());
         assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());

         assertEquals(start, bufferSample.getFrom());
         assertEquals(properties.getOutPoint(), bufferSample.getTo());
         assertEquals(SharedMemoryTools.computeSubLength(start, properties.getOutPoint(), properties.getSize()), bufferSample.getSampleLength());
         assertEquals(buffer.copy(start, bufferSample.getSampleLength(), properties.copy()), bufferSample);

         // Request a part of the buffer given the starting point and the length of the sample
         start = random.nextInt(properties.getSize());
         int length = random.nextInt(properties.getSize());
         linkedYoVariable.requestBufferWindow(start, length);
         assertTrue(linkedYoVariable.hasRequestPending());
         assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());
         linkedYoVariable.prepareForPull();
         assertFalse(linkedYoVariable.hasRequestPending());

         if (length == 0)
         {
            assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());
            assertNull(linkedYoVariable.pollRequestedBufferSample());
         }
         else
         {
            assertTrue(linkedYoVariable.isRequestedBufferSampleAvailable());
            bufferSample = linkedYoVariable.pollRequestedBufferSample();
            assertFalse(linkedYoVariable.hasRequestPending());
            assertFalse(linkedYoVariable.isRequestedBufferSampleAvailable());

            assertEquals(start, bufferSample.getFrom());
            assertEquals(SharedMemoryTools.computeToIndex(start, length, properties.getSize()), bufferSample.getTo());
            assertEquals(length, bufferSample.getSampleLength());
            assertEquals(buffer.copy(start, bufferSample.getSampleLength(), properties.copy()), bufferSample);
         }
      }
   }

   private static <T extends YoVariable> void assertYoEquals(T expected, T actual)
   {
      assertEquals(expected.getValueAsLongBits(), actual.getValueAsLongBits());
   }
}

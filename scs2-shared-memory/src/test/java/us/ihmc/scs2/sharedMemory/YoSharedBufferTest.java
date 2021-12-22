package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoSharedBufferTest
{
   private static final int ITERATIONS = 200;

   @Test
   public void testCropBuffer()
   {
      Random random = new Random(6234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);
         YoBufferPropertiesReadOnly newProperties = yoSharedBuffer.getProperties();
         YoBufferPropertiesReadOnly initialProperties = newProperties.copy();

         CropBufferRequest cropBufferRequest = new CropBufferRequest(random.nextInt(initialProperties.getSize()), random.nextInt(initialProperties.getSize()));

         yoSharedBuffer.cropBuffer(cropBufferRequest);
         assertEquals(cropBufferRequest.getCroppedSize(initialProperties.getSize()), newProperties.getSize());
         assertEquals(0, newProperties.getInPoint());
         assertEquals(newProperties.getSize() - 1, newProperties.getOutPoint());
         assertEquals(newProperties.getInPoint(), newProperties.getCurrentIndex());
      }
   }

   @Test
   public void testResizeBuffer()
   {
      Random random = new Random(6234);

      { // Assert cases the buffer won't resize
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);

         assertFalse(yoSharedBuffer.resizeBuffer(0));
         assertFalse(yoSharedBuffer.resizeBuffer(-1));
         assertFalse(yoSharedBuffer.resizeBuffer(yoSharedBuffer.getProperties().getSize()));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Increase the size
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);
         YoBufferPropertiesReadOnly newProperties = yoSharedBuffer.getProperties();
         YoBufferPropertiesReadOnly initialProperties = newProperties.copy();

         int newSize = initialProperties.getSize() + random.nextInt(1000) + 1;

         assertTrue(yoSharedBuffer.resizeBuffer(newSize));
         assertEquals(newSize, newProperties.getSize());
         assertEquals(0, newProperties.getInPoint());
         assertEquals(initialProperties.getActiveBufferLength() - 1, newProperties.getOutPoint());
         if (initialProperties.isIndexBetweenBounds(initialProperties.getCurrentIndex()))
         {
            int expectedNewCurrentIndex = initialProperties.getCurrentIndex() - initialProperties.getInPoint();
            if (expectedNewCurrentIndex < 0)
               expectedNewCurrentIndex += initialProperties.getSize();
            assertEquals(expectedNewCurrentIndex, newProperties.getCurrentIndex());
         }
         else
         {
            assertEquals(newProperties.getOutPoint(), newProperties.getCurrentIndex());
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Decrease the size while remaining greater than the length of the active part of the buffer
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);
         YoBufferPropertiesReadOnly newProperties = yoSharedBuffer.getProperties();

         while (newProperties.getActiveBufferLength() == newProperties.getSize())
            yoSharedBuffer.setOutPoint(random.nextInt(newProperties.getSize()));

         YoBufferPropertiesReadOnly initialProperties = newProperties.copy();

         int newSize = RandomNumbers.nextInt(random, initialProperties.getActiveBufferLength(), initialProperties.getSize() - 1);

         assertTrue(yoSharedBuffer.resizeBuffer(newSize));
         assertEquals(newSize, newProperties.getSize());
         assertEquals(0, newProperties.getInPoint());
         assertEquals(initialProperties.getActiveBufferLength() - 1, newProperties.getOutPoint());
         if (initialProperties.isIndexBetweenBounds(initialProperties.getCurrentIndex()))
         {
            int expectedNewCurrentIndex = initialProperties.getCurrentIndex() - initialProperties.getInPoint();
            if (expectedNewCurrentIndex < 0)
               expectedNewCurrentIndex += initialProperties.getSize();
            assertEquals(expectedNewCurrentIndex, newProperties.getCurrentIndex());
         }
         else
         {
            assertEquals(newProperties.getOutPoint(), newProperties.getCurrentIndex());
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Decrease the size to something smaller than the length of the active part of the buffer.
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);
         YoBufferPropertiesReadOnly newProperties = yoSharedBuffer.getProperties();
         YoBufferPropertiesReadOnly initialProperties = newProperties.copy();

         int newSize = random.nextInt(initialProperties.getActiveBufferLength()) + 1;

         if (newSize == newProperties.getSize())
         {
            i--;
            continue;
         }

         assertTrue(yoSharedBuffer.resizeBuffer(newSize));
         assertEquals(newSize, newProperties.getSize());
         assertEquals(0, newProperties.getInPoint());
         assertEquals(newSize - 1, newProperties.getOutPoint());

         int expectedNewCurrentIndex = initialProperties.getCurrentIndex() - initialProperties.getInPoint();
         if (expectedNewCurrentIndex < 0)
            expectedNewCurrentIndex += initialProperties.getSize();
         expectedNewCurrentIndex += -initialProperties.getActiveBufferLength() + newSize;
         if (expectedNewCurrentIndex < 0 || expectedNewCurrentIndex >= newSize)
            assertEquals(newProperties.getOutPoint(), newProperties.getCurrentIndex());
         else
            assertEquals(expectedNewCurrentIndex, newProperties.getCurrentIndex());
      }
   }

   @Test
   public void testSetCurrentIndex()
   {
      Random random = new Random(4566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);
         yoSharedBuffer.readBuffer();

         YoBufferPropertiesReadOnly properties = yoSharedBuffer.getProperties();

         assertFalse(yoSharedBuffer.setCurrentIndex(-1));
         assertFalse(yoSharedBuffer.setCurrentIndex(properties.getSize()));

         int newCurrentIndex = random.nextInt(properties.getSize());
         assertEquals(newCurrentIndex != properties.getCurrentIndex(), yoSharedBuffer.setCurrentIndex(newCurrentIndex));

         assertEquals(newCurrentIndex, properties.getCurrentIndex());
         assertFalse(yoSharedBuffer.setCurrentIndex(newCurrentIndex));
      }
   }

   @Test
   public void testSetInOutPoint()
   {
      Random random = new Random(4566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);
         yoSharedBuffer.readBuffer();

         YoBufferPropertiesReadOnly properties = yoSharedBuffer.getProperties();

         // Test setInPoint
         assertFalse(yoSharedBuffer.setInPoint(-1));
         assertFalse(yoSharedBuffer.setInPoint(properties.getSize()));

         int newInPoint = random.nextInt(properties.getSize());
         assertEquals(newInPoint != properties.getInPoint(), yoSharedBuffer.setInPoint(newInPoint));

         assertEquals(newInPoint, properties.getInPoint());
         assertFalse(yoSharedBuffer.setInPoint(newInPoint));

         // Test setInPoint
         assertFalse(yoSharedBuffer.setOutPoint(-1));
         assertFalse(yoSharedBuffer.setOutPoint(properties.getSize()));

         int newOutPoint = random.nextInt(properties.getSize());
         assertEquals(newOutPoint != properties.getOutPoint(), yoSharedBuffer.setOutPoint(newOutPoint));

         assertEquals(newOutPoint, properties.getOutPoint());
         assertFalse(yoSharedBuffer.setOutPoint(newOutPoint));
      }
   }

   @Test
   public void testProcessLinkedPushRequests()
   {
      Random random = new Random(4566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);
         yoSharedBuffer.readBuffer();

         YoBufferPropertiesReadOnly properties = yoSharedBuffer.getProperties();
         YoRegistry bufferRootRegistry = yoSharedBuffer.getRootRegistry();

         List<YoVariable> allBufferYoVariables = bufferRootRegistry.collectSubtreeVariables();
         if (allBufferYoVariables.isEmpty())
         {
            i--;
            continue;
         }
         YoVariable bufferYoVariable = allBufferYoVariables.get(random.nextInt(allBufferYoVariables.size()));
         YoRegistry consumerRegistry = SharedMemoryTools.newEmptyCloneRegistry(bufferYoVariable.getRegistry());
         YoVariable consumerYoVariable = bufferYoVariable.duplicate(consumerRegistry);
         LinkedYoVariable<?> linkedYoVariable = yoSharedBuffer.newLinkedYoVariable(consumerYoVariable);

         SharedMemoryRandomTools.randomizeYoVariable(random, consumerYoVariable);
         linkedYoVariable.push();

         boolean writeBuffer = random.nextBoolean();
         yoSharedBuffer.processLinkedPushRequests(writeBuffer);

         assertYoEquals(consumerYoVariable, bufferYoVariable);

         if (writeBuffer)
            assertVariableEqualsBufferAt(bufferYoVariable,
                                         yoSharedBuffer.getRegistryBuffer().findYoVariableBuffer(bufferYoVariable),
                                         properties.getCurrentIndex());
      }
   }

   @Test
   public void testFlushLinkedPushRequests()
   {
      Random random = new Random(4566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);
         yoSharedBuffer.readBuffer();

         YoRegistry bufferRootRegistry = yoSharedBuffer.getRootRegistry();

         List<YoVariable> allBufferYoVariables = bufferRootRegistry.collectSubtreeVariables();
         if (allBufferYoVariables.isEmpty())
         {
            i--;
            continue;
         }
         YoVariable bufferYoVariable = allBufferYoVariables.get(random.nextInt(allBufferYoVariables.size()));
         YoRegistry consumerRegistry = SharedMemoryTools.newEmptyCloneRegistry(bufferYoVariable.getRegistry());
         YoVariable consumerYoVariable = bufferYoVariable.duplicate(consumerRegistry);
         LinkedYoVariable<?> linkedYoVariable = yoSharedBuffer.newLinkedYoVariable(consumerYoVariable);

         SharedMemoryRandomTools.randomizeYoVariable(random, consumerYoVariable);
         linkedYoVariable.push();

         yoSharedBuffer.flushLinkedPushRequests();

         assertFalse(linkedYoVariable.processPush(true));
      }
   }

   @Test
   public void testReadBuffer()
   {
      Random random = new Random(4566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);

         YoRegistry bufferRootRegistry = yoSharedBuffer.getRootRegistry();
         YoRegistryBuffer registryBuffer = yoSharedBuffer.getRegistryBuffer();
         YoBufferPropertiesReadOnly properties = yoSharedBuffer.getProperties();

         List<YoVariable> allBufferYoVariables = bufferRootRegistry.collectSubtreeVariables();
         if (allBufferYoVariables.isEmpty())
         {
            i--;
            continue;
         }

         long[] bufferBackedUp = allBufferYoVariables.stream().map(v -> registryBuffer.findYoVariableBuffer(v)).mapToLong(b -> b.getValueAsLongBits())
                                                     .toArray();
         yoSharedBuffer.readBuffer();

         for (int j = 0; j < allBufferYoVariables.size(); j++)
         {
            YoVariable bufferYoVariable = allBufferYoVariables.get(j);
            YoVariableBuffer<?> buffer = registryBuffer.findYoVariableBuffer(bufferYoVariable);
            assertVariableEqualsBufferAt(bufferYoVariable, buffer, properties.getCurrentIndex());
            assertEquals(bufferBackedUp[j], buffer.getValueAsLongBits());
         }
      }
   }

   @Test
   public void testWriteBuffer()
   {
      Random random = new Random(4566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);

         YoRegistry bufferRootRegistry = yoSharedBuffer.getRootRegistry();
         YoRegistryBuffer registryBuffer = yoSharedBuffer.getRegistryBuffer();
         YoBufferPropertiesReadOnly properties = yoSharedBuffer.getProperties();

         List<YoVariable> allBufferYoVariables = bufferRootRegistry.collectSubtreeVariables();
         if (allBufferYoVariables.isEmpty())
         {
            i--;
            continue;
         }

         long[] variableBackedUp = allBufferYoVariables.stream().mapToLong(YoVariable::getValueAsLongBits).toArray();
         yoSharedBuffer.writeBuffer();

         for (int j = 0; j < allBufferYoVariables.size(); j++)
         {
            YoVariable bufferYoVariable = allBufferYoVariables.get(j);
            YoVariableBuffer<?> buffer = registryBuffer.findYoVariableBuffer(bufferYoVariable);
            assertVariableEqualsBufferAt(bufferYoVariable, buffer, properties.getCurrentIndex());
            assertEquals(variableBackedUp[j], bufferYoVariable.getValueAsLongBits());
         }
      }
   }

   @Test
   public void testPrepareLinkedBuffersForPull()
   {
      Random random = new Random(2356);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoSharedBuffer yoSharedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 2, 5);

         YoRegistry bufferRootRegistry = yoSharedBuffer.getRootRegistry();

         List<YoVariable> allBufferYoVariables = bufferRootRegistry.collectSubtreeVariables();
         if (allBufferYoVariables.isEmpty())
         {
            i--;
            continue;
         }

         LinkedYoRegistry linkedYoRegistry = yoSharedBuffer.newLinkedYoRegistry();

         List<YoVariable> allConsumerYoVariables = linkedYoRegistry.getRootRegistry().collectSubtreeVariables();
         // Need to manually indicate that all YoVariable should be linked and add a user to the links so they don't get thrown away
         allConsumerYoVariables.forEach(var -> linkedYoRegistry.linkYoVariable(var).addUser(this));

         allBufferYoVariables.forEach(v -> SharedMemoryRandomTools.randomizeYoVariable(random, v));
         long[] bufferVariableBackedUp = allBufferYoVariables.stream().mapToLong(YoVariable::getValueAsLongBits).toArray();
         yoSharedBuffer.prepareLinkedBuffersForPull();

         for (int j = 0; j < allBufferYoVariables.size(); j++)
         {
            YoVariable bufferYoVariable = allBufferYoVariables.get(j);
            assertEquals(bufferVariableBackedUp[j], bufferYoVariable.getValueAsLongBits());
         }

         linkedYoRegistry.pull();

         for (int j = 0; j < allBufferYoVariables.size(); j++)
         {
            YoVariable bufferYoVariable = allBufferYoVariables.get(j);
            YoVariable consumerYoVariable = allConsumerYoVariables.get(j);
            assertEquals(bufferVariableBackedUp[j], bufferYoVariable.getValueAsLongBits());
            assertYoEquals(bufferYoVariable, consumerYoVariable);
         }
      }
   }

   private static void assertYoEquals(YoVariable expected, YoVariable actual)
   {
      assertTrue(expected.getClass() == actual.getClass());
      assertEquals(expected.getValueAsLongBits(), actual.getValueAsLongBits());
   }

   private static void assertVariableEqualsBufferAt(YoVariable yoVariable, YoVariableBuffer<?> buffer, int index)
   {
      if (yoVariable instanceof YoBoolean)
         assertEquals(((YoBoolean) yoVariable).getValue(), ((YoBooleanBuffer) buffer).getBuffer()[index]);
      else if (yoVariable instanceof YoDouble)
         assertEquals(((YoDouble) yoVariable).getValue(), ((YoDoubleBuffer) buffer).getBuffer()[index]);
      else if (yoVariable instanceof YoInteger)
         assertEquals(((YoInteger) yoVariable).getValue(), ((YoIntegerBuffer) buffer).getBuffer()[index]);
      else if (yoVariable instanceof YoLong)
         assertEquals(((YoLong) yoVariable).getValue(), ((YoLongBuffer) buffer).getBuffer()[index]);
      else if (yoVariable instanceof YoEnum)
         assertEquals(((YoEnum<?>) yoVariable).getOrdinal(), ((YoEnumBuffer<?>) buffer).getBuffer()[index]);
      else
         throw new IllegalStateException("Type not handled: " + yoVariable.getClass());
   }
}

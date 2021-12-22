package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class YoLongBufferTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructors()
   {
      Random random = new Random(467);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoLong yoLong = SharedMemoryRandomTools.nextYoLong(random, new YoRegistry("Dummy"));
         YoBufferProperties yoBufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoLongBuffer yoLongBuffer = new YoLongBuffer(yoLong, yoBufferProperties);
         assertTrue(yoLong == yoLongBuffer.getYoVariable());
         assertTrue(yoBufferProperties == yoLongBuffer.getProperties());
         assertEquals(yoBufferProperties.getSize(), yoLongBuffer.getBuffer().length);

         for (int j = 0; j < yoBufferProperties.getSize(); j++)
            assertEquals(0L, yoLongBuffer.getBuffer()[j]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoLong yoLong = SharedMemoryRandomTools.nextYoLong(random, new YoRegistry("Dummy"));
         YoBufferProperties yoBufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoLongBuffer yoLongBuffer = (YoLongBuffer) YoVariableBuffer.newYoVariableBuffer(yoLong, yoBufferProperties);
         assertTrue(yoLong == yoLongBuffer.getYoVariable());
         assertTrue(yoBufferProperties == yoLongBuffer.getProperties());
         assertEquals(yoBufferProperties.getSize(), yoLongBuffer.getBuffer().length);

         for (int j = 0; j < yoBufferProperties.getSize(); j++)
            assertEquals(0L, yoLongBuffer.getBuffer()[j]);
      }
   }

   @Test
   public void testResizeBuffer()
   {
      Random random = new Random(8967254);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoLongBuffer yoLongBuffer = SharedMemoryRandomTools.nextYoLongBuffer(random, new YoRegistry("Dummy"));
         YoBufferProperties originalBufferProperties = new YoBufferProperties(yoLongBuffer.getProperties());
         int from = random.nextInt(yoLongBuffer.getProperties().getSize());
         int newLength = random.nextInt(yoLongBuffer.getProperties().getSize());
         long[] expectedBuffer = SharedMemoryTools.ringArrayCopy(yoLongBuffer.getBuffer(), from, newLength);

         yoLongBuffer.resizeBuffer(from, newLength);
         assertArrayEquals(expectedBuffer, yoLongBuffer.getBuffer());
         assertEquals(originalBufferProperties, yoLongBuffer.getProperties());

         long[] buffer = yoLongBuffer.getBuffer();
         yoLongBuffer.resizeBuffer(0, buffer.length);
         assertTrue(buffer == yoLongBuffer.getBuffer());
      }
   }

   @Test
   public void testWriteBuffer()
   {
      Random random = new Random(867324);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoLongBuffer yoLongBuffer = SharedMemoryRandomTools.nextYoLongBuffer(random, new YoRegistry("Dummy"));
         YoLong yoLong = yoLongBuffer.getYoVariable();

         int currentIndex = yoLongBuffer.getProperties().getCurrentIndex();
         for (int j = 0; j < 10; j++)
         {
            long newValue = RandomNumbers.nextInt(random, -100000, 100000);
            yoLong.set(newValue);
            yoLongBuffer.writeBuffer();
            assertEquals(newValue, yoLong.getValue());
            assertEquals(newValue, yoLongBuffer.getBuffer()[currentIndex]);
         }
      }
   }

   @Test
   public void testReadBuffer()
   {
      Random random = new Random(867324);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoLongBuffer yoLongBuffer = SharedMemoryRandomTools.nextYoLongBuffer(random, new YoRegistry("Dummy"));
         YoLong yoLong = yoLongBuffer.getYoVariable();

         int currentIndex = yoLongBuffer.getProperties().getCurrentIndex();
         for (int j = 0; j < 10; j++)
         {
            long newValue = RandomNumbers.nextInt(random, -100000, 100000);
            yoLongBuffer.getBuffer()[currentIndex] = newValue;
            yoLongBuffer.readBuffer();
            assertEquals(newValue, yoLong.getValue());
            assertEquals(newValue, yoLongBuffer.getBuffer()[currentIndex]);
         }
      }
   }

   @Test
   public void testCopy()
   {
      Random random = new Random(43566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoLongBuffer yoLongBuffer = SharedMemoryRandomTools.nextYoLongBuffer(random, new YoRegistry("Dummy"));
         int from = random.nextInt(yoLongBuffer.getProperties().getSize());
         int length = random.nextInt(yoLongBuffer.getProperties().getSize() - 1) + 1;

         long[] expectedCopy = SharedMemoryTools.ringArrayCopy(yoLongBuffer.getBuffer(), from, length);
         BufferSample<long[]> actualCopy = yoLongBuffer.copy(from, length, yoLongBuffer.getProperties().copy());

         assertEquals(from, actualCopy.getFrom());
         assertEquals(length, actualCopy.getSampleLength());
         assertEquals(yoLongBuffer.getProperties(), actualCopy.getBufferProperties());
         int to = from + length - 1;
         if (to >= yoLongBuffer.getProperties().getSize())
            to -= yoLongBuffer.getProperties().getSize();
         assertEquals(to, actualCopy.getTo());
         assertArrayEquals(expectedCopy, actualCopy.getSample());
      }
   }

   @Test
   public void testNewLinkedYoVariable()
   {
      Random random = new Random(87324);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoLongBuffer yoLongBuffer = SharedMemoryRandomTools.nextYoLongBuffer(random, new YoRegistry("Dummy"));
         YoLong linkedLong = new YoLong("linked", new YoRegistry("Dummy"));
         LinkedYoLong linkedYoVariable = yoLongBuffer.newLinkedYoVariable(linkedLong);
         assertTrue(linkedLong == linkedYoVariable.getLinkedYoVariable());
         assertTrue(yoLongBuffer == linkedYoVariable.getBuffer());
      }
   }
}

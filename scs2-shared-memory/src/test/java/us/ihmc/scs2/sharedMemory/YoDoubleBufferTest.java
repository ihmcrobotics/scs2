package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoDoubleBufferTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructors()
   {
      Random random = new Random(467);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDouble yoDouble = SharedMemoryRandomTools.nextYoDouble(random, new YoRegistry("Dummy"));
         YoBufferProperties yoBufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoDoubleBuffer yoDoubleBuffer = new YoDoubleBuffer(yoDouble, yoBufferProperties);
         assertTrue(yoDouble == yoDoubleBuffer.getYoVariable());
         assertTrue(yoBufferProperties == yoDoubleBuffer.getProperties());
         assertEquals(yoBufferProperties.getSize(), yoDoubleBuffer.getBuffer().length);

         for (int j = 0; j < yoBufferProperties.getSize(); j++)
            assertEquals(0.0, yoDoubleBuffer.getBuffer()[j]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDouble yoDouble = SharedMemoryRandomTools.nextYoDouble(random, new YoRegistry("Dummy"));
         YoBufferProperties yoBufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoDoubleBuffer yoDoubleBuffer = (YoDoubleBuffer) YoVariableBuffer.newYoVariableBuffer(yoDouble, yoBufferProperties);
         assertTrue(yoDouble == yoDoubleBuffer.getYoVariable());
         assertTrue(yoBufferProperties == yoDoubleBuffer.getProperties());
         assertEquals(yoBufferProperties.getSize(), yoDoubleBuffer.getBuffer().length);

         for (int j = 0; j < yoBufferProperties.getSize(); j++)
            assertEquals(0.0, yoDoubleBuffer.getBuffer()[j]);
      }
   }

   @Test
   public void testResizeBuffer()
   {
      Random random = new Random(8967254);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDoubleBuffer yoDoubleBuffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         YoBufferProperties originalBufferProperties = new YoBufferProperties(yoDoubleBuffer.getProperties());
         int from = random.nextInt(yoDoubleBuffer.getProperties().getSize());
         int newLength = random.nextInt(yoDoubleBuffer.getProperties().getSize());
         double[] expectedBuffer = SharedMemoryTools.ringArrayCopy(yoDoubleBuffer.getBuffer(), from, newLength);

         yoDoubleBuffer.resizeBuffer(from, newLength);
         assertArrayEquals(expectedBuffer, yoDoubleBuffer.getBuffer());
         assertEquals(originalBufferProperties, yoDoubleBuffer.getProperties());

         double[] buffer = yoDoubleBuffer.getBuffer();
         yoDoubleBuffer.resizeBuffer(0, buffer.length);
         assertTrue(buffer == yoDoubleBuffer.getBuffer());
      }
   }

   @Test
   public void testWriteBuffer()
   {
      Random random = new Random(867324);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDoubleBuffer yoDoubleBuffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         YoDouble yoDouble = yoDoubleBuffer.getYoVariable();

         int currentIndex = yoDoubleBuffer.getProperties().getCurrentIndex();
         for (int j = 0; j < 10; j++)
         {
            double newValue = EuclidCoreRandomTools.nextDouble(random, 1000.0);
            yoDouble.set(newValue);
            yoDoubleBuffer.writeBuffer();
            assertEquals(newValue, yoDouble.getValue());
            assertEquals(newValue, yoDoubleBuffer.getBuffer()[currentIndex]);
         }
      }
   }

   @Test
   public void testReadBuffer()
   {
      Random random = new Random(867324);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDoubleBuffer yoDoubleBuffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         YoDouble yoDouble = yoDoubleBuffer.getYoVariable();

         int currentIndex = yoDoubleBuffer.getProperties().getCurrentIndex();
         for (int j = 0; j < 10; j++)
         {
            double newValue = EuclidCoreRandomTools.nextDouble(random, 1000.0);
            yoDoubleBuffer.getBuffer()[currentIndex] = newValue;
            yoDoubleBuffer.readBuffer();
            assertEquals(newValue, yoDouble.getValue());
            assertEquals(newValue, yoDoubleBuffer.getBuffer()[currentIndex]);
         }
      }
   }

   @Test
   public void testCopy()
   {
      Random random = new Random(43566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoDoubleBuffer yoDoubleBuffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         int from = random.nextInt(yoDoubleBuffer.getProperties().getSize());
         int length = random.nextInt(yoDoubleBuffer.getProperties().getSize() - 1) + 1;

         double[] expectedCopy = SharedMemoryTools.ringArrayCopy(yoDoubleBuffer.getBuffer(), from, length);
         BufferSample<double[]> actualCopy = yoDoubleBuffer.copy(from, length, yoDoubleBuffer.getProperties().copy());

         assertEquals(from, actualCopy.getFrom());
         assertEquals(length, actualCopy.getSampleLength());
         assertEquals(yoDoubleBuffer.getProperties(), actualCopy.getBufferProperties());
         int to = from + length - 1;
         if (to >= yoDoubleBuffer.getProperties().getSize())
            to -= yoDoubleBuffer.getProperties().getSize();
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
         YoDoubleBuffer yoDoubleBuffer = SharedMemoryRandomTools.nextYoDoubleBuffer(random, new YoRegistry("Dummy"));
         YoDouble linkedDouble = new YoDouble("linked", new YoRegistry("Dummy"));
         LinkedYoDouble linkedYoVariable = yoDoubleBuffer.newLinkedYoVariable(linkedDouble);
         assertTrue(linkedDouble == linkedYoVariable.getLinkedYoVariable());
         assertTrue(yoDoubleBuffer == linkedYoVariable.getBuffer());
      }
   }
}

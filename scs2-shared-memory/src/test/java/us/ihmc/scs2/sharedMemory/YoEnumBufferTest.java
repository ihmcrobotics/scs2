package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoEnumBufferTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructors()
   {
      Random random = new Random(467);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoEnum<?> yoEnum = SharedMemoryRandomTools.nextYoEnum(random, new YoRegistry("Dummy"));
         YoBufferProperties yoBufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoEnumBuffer<?> yoEnumBuffer = new YoEnumBuffer<>(yoEnum, yoBufferProperties);
         assertTrue(yoEnum == yoEnumBuffer.getYoVariable());
         assertTrue(yoBufferProperties == yoEnumBuffer.getProperties());
         assertEquals(yoBufferProperties.getSize(), yoEnumBuffer.getBuffer().length);

         for (int j = 0; j < yoBufferProperties.getSize(); j++)
            assertEquals((byte) 0, yoEnumBuffer.getBuffer()[j]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoEnum<?> yoEnum = SharedMemoryRandomTools.nextYoEnum(random, new YoRegistry("Dummy"));
         YoBufferProperties yoBufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoEnumBuffer<?> yoEnumBuffer = (YoEnumBuffer<?>) YoVariableBuffer.newYoVariableBuffer(yoEnum, yoBufferProperties);
         assertTrue(yoEnum == yoEnumBuffer.getYoVariable());
         assertTrue(yoBufferProperties == yoEnumBuffer.getProperties());
         assertEquals(yoBufferProperties.getSize(), yoEnumBuffer.getBuffer().length);

         for (int j = 0; j < yoBufferProperties.getSize(); j++)
            assertEquals((byte) 0, yoEnumBuffer.getBuffer()[j]);
      }
   }

   @Test
   public void testResizeBuffer()
   {
      Random random = new Random(8967254);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoEnumBuffer<?> yoEnumBuffer = SharedMemoryRandomTools.nextYoEnumBuffer(random, new YoRegistry("Dummy"));
         YoBufferProperties originalBufferProperties = new YoBufferProperties(yoEnumBuffer.getProperties());
         int from = random.nextInt(yoEnumBuffer.getProperties().getSize());
         int newLength = random.nextInt(yoEnumBuffer.getProperties().getSize());
         byte[] expectedBuffer = SharedMemoryTools.ringArrayCopy(yoEnumBuffer.getBuffer(), from, newLength);

         yoEnumBuffer.resizeBuffer(from, newLength);
         assertArrayEquals(expectedBuffer, yoEnumBuffer.getBuffer());
         assertEquals(originalBufferProperties, yoEnumBuffer.getProperties());

         byte[] buffer = yoEnumBuffer.getBuffer();
         yoEnumBuffer.resizeBuffer(0, buffer.length);
         assertTrue(buffer == yoEnumBuffer.getBuffer());
      }
   }

   @Test
   public void testWriteBuffer()
   {
      Random random = new Random(867324);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoEnumBuffer<?> yoEnumBuffer = SharedMemoryRandomTools.nextYoEnumBuffer(random, new YoRegistry("Dummy"));
         YoEnum<?> yoEnum = yoEnumBuffer.getYoVariable();

         int currentIndex = yoEnumBuffer.getProperties().getCurrentIndex();
         for (int j = 0; j < 10; j++)
         {
            int newValue = yoEnum.isNullAllowed() ? random.nextInt(yoEnum.getEnumSize() + 1) - 1 : random.nextInt(yoEnum.getEnumSize());
            yoEnum.set(newValue);
            yoEnumBuffer.writeBuffer();
            assertEquals(newValue, yoEnum.getOrdinal());
            assertEquals((byte) newValue, yoEnumBuffer.getBuffer()[currentIndex]);
         }
      }
   }

   @Test
   public void testReadBuffer()
   {
      Random random = new Random(867324);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoEnumBuffer<?> yoEnumBuffer = SharedMemoryRandomTools.nextYoEnumBuffer(random, new YoRegistry("Dummy"));
         YoEnum<?> yoEnum = yoEnumBuffer.getYoVariable();

         int currentIndex = yoEnumBuffer.getProperties().getCurrentIndex();
         for (int j = 0; j < 10; j++)
         {
            int newValue = yoEnum.isNullAllowed() ? random.nextInt(yoEnum.getEnumSize() + 1) - 1 : random.nextInt(yoEnum.getEnumSize());
            yoEnumBuffer.getBuffer()[currentIndex] = (byte) newValue;
            yoEnumBuffer.readBuffer();
            assertEquals(newValue, yoEnum.getOrdinal());
            assertEquals((byte) newValue, yoEnumBuffer.getBuffer()[currentIndex]);
         }
      }
   }

   @Test
   public void testCopy()
   {
      Random random = new Random(43566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoEnumBuffer<?> yoEnumBuffer = SharedMemoryRandomTools.nextYoEnumBuffer(random, new YoRegistry("Dummy"));
         int from = random.nextInt(yoEnumBuffer.getProperties().getSize());
         int length = random.nextInt(yoEnumBuffer.getProperties().getSize() - 1) + 1;

         byte[] expectedCopy = SharedMemoryTools.ringArrayCopy(yoEnumBuffer.getBuffer(), from, length);
         BufferSample<byte[]> actualCopy = yoEnumBuffer.copy(from, length, yoEnumBuffer.getProperties().copy());

         assertEquals(from, actualCopy.getFrom());
         assertEquals(length, actualCopy.getSampleLength());
         assertEquals(yoEnumBuffer.getProperties(), actualCopy.getBufferProperties());
         int to = from + length - 1;
         if (to >= yoEnumBuffer.getProperties().getSize())
            to -= yoEnumBuffer.getProperties().getSize();
         assertEquals(to, actualCopy.getTo());
         assertArrayEquals(expectedCopy, actualCopy.getSample());
      }
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   @Test
   public void testNewLinkedYoVariable()
   {
      Random random = new Random(87324);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoEnumBuffer yoEnumBuffer = SharedMemoryRandomTools.nextYoEnumBuffer(random, new YoRegistry("Dummy"));
         YoEnum linkedEnum = new YoEnum<>("linked",
                                          new YoRegistry("Dummy"),
                                          ((YoEnum) yoEnumBuffer.getYoVariable()).getEnumType(),
                                          ((YoEnum) yoEnumBuffer.getYoVariable()).isNullAllowed());
         LinkedYoEnum<?> linkedYoVariable = yoEnumBuffer.newLinkedYoVariable(linkedEnum);
         assertTrue(linkedEnum == linkedYoVariable.getLinkedYoVariable());
         assertTrue(yoEnumBuffer == linkedYoVariable.getBuffer());
      }
   }
}

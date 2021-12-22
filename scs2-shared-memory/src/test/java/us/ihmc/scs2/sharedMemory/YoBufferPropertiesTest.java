package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;

public class YoBufferPropertiesTest
{
   private static final int ITERATIONS = 100000;

   @Test
   public void testConstructors()
   {
      Random random = new Random(45346);

      { // Empty constructor
         YoBufferProperties yoBufferProperties = new YoBufferProperties();
         assertEquals(0, yoBufferProperties.getInPoint());
         assertEquals(0, yoBufferProperties.getOutPoint());
         assertEquals(0, yoBufferProperties.getCurrentIndex());
         assertEquals(0, yoBufferProperties.getSize());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // YoBufferProperties(int index, int size)
         int size = random.nextInt(1280000);
         int index = random.nextInt(size);
         YoBufferProperties yoBufferProperties = new YoBufferProperties(index, size);
         assertEquals(0, yoBufferProperties.getInPoint());
         assertEquals(0, yoBufferProperties.getOutPoint());
         assertEquals(index, yoBufferProperties.getCurrentIndex());
         assertEquals(size, yoBufferProperties.getSize());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBufferProperties original = SharedMemoryRandomTools.nextYoBufferProperties(random, random.nextInt(1280000));
         YoBufferProperties clone = new YoBufferProperties(original);
         assertEquals(original.getInPoint(), clone.getInPoint());
         assertEquals(original.getOutPoint(), clone.getOutPoint());
         assertEquals(original.getCurrentIndex(), clone.getCurrentIndex());
         assertEquals(original.getSize(), clone.getSize());
      }
   }

   @Test
   public void testIncrementIndex()
   {
      Random random = new Random(436);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test incrementIndex(updateBufferBounds == false)
         int size = random.nextInt(10000) + 1;
         int inPoint = random.nextInt(size);
         int index = random.nextInt(size);
         int outPoint = random.nextInt(size);

         YoBufferProperties properties = new YoBufferProperties(index, size);
         properties.setInPointIndex(inPoint);
         properties.setOutPointIndex(outPoint);

         int newIndex = properties.incrementIndex(false);
         assertEquals(newIndex, properties.getCurrentIndex());
         assertEquals(size, properties.getSize());
         assertEquals(inPoint, properties.getInPoint());
         assertEquals(outPoint, properties.getOutPoint());

         if (index == size - 1 && properties.isIndexBetweenBounds(0))
            assertEquals(0, newIndex);
         else if (properties.isIndexBetweenBounds(index) && properties.isIndexBetweenBounds(index + 1))
            assertEquals(index + 1, newIndex);
         else
            assertEquals(inPoint, newIndex);

         index = outPoint;
         properties.setCurrentIndexUnsafe(index);
         newIndex = properties.incrementIndex(false);
         assertEquals(inPoint, newIndex);

         index = size - 1;
         properties.setCurrentIndexUnsafe(index);
         newIndex = properties.incrementIndex(false);
         if (properties.isIndexBetweenBounds(size - 1) && properties.isIndexBetweenBounds(0))
            assertEquals(0, newIndex);
         else
            assertEquals(inPoint, newIndex);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test incrementIndex(updateBufferBounds == true)
         int size = random.nextInt(10000) + 2;
         int inPoint = random.nextInt(size);
         int outPoint = random.nextInt(size);
         int index = random.nextInt(size);
         String messagePrefix = "iteration " + i + ", index " + index + ", size " + size + ", in " + inPoint + ", out " + outPoint;

         YoBufferProperties properties = new YoBufferProperties(index, size);
         properties.setInPointIndex(inPoint);
         properties.setOutPointIndex(outPoint);

         int newIndex = properties.incrementIndex(true);
         assertEquals(newIndex, properties.getCurrentIndex(), messagePrefix);
         assertEquals(size, properties.getSize(), messagePrefix);
         assertEquals(newIndex, properties.getOutPoint());
         assertNotEquals(properties.getInPoint(), properties.getOutPoint());

         if (inPoint != newIndex)
            assertEquals(inPoint, properties.getInPoint(), messagePrefix);
         else
            assertEquals((inPoint + 1) % size, properties.getInPoint(), messagePrefix);

         if (index < size - 1)
            assertEquals(index + 1, newIndex, messagePrefix);
         else
            assertEquals(0, newIndex, messagePrefix);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test incrementIndex(updateBufferBounds, stepSize > 0)
         int size = random.nextInt(10000) + 1;
         int inPoint = random.nextInt(size);
         int index = random.nextInt(size);
         int outPoint = random.nextInt(size);
         int stepSize = random.nextInt(500);
         boolean updateBufferBounds = random.nextBoolean();

         YoBufferProperties actualProperties = new YoBufferProperties(index, size);
         actualProperties.setInPointIndex(inPoint);
         actualProperties.setOutPointIndex(outPoint);
         actualProperties.incrementIndex(updateBufferBounds, stepSize);

         YoBufferProperties expectedProperties = new YoBufferProperties(index, size);
         expectedProperties.setInPointIndex(inPoint);
         expectedProperties.setOutPointIndex(outPoint);
         for (int j = 0; j < stepSize; j++)
            expectedProperties.incrementIndex(updateBufferBounds);
         assertEquals(expectedProperties, actualProperties);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test incrementIndex(updateBufferBounds, stepSize < 0)
         int size = random.nextInt(10000) + 1;
         int inPoint = random.nextInt(size);
         int index = random.nextInt(size);
         int outPoint = random.nextInt(size);
         int stepSize = -random.nextInt(500);
         boolean updateBufferBounds = random.nextBoolean();

         YoBufferProperties actualProperties = new YoBufferProperties(index, size);
         actualProperties.setInPointIndex(inPoint);
         actualProperties.setOutPointIndex(outPoint);
         actualProperties.incrementIndex(updateBufferBounds, stepSize);

         YoBufferProperties expectedProperties = new YoBufferProperties(index, size);
         expectedProperties.setInPointIndex(inPoint);
         expectedProperties.setOutPointIndex(outPoint);
         for (int j = 0; j < Math.abs(stepSize); j++)
            expectedProperties.decrementIndex();
         assertEquals(expectedProperties, actualProperties);
      }
   }

   @Test
   public void testDecrementIndex()
   {
      Random random = new Random(4346547);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test decrementIndex()
         int size = random.nextInt(10000) + 1;
         int inPoint = random.nextInt(size);
         int index = random.nextInt(size);
         int outPoint = random.nextInt(size);

         YoBufferProperties properties = new YoBufferProperties(index, size);
         properties.setInPointIndex(inPoint);
         properties.setOutPointIndex(outPoint);

         int newIndex = properties.decrementIndex();
         assertEquals(newIndex, properties.getCurrentIndex());
         assertEquals(size, properties.getSize());
         assertEquals(inPoint, properties.getInPoint());
         assertEquals(outPoint, properties.getOutPoint());

         if (index == 0 && properties.isIndexBetweenBounds(size - 1))
            assertEquals(size - 1, newIndex);
         else if (properties.isIndexBetweenBounds(index) && properties.isIndexBetweenBounds(index - 1))
            assertEquals(index - 1, newIndex);
         else
            assertEquals(outPoint, newIndex);

         index = inPoint;
         properties.setCurrentIndexUnsafe(index);
         newIndex = properties.decrementIndex();
         assertEquals(outPoint, newIndex);

         index = 0;
         properties.setCurrentIndexUnsafe(index);
         newIndex = properties.decrementIndex();
         if (properties.isIndexBetweenBounds(size - 1) && properties.isIndexBetweenBounds(0))
            assertEquals(size - 1, newIndex);
         else
            assertEquals(outPoint, newIndex);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test decrementIndex(stepSize > 0)
         int size = random.nextInt(10000) + 1;
         int inPoint = random.nextInt(size);
         int index = random.nextInt(size);
         int outPoint = random.nextInt(size);
         int stepSize = random.nextInt(500);

         YoBufferProperties actualProperties = new YoBufferProperties(index, size);
         actualProperties.setInPointIndex(inPoint);
         actualProperties.setOutPointIndex(outPoint);
         actualProperties.decrementIndex(stepSize);

         YoBufferProperties expectedProperties = new YoBufferProperties(index, size);
         expectedProperties.setInPointIndex(inPoint);
         expectedProperties.setOutPointIndex(outPoint);
         for (int j = 0; j < stepSize; j++)
            expectedProperties.decrementIndex();
         assertEquals(expectedProperties, actualProperties);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test decrementIndex(stepSize < 0)
         int size = random.nextInt(10000) + 1;
         int inPoint = random.nextInt(size);
         int index = random.nextInt(size);
         int outPoint = random.nextInt(size);
         int stepSize = -random.nextInt(500);

         YoBufferProperties actualProperties = new YoBufferProperties(index, size);
         actualProperties.setInPointIndex(inPoint);
         actualProperties.setOutPointIndex(outPoint);
         actualProperties.decrementIndex(stepSize);

         YoBufferProperties expectedProperties = new YoBufferProperties(index, size);
         expectedProperties.setInPointIndex(inPoint);
         expectedProperties.setOutPointIndex(outPoint);
         for (int j = 0; j < Math.abs(stepSize); j++)
            expectedProperties.incrementIndex(false);
         assertEquals(expectedProperties, actualProperties);
      }
   }

   @Test
   public void testSetCurrentIndex()
   {
      Random random = new Random(4677);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(10000) + 1;
         int index = random.nextInt(size);

         YoBufferProperties actual = new YoBufferProperties(index, size);
         YoBufferProperties expected = new YoBufferProperties(actual);

         assertFalse(actual.setCurrentIndex(index));
         assertEquals(actual, expected);
         assertFalse(actual.setCurrentIndex(-random.nextInt(1000) - 1));
         assertEquals(actual, expected);
         assertFalse(actual.setCurrentIndex(size + random.nextInt(1000)));
         assertEquals(actual, expected);

         if (size > 1)
         {
            int newIndex = random.nextInt(size);
            while (newIndex == index)
               newIndex = random.nextInt(size);

            assertTrue(actual.setCurrentIndex(newIndex));
            assertEquals(newIndex, actual.getCurrentIndex());
            assertEquals(expected.getSize(), actual.getSize());
            assertEquals(expected.getInPoint(), actual.getInPoint());
            assertEquals(expected.getOutPoint(), actual.getOutPoint());
         }
      }
   }

   @Test
   public void testSetCurrentIndexUnsafe()
   {
      Random random = new Random(9746);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBufferProperties original = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoBufferProperties clone = new YoBufferProperties(original);

         int newIndex = RandomNumbers.nextInt(random, -5000000, 5000000);
         clone.setCurrentIndexUnsafe(newIndex);

         assertEquals(newIndex, clone.getCurrentIndex());
         assertEquals(original.getSize(), clone.getSize());
         assertEquals(original.getInPoint(), clone.getInPoint());
         assertEquals(original.getOutPoint(), clone.getOutPoint());
      }
   }

   @Test
   public void testSetSize()
   {
      Random random = new Random(9746);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBufferProperties original = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoBufferProperties clone = new YoBufferProperties(original);

         int newSize = RandomNumbers.nextInt(random, 1, 5000000);
         clone.setSize(newSize);

         assertEquals(newSize, clone.getSize());
         assertEquals(original.getCurrentIndex(), clone.getCurrentIndex());
         assertEquals(original.getInPoint(), clone.getInPoint());
         assertEquals(original.getOutPoint(), clone.getOutPoint());

         assertThrows(IllegalArgumentException.class, () -> clone.setSize(-random.nextInt(1000)));
      }
   }

   @Test
   public void testSetInPoint()
   {
      Random random = new Random(4677);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(10000) + 1;

         YoBufferProperties actual = SharedMemoryRandomTools.nextYoBufferProperties(random, size);
         YoBufferProperties expected = new YoBufferProperties(actual);

         assertFalse(actual.setInPointIndex(actual.getInPoint()));
         assertEquals(actual, expected);
         assertFalse(actual.setInPointIndex(-random.nextInt(1000) - 1));
         assertEquals(actual, expected);
         assertFalse(actual.setInPointIndex(size + random.nextInt(1000)));
         assertEquals(actual, expected);

         if (size > 1)
         {
            int newInPoint = random.nextInt(size);
            while (newInPoint == actual.getInPoint())
               newInPoint = random.nextInt(size);

            assertTrue(actual.setInPointIndex(newInPoint));
            assertEquals(expected.getCurrentIndex(), actual.getCurrentIndex());
            assertEquals(expected.getSize(), actual.getSize());
            assertEquals(newInPoint, actual.getInPoint());
            assertEquals(expected.getOutPoint(), actual.getOutPoint());
         }
      }
   }

   @Test
   public void testSetOutPoint()
   {
      Random random = new Random(4677);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(10000) + 1;

         YoBufferProperties actual = SharedMemoryRandomTools.nextYoBufferProperties(random, size);
         YoBufferProperties expected = new YoBufferProperties(actual);

         assertFalse(actual.setOutPointIndex(actual.getOutPoint()));
         assertEquals(actual, expected);
         assertFalse(actual.setOutPointIndex(-random.nextInt(1000) - 1));
         assertEquals(actual, expected);
         assertFalse(actual.setOutPointIndex(size + random.nextInt(1000)));
         assertEquals(actual, expected);

         if (size > 1)
         {
            int newOutPoint = random.nextInt(size);
            while (newOutPoint == actual.getOutPoint())
               newOutPoint = random.nextInt(size);

            assertTrue(actual.setOutPointIndex(newOutPoint));
            assertEquals(expected.getCurrentIndex(), actual.getCurrentIndex());
            assertEquals(expected.getSize(), actual.getSize());
            assertEquals(expected.getInPoint(), actual.getInPoint());
            assertEquals(newOutPoint, actual.getOutPoint());
         }
      }
   }

   @Test
   public void testGetActiveBufferLength()
   {
      Random random = new Random(63234);

      for (int i = 0; i < ITERATIONS; i++)
      { // Use naive approach to measure the length
         YoBufferProperties properties = SharedMemoryRandomTools.nextYoBufferProperties(random);

         int index = properties.getInPoint();
         int expectedLength = 1;

         while (index != properties.getOutPoint())
         {
            index++;
            if (index >= properties.getSize())
               index = 0;
            expectedLength++;
         }

         assertEquals(expectedLength, properties.getActiveBufferLength());
      }
   }

   @Test
   public void testIsIndexBetweenBounds()
   {
      Random random = new Random(4367);

      for (int i = 0; i < ITERATIONS; i++)
      { // in-point < out-point
         int size = random.nextInt(10000) + 1;
         int inPoint = random.nextInt(size);
         int outPoint = RandomNumbers.nextInt(random, inPoint, size - 1);

         int indexInside = RandomNumbers.nextInt(random, inPoint, outPoint);
         int indexOutside;
         if (random.nextBoolean() && outPoint < size - 1)
            indexOutside = RandomNumbers.nextInt(random, outPoint + 1, size - 1);
         else if (inPoint > 0)
            indexOutside = RandomNumbers.nextInt(random, 0, inPoint - 1);
         else
            indexOutside = -1;

         YoBufferProperties properties = new YoBufferProperties();
         properties.setSize(size);
         properties.setInPointIndex(inPoint);
         properties.setOutPointIndex(outPoint);
         assertTrue(properties.isIndexBetweenBounds(indexInside));
         assertFalse(properties.isIndexBetweenBounds(indexOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // in-point > out-point
         int size = random.nextInt(10000) + 2;
         int outPoint = random.nextInt(size - 1);
         int inPoint = RandomNumbers.nextInt(random, outPoint + 1, size - 1);

         int indexInside;
         if (random.nextBoolean())
            indexInside = RandomNumbers.nextInt(random, 0, outPoint);
         else
            indexInside = RandomNumbers.nextInt(random, inPoint, size - 1);
         int indexOutside;
         if (outPoint < inPoint - 1)
            indexOutside = RandomNumbers.nextInt(random, outPoint + 1, inPoint - 1);
         else
            indexOutside = -1;

         YoBufferProperties properties = new YoBufferProperties();
         properties.setSize(size);
         properties.setInPointIndex(inPoint);
         properties.setOutPointIndex(outPoint);
         assertTrue(properties.isIndexBetweenBounds(indexInside));
         assertFalse(properties.isIndexBetweenBounds(indexOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // in-point == out-point
         int size = random.nextInt(10000) + 1;
         int inPoint = random.nextInt(size);
         int outPoint = inPoint;

         int indexInside = inPoint;
         int indexOutside;
         if (outPoint < size - 1)
            indexOutside = RandomNumbers.nextInt(random, outPoint + 1, size - 1);
         else if (inPoint > 0)
            indexOutside = RandomNumbers.nextInt(random, 0, inPoint - 1);
         else
            indexOutside = -1;

         YoBufferProperties properties = new YoBufferProperties();
         properties.setSize(size);
         properties.setInPointIndex(inPoint);
         properties.setOutPointIndex(outPoint);
         assertTrue(properties.isIndexBetweenBounds(indexInside));
         assertFalse(properties.isIndexBetweenBounds(indexOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // index out of buffer size
         int size = random.nextInt(10000) + 1;

         YoBufferProperties properties = new YoBufferProperties();
         properties.setSize(size);
         properties.setInPointIndex(random.nextInt(size));
         properties.setOutPointIndex(random.nextInt(size));

         int indexOutside = -random.nextInt(100) - 1;
         assertFalse(properties.isIndexBetweenBounds(indexOutside));
         indexOutside = size + random.nextInt(100);
         assertFalse(properties.isIndexBetweenBounds(indexOutside));
      }
   }

   @Test
   public void testCopy()
   {
      Random random = new Random(5346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBufferProperties original = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoBufferPropertiesReadOnly copy = original.copy();

         assertEquals(original.getInPoint(), copy.getInPoint());
         assertEquals(original.getOutPoint(), copy.getOutPoint());
         assertEquals(original.getCurrentIndex(), copy.getCurrentIndex());
         assertEquals(original.getSize(), copy.getSize());
      }
   }

   @Test
   public void testEquals()
   {
      Random random = new Random(6577);

      for (int i = 0; i < ITERATIONS; i++)
      {
         TestProperties testProperties = new TestProperties(random);
         YoBufferProperties properties1 = new YoBufferProperties(testProperties);
         YoBufferProperties properties2 = new YoBufferProperties(testProperties);

         assertTrue(testProperties.equals(properties1));
         assertTrue(properties1.equals(properties2));
         assertTrue(properties1.equals((YoBufferPropertiesReadOnly) properties2));
         assertTrue(properties1.equals(properties1));
         assertFalse(properties1.equals(null));
         assertFalse(properties1.equals(new ArrayList<>()));

         testProperties.inPoint++;
         properties2.set(testProperties);
         assertFalse(testProperties.equals(properties1));
         assertFalse(properties1.equals(properties2));
         assertTrue(testProperties.equals(properties2));

         testProperties.inPoint--;
         testProperties.outPoint++;
         properties2.set(testProperties);
         assertFalse(testProperties.equals(properties1));
         assertFalse(properties1.equals(properties2));
         assertTrue(testProperties.equals(properties2));

         testProperties.outPoint--;
         testProperties.currentIndex++;
         properties2.set(testProperties);
         assertFalse(testProperties.equals(properties1));
         assertFalse(properties1.equals(properties2));
         assertTrue(testProperties.equals(properties2));

         testProperties.currentIndex--;
         testProperties.size++;
         properties2.set(testProperties);
         assertFalse(testProperties.equals(properties1));
         assertFalse(properties1.equals(properties2));
         assertTrue(testProperties.equals(properties2));
      }
   }

   private static class TestProperties implements YoBufferPropertiesReadOnly
   {
      private int inPoint = 0;
      private int outPoint = 0;

      private int currentIndex = 0;
      private int size = 0;

      public TestProperties(Random random)
      {
         inPoint = random.nextInt(416516516);
         outPoint = random.nextInt(416516516);
         currentIndex = random.nextInt(416516516);
         size = random.nextInt(416516516);
      }

      @Override
      public int getCurrentIndex()
      {
         return currentIndex;
      }

      @Override
      public int getSize()
      {
         return size;
      }

      @Override
      public int getInPoint()
      {
         return inPoint;
      }

      @Override
      public int getOutPoint()
      {
         return outPoint;
      }
   }
}

package us.ihmc.scs2.sharedMemory.tools;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Arrays;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class SharedMemoryToolsTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testComputeSubLength()
   {
      Random random = new Random(34536);

      { // Test exceptions
         assertThrows(IllegalArgumentException.class, () -> SharedMemoryTools.computeSubLength(0, 10, 0));
         assertThrows(IllegalArgumentException.class, () -> SharedMemoryTools.computeSubLength(0, 10, -1));
         assertThrows(IndexOutOfBoundsException.class, () -> SharedMemoryTools.computeSubLength(0, 10, 5));
         assertThrows(IndexOutOfBoundsException.class, () -> SharedMemoryTools.computeSubLength(5, 4, 5));
         assertThrows(IndexOutOfBoundsException.class, () -> SharedMemoryTools.computeSubLength(-1, 4, 5));
         assertThrows(IndexOutOfBoundsException.class, () -> SharedMemoryTools.computeSubLength(0, -1, 5));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int length = random.nextInt(1000) + 1;
         int from = random.nextInt(length);
         int to = random.nextInt(length);
         int subLength = SharedMemoryTools.computeSubLength(from, to, length);

         // Use naive approach to measure the length

         int index = from;
         int expectedSubLength = 1;

         while (index != to)
         {
            index++;
            if (index >= length)
               index = 0;
            expectedSubLength++;
         }

         assertEquals(expectedSubLength, subLength);
      }
   }

   @Test
   public void testComputeFromIndex()
   {
      Random random = new Random(2456);

      { // Test exceptions
         assertThrows(IllegalArgumentException.class, () -> SharedMemoryTools.computeFromIndex(9, 1, 0));
         assertThrows(IllegalArgumentException.class, () -> SharedMemoryTools.computeFromIndex(9, 0, 1));
         assertThrows(IllegalArgumentException.class, () -> SharedMemoryTools.computeFromIndex(9, 2, 1));
         assertThrows(IndexOutOfBoundsException.class, () -> SharedMemoryTools.computeFromIndex(11, 10, 11));
         assertThrows(IndexOutOfBoundsException.class, () -> SharedMemoryTools.computeFromIndex(-1, 10, 11));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int length = random.nextInt(1000) + 1;
         int to = random.nextInt(length);
         int subLength = random.nextInt(length - 1) + 1;
         int from = SharedMemoryTools.computeFromIndex(to, subLength, length);

         assertEquals(subLength, SharedMemoryTools.computeSubLength(from, to, length));
      }
   }

   @Test
   public void testComputeToIndex()
   {
      Random random = new Random(2456);

      { // Test exceptions
         assertThrows(IllegalArgumentException.class, () -> SharedMemoryTools.computeToIndex(9, 1, 0));
         assertThrows(IllegalArgumentException.class, () -> SharedMemoryTools.computeToIndex(9, 0, 1));
         assertThrows(IllegalArgumentException.class, () -> SharedMemoryTools.computeToIndex(9, 2, 1));
         assertThrows(IndexOutOfBoundsException.class, () -> SharedMemoryTools.computeToIndex(11, 10, 11));
         assertThrows(IndexOutOfBoundsException.class, () -> SharedMemoryTools.computeToIndex(-1, 10, 11));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int length = random.nextInt(1000) + 1;
         int from = random.nextInt(length);
         int subLength = random.nextInt(length - 1) + 1;
         int to = SharedMemoryTools.computeToIndex(from, subLength, length);

         assertEquals(subLength, SharedMemoryTools.computeSubLength(from, to, length));
      }
   }

   @Test
   public void testRingArrayCopyWithBoolean()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test full copy.
         int length = random.nextInt(1000) + 1;
         boolean[] source = nextBooleanArray(random, length);
         boolean[] copy = SharedMemoryTools.ringArrayCopy(source, 0, length);
         assertArrayEquals(source, copy);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test partial copy without going over the source's length.
         int length = random.nextInt(1000) + 1;
         boolean[] source = nextBooleanArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(length - from);
         boolean[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[k + from])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test full copy starting at random index.
         int length = random.nextInt(1000) + 1;
         int from = random.nextInt(length);
         boolean[] source = nextBooleanArray(random, length);
         boolean[] copy = SharedMemoryTools.ringArrayCopy(source, from, length);

         for (int k = 0; k < length; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + "\nCopy: " + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test partial copy going over the source's length.
         int length = random.nextInt(1000) + 1;
         boolean[] source = nextBooleanArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(from + 1) + length - from;
         boolean[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }
   }

   @Test
   public void testRingArrayCopyWithDouble()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test full copy.
         int length = random.nextInt(1000) + 1;
         double[] source = nextDoubleArray(random, length);
         double[] copy = SharedMemoryTools.ringArrayCopy(source, 0, length);
         assertArrayEquals(source, copy);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test partial copy without going over the source's length.
         int length = random.nextInt(1000) + 1;
         double[] source = nextDoubleArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(length - from);
         double[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[k + from])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test full copy starting at random index.
         int length = random.nextInt(1000) + 1;
         int from = random.nextInt(length);
         double[] source = nextDoubleArray(random, length);
         double[] copy = SharedMemoryTools.ringArrayCopy(source, from, length);

         for (int k = 0; k < length; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + "\nCopy: " + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test partial copy going over the source's length.
         int length = random.nextInt(1000) + 1;
         double[] source = nextDoubleArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(from + 1) + length - from;
         double[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }
   }

   @Test
   public void testRingArrayCopyWithInteger()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test full copy.
         int length = random.nextInt(1000) + 1;
         int[] source = nextIntegerArray(random, length);
         int[] copy = SharedMemoryTools.ringArrayCopy(source, 0, length);
         assertArrayEquals(source, copy);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test partial copy without going over the source's length.
         int length = random.nextInt(1000) + 1;
         int[] source = nextIntegerArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(length - from);
         int[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[k + from])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test full copy starting at random index.
         int length = random.nextInt(1000) + 1;
         int from = random.nextInt(length);
         int[] source = nextIntegerArray(random, length);
         int[] copy = SharedMemoryTools.ringArrayCopy(source, from, length);

         for (int k = 0; k < length; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + "\nCopy: " + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test partial copy going over the source's length.
         int length = random.nextInt(1000) + 1;
         int[] source = nextIntegerArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(from + 1) + length - from;
         int[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }
   }

   @Test
   public void testRingArrayCopyWithLong()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test full copy.
         int length = random.nextInt(1000) + 1;
         long[] source = nextLongArray(random, length);
         long[] copy = SharedMemoryTools.ringArrayCopy(source, 0, length);
         assertArrayEquals(source, copy);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test partial copy without going over the source's length.
         int length = random.nextInt(1000) + 1;
         long[] source = nextLongArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(length - from);
         long[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[k + from])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test full copy starting at random index.
         int length = random.nextInt(1000) + 1;
         int from = random.nextInt(length);
         long[] source = nextLongArray(random, length);
         long[] copy = SharedMemoryTools.ringArrayCopy(source, from, length);

         for (int k = 0; k < length; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + "\nCopy: " + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test partial copy going over the source's length.
         int length = random.nextInt(1000) + 1;
         long[] source = nextLongArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(from + 1) + length - from;
         long[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }
   }

   @Test
   public void testRingArrayCopyWithByte()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test full copy.
         int length = random.nextInt(1000) + 1;
         byte[] source = nextByteArray(random, length);
         byte[] copy = SharedMemoryTools.ringArrayCopy(source, 0, length);
         assertArrayEquals(source, copy);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial example: Test partial copy without going over the source's length.
         int length = random.nextInt(1000) + 1;
         byte[] source = nextByteArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(length - from);
         byte[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[k + from])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test full copy starting at random index.
         int length = random.nextInt(1000) + 1;
         int from = random.nextInt(length);
         byte[] source = nextByteArray(random, length);
         byte[] copy = SharedMemoryTools.ringArrayCopy(source, from, length);

         for (int k = 0; k < length; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + "\nCopy: " + Arrays.toString(copy));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Circular example: Test partial copy going over the source's length.
         int length = random.nextInt(1000) + 1;
         byte[] source = nextByteArray(random, length);
         int from = random.nextInt(length);
         int copyLength = random.nextInt(from + 1) + length - from;
         byte[] copy = SharedMemoryTools.ringArrayCopy(source, from, copyLength);

         for (int k = 0; k < copyLength; k++)
         {
            if (copy[k] != source[(k + from) % length])
               throw new AssertionError("Source: " + Arrays.toString(source) + "\nfrom=" + from + ", copyLength=" + copyLength + "\nCopy: "
                     + Arrays.toString(copy));
         }
      }
   }

   public static boolean[] nextBooleanArray(Random random, int length)
   {
      boolean[] next = new boolean[length];
      for (int i = 0; i < length; i++)
         next[i] = random.nextBoolean();
      return next;
   }

   public static double[] nextDoubleArray(Random random, int length)
   {
      double[] next = new double[length];
      for (int i = 0; i < length; i++)
         next[i] = EuclidCoreRandomTools.nextDouble(random);
      return next;
   }

   public static int[] nextIntegerArray(Random random, int length)
   {
      int[] next = new int[length];
      for (int i = 0; i < length; i++)
         next[i] = RandomNumbers.nextInt(random, -100000, 100000);
      return next;
   }

   public static long[] nextLongArray(Random random, int length)
   {
      long[] next = new long[length];
      for (int i = 0; i < length; i++)
         next[i] = RandomNumbers.nextInt(random, -100000, 100000);
      return next;
   }

   public static byte[] nextByteArray(Random random, int length)
   {
      byte[] next = new byte[length];
      random.nextBytes(next);
      return next;
   }

   @Test
   public void testDuplicateMissingYoVariablesInTarget()
   {
      Random random = new Random(469);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with single registry
         int numberOfVariables = RandomNumbers.nextInt(random, 0, 100);
         YoRegistry original = SharedMemoryRandomTools.nextYoRegistry(random, numberOfVariables);
         YoRegistry target = new YoRegistry(SharedMemoryRandomTools.nextAlphanumericString(random, 1, 50));
         int numberOfYoVariablesCreated = SharedMemoryTools.duplicateMissingYoVariablesInTarget(original, target);

         assertEquals(numberOfVariables, numberOfYoVariablesCreated);

         for (YoVariable originalVariable : original.getVariables())
         {
            YoVariable targetVariable = target.findVariable(originalVariable.getName());
            assertNotNull(targetVariable);
            assertEquals(originalVariable.getClass(), targetVariable.getClass());
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test duplicating entire registry tree
         int numberOfVariables = RandomNumbers.nextInt(random, 0, 100);
         YoRegistry originalRoot = SharedMemoryRandomTools.nextYoRegistryTree(random, numberOfVariables, 50)[0];
         YoRegistry targetRoot = new YoRegistry(originalRoot.getName());
         int numberOfYoVariablesCreated = SharedMemoryTools.duplicateMissingYoVariablesInTarget(originalRoot, targetRoot);

         assertEquals(originalRoot.collectSubtreeVariables().size(), numberOfYoVariablesCreated);

         for (YoRegistry originalRegistry : originalRoot.collectSubtreeRegistries())
         {
            YoRegistry targetRegistry = targetRoot.findRegistry(originalRegistry.getNamespace());
            assertNotNull(targetRegistry);
            assertEquals(originalRegistry.getNumberOfVariables(), targetRegistry.getNumberOfVariables());
         }

         for (YoVariable originalVariable : originalRoot.collectSubtreeVariables())
         {
            YoVariable targetVariable = targetRoot.findVariable(originalVariable.getNamespace().toString(), originalVariable.getName());
            assertNotNull(targetVariable);
            assertEquals(originalVariable.getClass(), targetVariable.getClass());
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test completing single registry
         int numberOfVariables = RandomNumbers.nextInt(random, 0, 50);
         YoRegistry original = SharedMemoryRandomTools.nextYoRegistry(random, numberOfVariables);
         YoRegistry target = new YoRegistry(SharedMemoryRandomTools.nextAlphanumericString(random, 1, 50));
         SharedMemoryTools.duplicateMissingYoVariablesInTarget(original, target);

         int numberOfMissingVariables = RandomNumbers.nextInt(random, 0, 50);
         SharedMemoryRandomTools.nextYoVariables(random, numberOfMissingVariables, original);

         int numberOfYoVariablesCreated = SharedMemoryTools.duplicateMissingYoVariablesInTarget(original, target);
         assertEquals(numberOfMissingVariables, numberOfYoVariablesCreated);

         for (YoVariable originalVariable : original.getVariables())
         {
            YoVariable targetVariable = target.findVariable(originalVariable.getName());
            assertNotNull(targetVariable);
            assertEquals(originalVariable.getClass(), targetVariable.getClass());
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test completing registry tree
         int numberOfVariables = RandomNumbers.nextInt(random, 0, 50);
         YoRegistry[] originalRegistries = SharedMemoryRandomTools.nextYoRegistryTree(random, numberOfVariables, 25);
         YoRegistry originalRoot = originalRegistries[0];
         YoRegistry targetRoot = new YoRegistry(originalRoot.getName());
         SharedMemoryTools.duplicateMissingYoVariablesInTarget(originalRoot, targetRoot);

         int numberOfMissingVariables = 0;

         for (int j = 0; j < 25; j++)
         {
            int n = RandomNumbers.nextInt(random, 0, 50);
            YoRegistry parent = originalRegistries[random.nextInt(originalRegistries.length)];
            YoRegistry registry = SharedMemoryRandomTools.nextYoRegistry(random, SharedMemoryRandomTools.nextAvailableRegistryName(random, 1, 50, parent), n);
            parent.addChild(registry);
            numberOfMissingVariables += n;
         }

         int numberOfYoVariablesCreated = SharedMemoryTools.duplicateMissingYoVariablesInTarget(originalRoot, targetRoot);
         assertEquals(numberOfMissingVariables, numberOfYoVariablesCreated);

         for (YoRegistry originalRegistry : originalRoot.collectSubtreeRegistries())
         {
            YoRegistry targetRegistry = targetRoot.findRegistry(originalRegistry.getNamespace());
            assertNotNull(targetRegistry);
            assertEquals(originalRegistry.getNumberOfVariables(), targetRegistry.getNumberOfVariables());
         }

         for (YoVariable originalVariable : originalRoot.collectSubtreeVariables())
         {
            YoVariable targetVariable = targetRoot.findVariable(originalVariable.getNamespace().toString(), originalVariable.getName());
            assertNotNull(targetVariable);
            assertEquals(originalVariable.getClass(), targetVariable.getClass());
         }
      }
   }
}

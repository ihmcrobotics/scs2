package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;

public class BufferSampleTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructor()
   {
      Random random = new Random(9782);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBufferProperties bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         int from = random.nextInt(bufferProperties.getSize());
         int sampleLength = random.nextInt(bufferProperties.getSize() - 1) + 1;
         double[] sample = new double[sampleLength];
         for (int j = 0; j < sampleLength; j++)
            sample[j] = random.nextDouble();
         BufferSample<double[]> bufferSample = new BufferSample<>(from, sample, sampleLength, bufferProperties);

         assertEquals(from, bufferSample.getFrom());
         int to = from + sampleLength - 1;
         if (to >= bufferProperties.getSize())
            to -= bufferProperties.getSize();
         assertEquals(to, bufferSample.getTo());
         assertEquals(bufferProperties, bufferSample.getBufferProperties());
         assertTrue(sample == bufferSample.getSample());
         assertEquals(sampleLength, bufferSample.getSampleLength());
      }
   }

   @Test
   public void testEquals()
   {
      Random random = new Random(63892);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoBufferPropertiesReadOnly bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         int from = random.nextInt(bufferProperties.getSize());
         int sampleLength = random.nextInt(bufferProperties.getSize() - 1) + 1;

         boolean[] booleans = new boolean[sampleLength];
         double[] doubles = new double[sampleLength];
         int[] ints = new int[sampleLength];
         long[] longs = new long[sampleLength];
         byte[] bytes = new byte[sampleLength];

         for (int j = 0; j < sampleLength; j++)
         {
            booleans[j] = random.nextBoolean();
            doubles[j] = random.nextDouble();
            ints[j] = random.nextInt();
            longs[j] = random.nextInt();
            bytes[j] = (byte) random.nextInt();
         }

         YoBufferPropertiesReadOnly bufferProperties2 = SharedMemoryRandomTools.nextYoBufferProperties(random);

         BufferSample<boolean[]> booleanSample = new BufferSample<>(from, booleans, sampleLength, bufferProperties);
         BufferSample<double[]> doubleSample = new BufferSample<>(from, doubles, sampleLength, bufferProperties);
         BufferSample<int[]> intSample = new BufferSample<>(from, ints, sampleLength, bufferProperties);
         BufferSample<long[]> longSample = new BufferSample<>(from, longs, sampleLength, bufferProperties);
         BufferSample<byte[]> byteSample = new BufferSample<>(from, bytes, sampleLength, bufferProperties);

         assertFalse(booleanSample.equals(null));
         assertTrue(booleanSample.equals(booleanSample));
         assertTrue(booleanSample.equals(new BufferSample<>(booleanSample)));
         assertFalse(booleanSample.equals(new BufferSample<>(from < bufferProperties.getSize() - 1 ? from + 1 : from - 1,
                                                             booleans,
                                                             sampleLength,
                                                             bufferProperties)));
         assertFalse(booleanSample.equals(new BufferSample<>(MathTools.clamp(from, 0, bufferProperties2.getSize() - 1),
                                                             booleans,
                                                             MathTools.clamp(sampleLength, 0, bufferProperties2.getSize()),
                                                             bufferProperties2)));
         assertFalse(booleanSample.equals(new BufferSample<>(from, booleans, sampleLength + 1, bufferProperties)));
         assertFalse(booleanSample.equals(new Object()));

         assertFalse(doubleSample.equals(null));
         assertTrue(doubleSample.equals(doubleSample));
         assertTrue(doubleSample.equals(new BufferSample<>(doubleSample)));
         assertFalse(doubleSample.equals(new BufferSample<>(from < bufferProperties.getSize() - 1 ? from + 1 : from - 1,
                                                            doubles,
                                                            sampleLength,
                                                            bufferProperties)));
         assertFalse(doubleSample.equals(new BufferSample<>(MathTools.clamp(from, 0, bufferProperties2.getSize() - 1),
                                                            doubles,
                                                            MathTools.clamp(sampleLength, 0, bufferProperties2.getSize()),
                                                            bufferProperties2)));
         assertFalse(doubleSample.equals(new BufferSample<>(from, doubles, sampleLength + 1, bufferProperties)));
         assertFalse(doubleSample.equals(new Object()));

         assertFalse(intSample.equals(null));
         assertTrue(intSample.equals(intSample));
         assertTrue(intSample.equals(new BufferSample<>(intSample)));
         assertFalse(intSample.equals(new BufferSample<>(from < bufferProperties.getSize() - 1 ? from + 1 : from - 1, ints, sampleLength, bufferProperties)));
         assertFalse(intSample.equals(new BufferSample<>(MathTools.clamp(from, 0, bufferProperties2.getSize() - 1),
                                                         ints,
                                                         MathTools.clamp(sampleLength, 0, bufferProperties2.getSize()),
                                                         bufferProperties2)));
         assertFalse(intSample.equals(new BufferSample<>(from, ints, sampleLength + 1, bufferProperties)));
         assertFalse(intSample.equals(new Object()));

         assertFalse(longSample.equals(null));
         assertTrue(longSample.equals(longSample));
         assertTrue(longSample.equals(new BufferSample<>(longSample)));
         assertFalse(longSample.equals(new BufferSample<>(from < bufferProperties.getSize() - 1 ? from + 1 : from - 1, longs, sampleLength, bufferProperties)));
         assertFalse(longSample.equals(new BufferSample<>(MathTools.clamp(from, 0, bufferProperties2.getSize() - 1),
                                                          longs,
                                                          MathTools.clamp(sampleLength, 0, bufferProperties2.getSize()),
                                                          bufferProperties2)));
         assertFalse(longSample.equals(new BufferSample<>(from, longs, sampleLength + 1, bufferProperties)));
         assertFalse(longSample.equals(new Object()));

         assertFalse(byteSample.equals(null));
         assertTrue(byteSample.equals(byteSample));
         assertTrue(byteSample.equals(new BufferSample<>(byteSample)));
         assertFalse(byteSample.equals(new BufferSample<>(from < bufferProperties.getSize() - 1 ? from + 1 : from - 1, bytes, sampleLength, bufferProperties)));
         assertFalse(byteSample.equals(new BufferSample<>(MathTools.clamp(from, 0, bufferProperties2.getSize() - 1),
                                                          bytes,
                                                          MathTools.clamp(sampleLength, 0, bufferProperties2.getSize()),
                                                          bufferProperties2)));
         assertFalse(byteSample.equals(new BufferSample<>(from, bytes, sampleLength + 1, bufferProperties)));
         assertFalse(byteSample.equals(new Object()));

         if (sampleLength > 0)
         {
            int index = random.nextInt(sampleLength);

            boolean[] booleans2 = Arrays.copyOf(booleans, sampleLength);
            booleans2[index] = !booleans[index];
            double[] doubles2 = Arrays.copyOf(doubles, sampleLength);
            doubles2[index] += 1.0;
            int[] ints2 = Arrays.copyOf(ints, sampleLength);
            ints2[index] += 1;
            long[] longs2 = Arrays.copyOf(longs, sampleLength);
            longs2[index] += 1L;
            byte[] bytes2 = Arrays.copyOf(bytes, sampleLength);
            bytes2[index] += (byte) 1;

            assertFalse(booleanSample.equals(new BufferSample<>(from, booleans2, sampleLength, bufferProperties)));
            assertFalse(doubleSample.equals(new BufferSample<>(from, doubles2, sampleLength, bufferProperties)));
            assertFalse(intSample.equals(new BufferSample<>(from, ints2, sampleLength, bufferProperties)));
            assertFalse(longSample.equals(new BufferSample<>(from, longs2, sampleLength, bufferProperties)));
            assertFalse(byteSample.equals(new BufferSample<>(from, bytes2, sampleLength, bufferProperties)));
         }
      }
   }

   @Test
   public void testSampleEquals()
   {
      Random random = new Random(80925);

      assertTrue(BufferSample.sampleEquals(null, null));
      assertFalse(BufferSample.sampleEquals(new boolean[10], null));
      assertFalse(BufferSample.sampleEquals(null, new boolean[10]));
      assertThrows(IllegalArgumentException.class, () -> BufferSample.sampleEquals(new Object(), new boolean[10]));
      assertThrows(IllegalArgumentException.class, () -> BufferSample.sampleEquals(new boolean[10], new Object()));

      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(200);
         boolean[] booleans = new boolean[size];
         double[] doubles = new double[size];
         int[] ints = new int[size];
         long[] longs = new long[size];
         byte[] bytes = new byte[size];

         for (int j = 0; j < size; j++)
         {
            booleans[j] = random.nextBoolean();
            doubles[j] = random.nextDouble();
            ints[j] = random.nextInt();
            longs[j] = random.nextInt();
            bytes[j] = (byte) random.nextInt();
         }

         assertTrue(BufferSample.sampleEquals(booleans, booleans));
         assertTrue(BufferSample.sampleEquals(booleans, Arrays.copyOf(booleans, size)));
         assertTrue(BufferSample.sampleEquals(doubles, doubles));
         assertTrue(BufferSample.sampleEquals(doubles, Arrays.copyOf(doubles, size)));
         assertTrue(BufferSample.sampleEquals(ints, ints));
         assertTrue(BufferSample.sampleEquals(ints, Arrays.copyOf(ints, size)));
         assertTrue(BufferSample.sampleEquals(longs, longs));
         assertTrue(BufferSample.sampleEquals(longs, Arrays.copyOf(longs, size)));
         assertTrue(BufferSample.sampleEquals(bytes, bytes));
         assertTrue(BufferSample.sampleEquals(bytes, Arrays.copyOf(bytes, size)));

         if (size > 0)
         {
            assertFalse(BufferSample.sampleEquals(booleans, Arrays.copyOf(booleans, size - 1)));
            assertFalse(BufferSample.sampleEquals(doubles, Arrays.copyOf(doubles, size - 1)));
            assertFalse(BufferSample.sampleEquals(ints, Arrays.copyOf(ints, size - 1)));
            assertFalse(BufferSample.sampleEquals(longs, Arrays.copyOf(longs, size - 1)));
            assertFalse(BufferSample.sampleEquals(bytes, Arrays.copyOf(bytes, size - 1)));
         }
         assertFalse(BufferSample.sampleEquals(booleans, Arrays.copyOf(booleans, size + 1)));
         assertFalse(BufferSample.sampleEquals(doubles, Arrays.copyOf(doubles, size + 1)));
         assertFalse(BufferSample.sampleEquals(ints, Arrays.copyOf(ints, size + 1)));
         assertFalse(BufferSample.sampleEquals(longs, Arrays.copyOf(longs, size + 1)));
         assertFalse(BufferSample.sampleEquals(bytes, Arrays.copyOf(bytes, size + 1)));

         assertFalse(BufferSample.sampleEquals(booleans, doubles));

         if (size > 0)
         {
            int index = random.nextInt(size);

            boolean[] booleans2 = Arrays.copyOf(booleans, size);
            booleans2[index] = !booleans[index];
            double[] doubles2 = Arrays.copyOf(doubles, size);
            doubles2[index] += 1.0;
            int[] ints2 = Arrays.copyOf(ints, size);
            ints2[index] += 1;
            long[] longs2 = Arrays.copyOf(longs, size);
            longs2[index] += 1L;
            byte[] bytes2 = Arrays.copyOf(bytes, size);
            bytes2[index] += (byte) 1;

            assertFalse(BufferSample.sampleEquals(booleans, booleans2));
            assertFalse(BufferSample.sampleEquals(doubles, doubles2));
            assertFalse(BufferSample.sampleEquals(ints, ints2));
            assertFalse(BufferSample.sampleEquals(longs, longs2));
            assertFalse(BufferSample.sampleEquals(bytes, bytes2));
         }
      }
   }
}

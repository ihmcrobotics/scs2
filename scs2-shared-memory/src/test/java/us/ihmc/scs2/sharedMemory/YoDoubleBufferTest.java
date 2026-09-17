package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
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
         LinkedYoDouble linkedYoVariable = yoDoubleBuffer.newLinkedYoVariable(linkedDouble, null);
         assertTrue(linkedDouble == linkedYoVariable.getLinkedYoVariable());
         assertTrue(yoDoubleBuffer == linkedYoVariable.getBuffer());
      }
   }

   /**
    * A resize/crop shuffles the data, so the record of which indices hold real data has to be shuffled the same way.
    * Rebuilding it as empty instead makes the whole cropped range look unpopulated, and the next read re-fetches it
    * from the {@link HistoricalValueBitsSource} - at indices that now mean something different - overwriting the
    * correct data the crop had just carried over.
    */
   @Test
   public void testCropKeepsCarriedOverDataInsteadOfRefetchingIt()
   {
      YoDouble yoDouble = new YoDouble("var", new YoRegistry("Dummy"));
      YoBufferProperties properties = new YoBufferProperties(0, 8);
      YoDoubleBuffer yoDoubleBuffer = new YoDoubleBuffer(yoDouble, properties);

      List<Integer> indicesFetchedFromSource = new ArrayList<>();
      yoDoubleBuffer.setHistoricalValueBitsSource((variable, from, length, writer) ->
      {
         for (int i = 0; i < length; i++)
         {
            indicesFetchedFromSource.add(from + i);
            writer.write(from + i, Double.doubleToLongBits(-1.0));
         }
      });

      // Fill every index with real data the way playback would, so nothing is left for the source to supply.
      for (int i = 0; i < 8; i++)
      {
         properties.setCurrentIndexUnsafe(i);
         yoDouble.set(i);
         yoDoubleBuffer.writeBuffer();
      }

      yoDoubleBuffer.copy(0, 8, properties);
      assertTrue(indicesFetchedFromSource.isEmpty(), "Fully written buffer should never consult the source");

      // Crop down to indices 2..5, mirroring YoSharedBuffer.cropBuffer: resize first, new size published after.
      yoDoubleBuffer.resizeBuffer(2, 4);
      properties.setSize(4);

      BufferSample<double[]> croppedSample = yoDoubleBuffer.copy(0, 4, properties);

      assertTrue(indicesFetchedFromSource.isEmpty(), "Cropped-in data is real data and must not be re-fetched");
      assertArrayEquals(new double[] {2.0, 3.0, 4.0, 5.0}, croppedSample.getSample());
   }

   /**
    * The flip side of {@link #testCropKeepsCarriedOverDataInsteadOfRefetchingIt()}: growing the buffer adds indices
    * that carry over nothing, and those do still have to be backfilled from the source.
    */
   @Test
   public void testGrowBackfillsOnlyTheIndicesThatCarriedOverNothing()
   {
      YoDouble yoDouble = new YoDouble("var", new YoRegistry("Dummy"));
      YoBufferProperties properties = new YoBufferProperties(0, 4);
      YoDoubleBuffer yoDoubleBuffer = new YoDoubleBuffer(yoDouble, properties);

      List<Integer> indicesFetchedFromSource = new ArrayList<>();
      yoDoubleBuffer.setHistoricalValueBitsSource((variable, from, length, writer) ->
      {
         for (int i = 0; i < length; i++)
         {
            indicesFetchedFromSource.add(from + i);
            writer.write(from + i, Double.doubleToLongBits(-1.0));
         }
      });

      for (int i = 0; i < 4; i++)
      {
         properties.setCurrentIndexUnsafe(i);
         yoDouble.set(i);
         yoDoubleBuffer.writeBuffer();
      }

      yoDoubleBuffer.resizeBuffer(0, 6);
      properties.setSize(6);

      yoDoubleBuffer.copy(0, 6, properties);

      assertEquals(List.of(4, 5), indicesFetchedFromSource);
   }

   /**
    * A budgeted backfill must stop when the allowance runs out and report the range as incomplete, so the caller
    * comes back for the rest instead of the buffer manager's thread being held for the whole range at once.
    */
   @Test
   public void testBudgetedBackfillStopsShortAndResumesWhereItLeftOff()
   {
      YoDouble yoDouble = new YoDouble("var", new YoRegistry("Dummy"));
      YoBufferProperties properties = new YoBufferProperties(0, 2048);
      YoDoubleBuffer yoDoubleBuffer = new YoDoubleBuffer(yoDouble, properties);

      HistoricalBackfillBudget budget = new HistoricalBackfillBudget();
      budget.setBudgetNanoseconds(2_000_000L); // 2ms
      yoDoubleBuffer.setBackfillBudget(budget);

      List<Integer> indicesFetchedFromSource = new ArrayList<>();
      yoDoubleBuffer.setHistoricalValueBitsSource((variable, from, length, writer) ->
      {
         for (int i = 0; i < length; i++)
         {
            indicesFetchedFromSource.add(from + i);
            writer.write(from + i, Double.doubleToLongBits(from + i));
         }
         // Stand in for the real cost of a source call - decompressing a batch of a log file.
         sleepMilliseconds(1);
      });

      budget.startCycle();
      assertFalse(yoDoubleBuffer.ensurePopulatedWithinBudget(0, 2048), "Should have run out of budget well short of 2048");

      int fetchedInFirstCycle = indicesFetchedFromSource.size();
      assertTrue(fetchedInFirstCycle > 0, "Should still make progress within the budget");
      assertTrue(fetchedInFirstCycle < 2048, "Should not have completed the range");

      // Keep handing it cycles until it reports the range complete.
      int cycles = 0;
      while (!yoDoubleBuffer.ensurePopulatedWithinBudget(0, 2048))
      {
         budget.startCycle();
         assertTrue(++cycles < 1000, "Backfill should converge, not spin");
      }

      // Every index fetched exactly once across all cycles, in order: no redundant re-fetching, no gaps.
      assertEquals(2048, indicesFetchedFromSource.size());
      for (int i = 0; i < 2048; i++)
         assertEquals(i, indicesFetchedFromSource.get(i));

      // And an already-complete range costs nothing more.
      budget.startCycle();
      assertTrue(yoDoubleBuffer.ensurePopulatedWithinBudget(0, 2048));
      assertEquals(2048, indicesFetchedFromSource.size());
   }

   /** An unbudgeted buffer must still backfill a whole range in one go, the way {@link YoVariableBuffer#copy} needs. */
   @Test
   public void testUnbudgetedBackfillCompletesInOneCall()
   {
      YoDouble yoDouble = new YoDouble("var", new YoRegistry("Dummy"));
      YoBufferProperties properties = new YoBufferProperties(0, 2048);
      YoDoubleBuffer yoDoubleBuffer = new YoDoubleBuffer(yoDouble, properties);

      List<Integer> indicesFetchedFromSource = new ArrayList<>();
      yoDoubleBuffer.setHistoricalValueBitsSource((variable, from, length, writer) ->
      {
         for (int i = 0; i < length; i++)
         {
            indicesFetchedFromSource.add(from + i);
            writer.write(from + i, Double.doubleToLongBits(from + i));
         }
      });

      assertTrue(yoDoubleBuffer.ensurePopulatedWithinBudget(0, 2048), "No budget installed means no early exit");
      assertEquals(2048, indicesFetchedFromSource.size());
   }

   private static void sleepMilliseconds(long milliseconds)
   {
      try
      {
         Thread.sleep(milliseconds);
      }
      catch (InterruptedException e)
      {
         Thread.currentThread().interrupt();
         throw new RuntimeException(e);
      }
   }
}

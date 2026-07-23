package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.YoVariableChartData.ChartDataUpdate;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.YoVariableChartData.DoubleArray;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Differential test for {@link ChartDataUpdate#readUpdate}: a consumer that polls every tick (via an
 * independent brute-force reference copy, bypassing readUpdate's own logic entirely) must always match a
 * consumer that polls intermittently (via the real, possibly-incremental readUpdate), even when the
 * intermittent consumer's skips straddle a forced rebuild ("scrub") -- the scenario
 * {@code YoVariableChartData.structureGeneration} exists to guard against.
 */
public class ChartDataUpdateConsumerCatchUpTest
{
   @Test
   public void testIntermittentConsumerMatchesAlwaysPollingReference()
   {
      Random random = new Random(99L);
      for (int trial = 0; trial < 8; trial++)
      {
         int bufferSize = 8 + random.nextInt(30);
         runTrial(random, bufferSize, 400);
      }
   }

   /** Narrow regression case: buffer is full and steady, consumer skips exactly across a single scrub tick. */
   @Test
   public void testSkipAcrossSingleScrubStillMatches()
   {
      Random random = new Random(7L);
      int bufferSize = 20;
      Producer producer = new Producer(bufferSize);

      NumberSeries referenceSeries = new NumberSeries("reference");
      NumberSeries sutSeries = new NumberSeries("sut");
      long sutTotal = -1, sutGeneration = -1;

      // Fill the buffer completely first, consumer polling every tick.
      for (int tick = 0; tick < bufferSize + 5; tick++)
      {
         ChartDataUpdate update = producer.applyContiguousTick(random);
         bruteForceApply(update, referenceSeries);
         update.readUpdate(sutSeries, sutTotal, sutGeneration);
         sutTotal = update.getTotalSamplesPublished();
         sutGeneration = update.getStructureGeneration();
         assertPointsEqual(referenceSeries, sutSeries, "fill tick=" + tick);
      }

      // One scrub tick: SUT does NOT poll this one, reference still does.
      ChartDataUpdate scrubUpdate = producer.applyScrubTick(random);
      bruteForceApply(scrubUpdate, referenceSeries);

      // Next normal tick: SUT polls now, must correctly catch up across the skipped scrub.
      ChartDataUpdate update = producer.applyContiguousTick(random);
      bruteForceApply(update, referenceSeries);
      update.readUpdate(sutSeries, sutTotal, sutGeneration);

      assertPointsEqual(referenceSeries, sutSeries, "post-scrub catch-up");
   }

   private static void runTrial(Random random, int bufferSize, int tickCount)
   {
      Producer producer = new Producer(bufferSize);

      NumberSeries referenceSeries = new NumberSeries("reference");
      NumberSeries sutSeries = new NumberSeries("sut");
      long sutTotal = -1, sutGeneration = -1;

      for (int tick = 0; tick < tickCount; tick++)
      {
         boolean scrub = random.nextInt(25) == 0;
         ChartDataUpdate update = scrub ? producer.applyScrubTick(random) : producer.applyContiguousTick(random);

         // Reference: always polls, via an independent brute-force copy (not readUpdate's logic at all).
         bruteForceApply(update, referenceSeries);

         // SUT: polls about 2/3 of ticks, via the real (possibly incremental) readUpdate.
         if (random.nextInt(3) != 0)
         {
            update.readUpdate(sutSeries, sutTotal, sutGeneration);
            sutTotal = update.getTotalSamplesPublished();
            sutGeneration = update.getStructureGeneration();
            assertPointsEqual(referenceSeries, sutSeries, "bufferSize=" + bufferSize + " tick=" + tick);
         }
      }
   }

   /**
    * Mirrors {@code YoVariableChartData.publishForCharts()}'s exact bookkeeping (canApplyIncrementally /
    * incrementallyPatchValuesAndBounds / rebuildEntireDataSet, plus the two new counters), for a single
    * simulated variable.
    */
   private static final class Producer
   {
      private final int bufferSize;
      private double[] values;
      private MonotonicIndexDeque maxCandidates, minCandidates;
      private int appliedInPoint = -1, appliedOutPoint = -1;
      private long totalSamplesPublished = 0, structureGeneration = 0;
      private int outPoint = -1, activeLength = 0;

      Producer(int bufferSize)
      {
         this.bufferSize = bufferSize;
      }

      ChartDataUpdate applyContiguousTick(Random random)
      {
         int insertCount = 1 + random.nextInt(4);
         double[] newValues = new double[insertCount];
         for (int i = 0; i < insertCount; i++)
            newValues[i] = random.nextInt(3) == 0 ? 5.0 : random.nextDouble() * 20.0 - 10.0;

         int from = SharedMemoryTools.increment(outPoint, 1, bufferSize);
         int newOutPoint = SharedMemoryTools.computeToIndex(from, insertCount, bufferSize);
         int newActiveLength = Math.min(bufferSize, activeLength + insertCount);
         int newInPoint = SharedMemoryTools.computeFromIndex(newOutPoint, newActiveLength, bufferSize);

         BufferSample<double[]> sample = new BufferSample<>(from, newValues, insertCount,
                                                              new TestBufferProperties(bufferSize, newOutPoint, newInPoint, newOutPoint));
         ChartDataUpdate update = apply(sample, false);
         outPoint = newOutPoint;
         activeLength = newActiveLength;
         return update;
      }

      /** A discontinuous jump to an unrelated position -- simulates a same-size crop/scrub/resend. */
      ChartDataUpdate applyScrubTick(Random random)
      {
         int insertCount = 1 + random.nextInt(3);
         double[] newValues = new double[insertCount];
         for (int i = 0; i < insertCount; i++)
            newValues[i] = random.nextDouble() * 100.0;

         int from = random.nextInt(bufferSize);
         int newOutPoint = SharedMemoryTools.computeToIndex(from, insertCount, bufferSize);
         int newActiveLength = insertCount;
         int newInPoint = from;

         BufferSample<double[]> sample = new BufferSample<>(from, newValues, insertCount,
                                                              new TestBufferProperties(bufferSize, newOutPoint, newInPoint, newOutPoint));
         ChartDataUpdate update = apply(sample, true);
         outPoint = newOutPoint;
         activeLength = newActiveLength;
         return update;
      }

      private ChartDataUpdate apply(BufferSample<double[]> sample, boolean forceRebuild)
      {
         boolean canIncremental = !forceRebuild && values != null
                                   && YoVariableChartData.canApplyIncrementally(bufferSize, appliedInPoint, appliedOutPoint, true, sample);

         DoubleArray published;
         if (canIncremental)
         {
            double[] minMax = YoVariableChartData.incrementallyPatchValuesAndBounds(values, sample, appliedInPoint, maxCandidates, minCandidates);
            DoubleArray canonical = wrap(values, bufferSize);
            canonical.valueMin = minMax[0];
            canonical.valueMax = minMax[1];
            totalSamplesPublished += sample.getSampleLength();
            published = canonical.snapshot();
         }
         else
         {
            maxCandidates = new MonotonicIndexDeque(bufferSize, true);
            minCandidates = new MonotonicIndexDeque(bufferSize, false);
            DoubleArray rebuilt = YoVariableChartData.rebuildEntireDataSet(values == null ? null : wrap(values, bufferSize), sample, maxCandidates,
                                                                            minCandidates);
            values = rebuilt.values;
            totalSamplesPublished += sample.getSampleLength();
            structureGeneration++;
            published = rebuilt;
         }

         appliedInPoint = sample.getBufferProperties().getInPoint();
         appliedOutPoint = sample.getBufferProperties().getOutPoint();

         return new ChartDataUpdate(published, sample.getBufferProperties(), totalSamplesPublished, structureGeneration);
      }
   }

   /** Independent of readUpdate's own logic: a separately hand-written brute-force oracle. */
   private static void bruteForceApply(ChartDataUpdate update, NumberSeries series)
   {
      DoubleArray dataSet = update.dataSet;
      while (series.getData().size() < dataSet.size)
         series.getData().add(new Point2D());
      while (series.getData().size() > dataSet.size)
         series.getData().remove(series.getData().size() - 1);
      for (int i = 0; i < dataSet.size; i++)
         series.getData().get(i).set(i, dataSet.values[i]);
   }

   private static void assertPointsEqual(NumberSeries expected, NumberSeries actual, String context)
   {
      assertEquals(expected.getData().size(), actual.getData().size(), context);
      for (int i = 0; i < expected.getData().size(); i++)
      {
         assertEquals(expected.getData().get(i).getX(), actual.getData().get(i).getX(), 0.0, context + " index=" + i);
         assertEquals(expected.getData().get(i).getY(), actual.getData().get(i).getY(), 0.0, context + " index=" + i);
      }
   }

   private static DoubleArray wrap(double[] values, int size)
   {
      DoubleArray wrapped = new DoubleArray(size);
      System.arraycopy(values, 0, wrapped.values, 0, size);
      return wrapped;
   }
}

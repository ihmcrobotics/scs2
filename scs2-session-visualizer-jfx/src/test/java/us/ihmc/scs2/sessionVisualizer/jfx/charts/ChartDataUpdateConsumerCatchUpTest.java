package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.YoVariableChartData.ChartDataUpdate;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.YoVariableChartData.DoubleArray;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
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
   // readUpdate's own eligibility check (canPatchIncrementally, in YoVariableChartData.ChartDataUpdate)
   // is deliberately not unit-tested in isolation the way canApplyIncrementally is in
   // YoVariableChartDataIncrementalUpdateTest. Instead every test here drives it end-to-end against an
   // independent oracle (bruteForceApply), because the property that actually matters -- "an
   // intermittently-polling consumer never falls behind or renders stale/wrong points" -- is a
   // statement about sequences of calls, not about any single call.

   @Test
   public void testIntermittentConsumerMatchesAlwaysPollingReference()
   {
      // Broad randomized coverage: varying buffer sizes, an SUT that misses about 1/3 of ticks, and
      // occasional scrubs sprinkled in at random points relative to the SUT's skips.
      Random random = new Random(99L); // fixed seed: failures must reproduce deterministically
      for (int trial = 0; trial < 8; trial++)
      {
         int bufferSize = 8 + random.nextInt(30);
         runTrial(random, bufferSize, 400);
      }
   }

   /**
    * Narrow, deterministic regression case: buffer is full and steady, consumer skips exactly across a
    * single scrub tick. This is the sharpest version of the hazard {@code structureGeneration} exists
    * to guard against -- if that counter check were ever dropped or off-by-one, this is the test that
    * would catch a consumer patching only the samples it thinks arrived and leaving the rest of the
    * series stale after a rebuild it never saw.
    */
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
         sutGeneration = update.getRebuildCounter();
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
      // sutTotal/sutGeneration are exactly what a real caller is expected to persist between calls:
      // the counters handed back by the previous readUpdate(), fed into the next one.
      long sutTotal = -1, sutGeneration = -1;

      for (int tick = 0; tick < tickCount; tick++)
      {
         boolean scrub = random.nextInt(25) == 0;
         ChartDataUpdate update = scrub ? producer.applyScrubTick(random) : producer.applyContiguousTick(random);

         // Reference: always polls, via an independent brute-force copy (not readUpdate's logic at all).
         bruteForceApply(update, referenceSeries);

         // SUT: polls about 2/3 of ticks, via the real (possibly incremental) readUpdate. Only asserted
         // on ticks it actually polls -- skipped ticks are exactly where staleness would accumulate
         // unnoticed if readUpdate's catch-up logic were wrong.
         if (random.nextInt(3) != 0)
         {
            update.readUpdate(sutSeries, sutTotal, sutGeneration);
            sutTotal = update.getTotalSamplesPublished();
            sutGeneration = update.getRebuildCounter();
            assertPointsEqual(referenceSeries, sutSeries, "bufferSize=" + bufferSize + " tick=" + tick);
         }
      }
   }

   /**
    * Mirrors {@code YoVariableChartData.publishForCharts()}'s exact bookkeeping (applyIncrementalUpdate /
    * incrementallyPatchValuesAndBounds / rebuildEntireDataSet, plus the two new counters), for a single
    * simulated variable. Reimplemented here rather than called directly because the real method is an
    * instance method on {@code YoVariableChartData} wired into a live session/JavaFX pipeline; if its
    * bookkeeping (when {@code structureGeneration} increments, in particular) ever changes, this class
    * needs a matching update.
    */
   private static final class Producer
   {
      private final int bufferSize;
      private DoubleArray canonicalDataSet;
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

      /**
       * A discontinuous jump to an unrelated position -- simulates a same-size crop/scrub/resend.
       * Forces a full rebuild (structureGeneration++), which is exactly the case a consumer must
       * detect via the generation counter rather than assuming its own incremental patch still applies.
       */
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
         boolean canIncremental = !forceRebuild && canonicalDataSet != null
                                   && YoVariableChartData.applyIncrementalUpdate(bufferSize, appliedInPoint, appliedOutPoint, false, sample);

         DoubleArray published;
         if (canIncremental)
         {
            YoVariableChartData.incrementallyPatchValuesAndBounds(canonicalDataSet, sample, appliedInPoint, maxCandidates, minCandidates);
            totalSamplesPublished += sample.getSampleLength();
            // Publish a snapshot, not the canonical array itself -- mirrors YoVariableChartData.publishForCharts().
            published = canonicalDataSet.deepCopy();
         }
         else
         {
            maxCandidates = new MonotonicIndexDeque(bufferSize, true);
            minCandidates = new MonotonicIndexDeque(bufferSize, false);
            canonicalDataSet = YoVariableChartData.rebuildEntireDataSet(canonicalDataSet, sample, maxCandidates, minCandidates);
            totalSamplesPublished += sample.getSampleLength();
            structureGeneration++;
            published = canonicalDataSet;
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

   /** Minimal, JavaFX-free {@link YoBufferPropertiesReadOnly} stub for constructing test {@code BufferSample}s. */
   private static final class TestBufferProperties implements YoBufferPropertiesReadOnly
   {
      private final int size, currentIndex, inPoint, outPoint;

      TestBufferProperties(int size, int currentIndex, int inPoint, int outPoint)
      {
         this.size = size;
         this.currentIndex = currentIndex;
         this.inPoint = inPoint;
         this.outPoint = outPoint;
      }

      @Override
      public int getSize()
      {
         return size;
      }

      @Override
      public int getCurrentIndex()
      {
         return currentIndex;
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

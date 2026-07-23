package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Covers the two building blocks of {@link YoVariableChartData}'s incremental min/max update
 * ({@code canApplyIncrementally} and {@code incrementallyPatchValuesAndBounds}) plus the unchanged
 * full-rebuild fallback ({@code rebuildEntireDataSet}), then closes with a randomized differential
 * test that pits the incremental path against the full-rebuild path directly. The example-based tests
 * pin individual edge cases with a readable failure message; the differential test at the bottom is
 * what actually gives confidence the two paths always agree, since it can't be reasoned about by
 * inspection the way a handful of hand-picked cases can.
 */
public class YoVariableChartDataIncrementalUpdateTest
{
   private static BufferSample<double[]> sample(int from, double[] values, int size, int currentIndex, int inPoint, int outPoint)
   {
      return new BufferSample<>(from, values, values.length, new TestBufferProperties(size, currentIndex, inPoint, outPoint));
   }

   // ---------------------------------------------------------------------------------------------
   // canApplyIncrementally: the gate that decides whether a tick can skip the O(bufferSize) rebuild.
   // A false positive here is the dangerous direction -- it would make the incremental path apply a
   // patch that silently produces the wrong values/min/max with no exception to catch it, so these
   // cases lean toward covering every way the "plain forward append" assumption can be violated.
   // ---------------------------------------------------------------------------------------------

   @Test
   public void testAcceptsForwardAppendWhileBufferStillFilling()
   {
      // size=10, applied window [0,4] (not yet full), 2 new samples appended, in-point unchanged.
      BufferSample<double[]> newSample = sample(5, new double[] {1, 2}, 10, 6, 0, 6);
      assertTrue(YoVariableChartData.canApplyIncrementally(10, 0, 4, true, newSample));
   }

   @Test
   public void testAcceptsForwardAppendOnceBufferIsFullAndWrapping()
   {
      // size=10, applied window [5,4] (full, wrapped), 1 new sample evicts exactly 1 old one.
      BufferSample<double[]> newSample = sample(5, new double[] {9}, 10, 5, 6, 5);
      assertTrue(YoVariableChartData.canApplyIncrementally(10, 5, 4, true, newSample));
   }

   @Test
   public void testAcceptsMultiSampleAppendWhileFull()
   {
      // size=10, applied window [5,4] (full), 3 new samples evict exactly 3 old ones.
      BufferSample<double[]> newSample = sample(5, new double[] {9, 9, 9}, 10, 7, 8, 7);
      assertTrue(YoVariableChartData.canApplyIncrementally(10, 5, 4, true, newSample));
   }

   @Test
   public void testRejectsFirstCall()
   {
      // No canonical dataset/deques exist yet (canonicalDataSetPresent=false), so there is nothing to
      // patch incrementally regardless of what the sample looks like -- must always fall back to a
      // full rebuild the very first time a variable is charted.
      BufferSample<double[]> newSample = sample(0, new double[] {1}, 10, 0, 0, 0);
      assertFalse(YoVariableChartData.canApplyIncrementally(-1, -1, -1, false, newSample));
   }

   @Test
   public void testRejectsSizeChange()
   {
      // Ring buffer capacity itself changed (10 -> 20). The old deques/values array are sized for the
      // previous capacity and cannot be reused; only a full rebuild can re-prime them correctly.
      BufferSample<double[]> newSample = sample(5, new double[] {1}, 20, 5, 0, 5);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 0, 4, true, newSample));
   }

   @Test
   public void testRejectsGapBetweenAppliedOutPointAndNewSample()
   {
      // applied out-point is 4, but the new sample starts at 6 -- a gap, not a plain append.
      // Incrementally patching would leave index 5 never written, silently keeping a stale value.
      BufferSample<double[]> newSample = sample(6, new double[] {1}, 10, 6, 0, 6);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 0, 4, true, newSample));
   }

   @Test
   public void testRejectsOverlapBetweenAppliedOutPointAndNewSample()
   {
      // applied out-point is 4, but the new sample starts at 3 -- overlaps already-applied data.
      // Re-inserting index 4 as a "new" candidate would double-push it into the deques.
      BufferSample<double[]> newSample = sample(3, new double[] {1, 2}, 10, 4, 0, 4);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 0, 4, true, newSample));
   }

   @Test
   public void testRejectsBackwardCropJumpOfInPoint()
   {
      // applied in-point 5 moving "backward" to 3 -- looks like an implausibly large forward eviction.
      // This shape shows up on a scrub/crop to an earlier position, not a live forward tick, and must
      // not be treated as "a lot of ordinary eviction happened."
      BufferSample<double[]> newSample = sample(5, new double[] {1}, 10, 5, 3, 5);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 5, 4, true, newSample));
   }

   @Test
   public void testRejectsInPointJumpingFurtherThanSamplesArrived()
   {
      // Only 1 new sample arrived, but the in-point jumped forward by 3 -- more evicted than inserted.
      // evictIfFront is only correct if called once per index that actually left the window in order;
      // accepting this would desync the deques' notion of the active window from reality.
      BufferSample<double[]> newSample = sample(5, new double[] {1}, 10, 5, 8, 5);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 5, 4, true, newSample));
   }

   // ---------------------------------------------------------------------------------------------
   // rebuildEntireDataSet reproduces today's (unchanged) getValueAt/updateBounds behavior.
   // This is the fallback path canApplyIncrementally routes to whenever the fast path is rejected
   // above, so these pin its output against fixed, hand-computed expectations before it's reused as
   // the "reference" oracle in the differential test below.
   // ---------------------------------------------------------------------------------------------

   @Test
   public void testRebuildFirstEverLoad()
   {
      double[] newValues = {3, 1, 4, 1, 5};
      BufferSample<double[]> newSample = sample(0, newValues, 5, 2, 0, 4);

      YoVariableChartData.DoubleArray rebuilt = YoVariableChartData.rebuildEntireDataSet(null, newSample, new MonotonicIndexDeque(5, true),
                                                                                          new MonotonicIndexDeque(5, false));

      assertArrayEquals(newValues, rebuilt.values, 0.0);
      assertEquals(1.0, rebuilt.valueMin, 0.0);
      assertEquals(5.0, rebuilt.valueMax, 0.0);
   }

   @Test
   public void testRebuildNonWrappedActiveWindow()
   {
      // Previous full buffer, only indices [2,4] are "active"; new sample only covers index 3.
      YoVariableChartData.DoubleArray previous = new YoVariableChartData.DoubleArray(5);
      previous.values[2] = 10;
      previous.values[3] = -7;
      previous.values[4] = 2;

      BufferSample<double[]> newSample = sample(3, new double[] {99}, 5, 3, 2, 4);
      YoVariableChartData.DoubleArray rebuilt = YoVariableChartData.rebuildEntireDataSet(previous, newSample, new MonotonicIndexDeque(5, true),
                                                                                          new MonotonicIndexDeque(5, false));

      assertEquals(10, rebuilt.values[2], 0.0);
      assertEquals(99, rebuilt.values[3], 0.0); // overwritten by the new sample
      assertEquals(2, rebuilt.values[4], 0.0);
      assertEquals(2.0, rebuilt.valueMin, 0.0); // min over active window [2,4]: {10, 99, 2}
      assertEquals(99.0, rebuilt.valueMax, 0.0);
   }

   @Test
   public void testRebuildWrappedActiveWindow()
   {
      // Active window wraps: in-point=3, out-point=1, over a buffer of size 5 -> indices {3,4,0,1}.
      YoVariableChartData.DoubleArray previous = new YoVariableChartData.DoubleArray(5);
      previous.values[3] = 1;
      previous.values[4] = 2;
      previous.values[0] = 3;
      previous.values[1] = 4;

      BufferSample<double[]> newSample = sample(1, new double[] {40}, 5, 1, 3, 1);
      YoVariableChartData.DoubleArray rebuilt = YoVariableChartData.rebuildEntireDataSet(previous, newSample, new MonotonicIndexDeque(5, true),
                                                                                          new MonotonicIndexDeque(5, false));

      assertEquals(1.0, rebuilt.valueMin, 0.0); // active window values: {1, 2, 3, 40}
      assertEquals(40.0, rebuilt.valueMax, 0.0);
   }

   @Test
   public void testRebuildActiveWindowOfLengthOne()
   {
      BufferSample<double[]> newSample = sample(2, new double[] {7}, 5, 2, 2, 2);
      YoVariableChartData.DoubleArray rebuilt = YoVariableChartData.rebuildEntireDataSet(null, newSample, new MonotonicIndexDeque(5, true),
                                                                                          new MonotonicIndexDeque(5, false));

      assertEquals(7.0, rebuilt.valueMin, 0.0);
      assertEquals(7.0, rebuilt.valueMax, 0.0);
   }

   // ---------------------------------------------------------------------------------------------
   // Differential/fuzz test: incremental path vs. always-full-rebuild, over identical random input.
   // This is the real safety net for the optimization -- it doesn't just exercise canApplyIncrementally
   // in isolation, it runs both algorithms tick-by-tick over the same randomized sample stream and
   // asserts every value/min/max still matches, which is the only practical way to catch a subtle
   // wraparound or eviction-ordering bug in the monotonic deque bookkeeping.
   // ---------------------------------------------------------------------------------------------

   @Test
   public void testIncrementalPathMatchesAlwaysRebuildingOverRandomTickSequence()
   {
      Random random = new Random(42L); // fixed seed: failures must reproduce deterministically

      for (int trial = 0; trial < 5; trial++)
      {
         int bufferSize = 8 + random.nextInt(20); // small sizes stress wraparound quickly
         runDifferentialTrial(random, bufferSize, 300);
      }
   }

   private static void runDifferentialTrial(Random random, int bufferSize, int tickCount)
   {
      // "Incremental" state: canonical values array + deques, evolved via canApplyIncrementally/incrementallyPatchValuesAndBounds/rebuildEntireDataSet.
      // These variables mirror exactly what YoVariableChartData.publishForCharts() carries across ticks.
      double[] incrementalValues = null;
      MonotonicIndexDeque maxCandidates = null;
      MonotonicIndexDeque minCandidates = null;
      int appliedSize = -1, appliedInPoint = -1, appliedOutPoint = -1;
      double incrementalMin = Double.NaN, incrementalMax = Double.NaN;

      // "Reference" state: always full-rebuild via rebuildEntireDataSet, no shortcuts.
      YoVariableChartData.DoubleArray referenceDataSet = null;

      int outPoint = -1; // sentinel: "no data yet" -- increment(-1, 1, size) correctly yields 0 for the first tick.
      int activeLength = 0;

      for (int tick = 0; tick < tickCount; tick++)
      {
         int insertCount = 1 + random.nextInt(4);
         double[] newValues = new double[insertCount];
         for (int i = 0; i < insertCount; i++)
            newValues[i] = random.nextInt(3) == 0 ? 5.0 : random.nextDouble() * 20.0 - 10.0; // frequent duplicates

         int from = SharedMemoryTools.increment(outPoint, 1, bufferSize);
         int newOutPoint = SharedMemoryTools.computeToIndex(from, insertCount, bufferSize);
         int newActiveLength = Math.min(bufferSize, activeLength + insertCount);
         int newInPoint = SharedMemoryTools.computeFromIndex(newOutPoint, newActiveLength, bufferSize);
         int currentIndex = newOutPoint;

         BufferSample<double[]> newSample = sample(from, newValues, bufferSize, currentIndex, newInPoint, newOutPoint);

         // Reference: always full rebuild.
         referenceDataSet = YoVariableChartData.rebuildEntireDataSet(referenceDataSet, newSample, new MonotonicIndexDeque(bufferSize, true),
                                                                       new MonotonicIndexDeque(bufferSize, false));

         // Incremental: pick the path exactly like publishForCharts() does.
         if (YoVariableChartData.canApplyIncrementally(appliedSize, appliedInPoint, appliedOutPoint, incrementalValues != null, newSample))
         {
            double[] minMax = YoVariableChartData.incrementallyPatchValuesAndBounds(incrementalValues, newSample, appliedInPoint, maxCandidates,
                                                                                      minCandidates);
            incrementalMin = minMax[0];
            incrementalMax = minMax[1];
         }
         else
         {
            maxCandidates = new MonotonicIndexDeque(bufferSize, true);
            minCandidates = new MonotonicIndexDeque(bufferSize, false);
            YoVariableChartData.DoubleArray rebuilt = YoVariableChartData.rebuildEntireDataSet(
                  incrementalValues == null ? null : wrap(incrementalValues, bufferSize), newSample, maxCandidates, minCandidates);
            incrementalValues = rebuilt.values;
            incrementalMin = rebuilt.valueMin;
            incrementalMax = rebuilt.valueMax;
         }
         appliedSize = bufferSize;
         appliedInPoint = newInPoint;
         appliedOutPoint = newOutPoint;

         String context = "bufferSize=" + bufferSize + " tick=" + tick;
         assertArrayEquals(referenceDataSet.values, incrementalValues, 0.0, context);
         assertEquals(referenceDataSet.valueMin, incrementalMin, 0.0, context);
         assertEquals(referenceDataSet.valueMax, incrementalMax, 0.0, context);

         outPoint = newOutPoint;
         activeLength = newActiveLength;
      }
   }

   private static YoVariableChartData.DoubleArray wrap(double[] values, int size)
   {
      YoVariableChartData.DoubleArray wrapped = new YoVariableChartData.DoubleArray(size);
      System.arraycopy(values, 0, wrapped.values, 0, size);
      return wrapped;
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

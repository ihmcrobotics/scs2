package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class YoVariableChartDataIncrementalUpdateTest
{
   private static BufferSample<double[]> sample(int from, double[] values, int size, int currentIndex, int inPoint, int outPoint)
   {
      return new BufferSample<>(from, values, values.length, new TestBufferProperties(size, currentIndex, inPoint, outPoint));
   }

   // ---------------------------------------------------------------------------------------------
   // canApplyIncrementally
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
      BufferSample<double[]> newSample = sample(0, new double[] {1}, 10, 0, 0, 0);
      assertFalse(YoVariableChartData.canApplyIncrementally(-1, -1, -1, false, newSample));
   }

   @Test
   public void testRejectsSizeChange()
   {
      BufferSample<double[]> newSample = sample(5, new double[] {1}, 20, 5, 0, 5);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 0, 4, true, newSample));
   }

   @Test
   public void testRejectsGapBetweenAppliedOutPointAndNewSample()
   {
      // applied out-point is 4, but the new sample starts at 6 -- a gap, not a plain append.
      BufferSample<double[]> newSample = sample(6, new double[] {1}, 10, 6, 0, 6);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 0, 4, true, newSample));
   }

   @Test
   public void testRejectsOverlapBetweenAppliedOutPointAndNewSample()
   {
      // applied out-point is 4, but the new sample starts at 3 -- overlaps already-applied data.
      BufferSample<double[]> newSample = sample(3, new double[] {1, 2}, 10, 4, 0, 4);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 0, 4, true, newSample));
   }

   @Test
   public void testRejectsBackwardCropJumpOfInPoint()
   {
      // applied in-point 5 moving "backward" to 3 -- looks like an implausibly large forward eviction.
      BufferSample<double[]> newSample = sample(5, new double[] {1}, 10, 5, 3, 5);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 5, 4, true, newSample));
   }

   @Test
   public void testRejectsInPointJumpingFurtherThanSamplesArrived()
   {
      // Only 1 new sample arrived, but the in-point jumped forward by 3 -- more evicted than inserted.
      BufferSample<double[]> newSample = sample(5, new double[] {1}, 10, 5, 8, 5);
      assertFalse(YoVariableChartData.canApplyIncrementally(10, 5, 4, true, newSample));
   }

   // ---------------------------------------------------------------------------------------------
   // rebuildEntireDataSet reproduces today's (unchanged) getValueAt/updateBounds behavior
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
   // ---------------------------------------------------------------------------------------------

   @Test
   public void testIncrementalPathMatchesAlwaysRebuildingOverRandomTickSequence()
   {
      Random random = new Random(42L);

      for (int trial = 0; trial < 5; trial++)
      {
         int bufferSize = 8 + random.nextInt(20); // small sizes stress wraparound quickly
         runDifferentialTrial(random, bufferSize, 300);
      }
   }

   private static void runDifferentialTrial(Random random, int bufferSize, int tickCount)
   {
      // "Incremental" state: canonical values array + deques, evolved via canApplyIncrementally/incrementallyPatchValuesAndBounds/rebuildEntireDataSet.
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
}

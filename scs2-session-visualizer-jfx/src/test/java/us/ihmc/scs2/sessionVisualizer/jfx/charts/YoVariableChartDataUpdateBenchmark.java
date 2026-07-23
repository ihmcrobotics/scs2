package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Manual timing comparison, not a regression test: how much compute time {@code ChartDataManager}'s
 * 100ms tick actually costs across a realistic number of charted variables, before vs. after the
 * incremental min/max change in {@link YoVariableChartData}. Unlike the JavaFX-rendering-side
 * benchmarks, this needs no JavaFX toolkit at all -- {@code YoVariableChartData}'s update methods are
 * plain Java, so the "before" behavior (always take the full O(bufferSize) rebuild path, exactly what
 * every tick did prior to this change) and the "after" behavior (use the incremental path whenever
 * {@link YoVariableChartData#canApplyIncrementally} allows it) can both be measured directly and
 * honestly, with no rendering/threading noise to work around.
 */
public class YoVariableChartDataUpdateBenchmark
{
   private static final int BUFFER_SIZE = 10_000;
   private static final int NUMBER_OF_VARIABLES = 20;
   private static final int MIN_SAMPLES_PER_TICK = 1;
   private static final int MAX_SAMPLES_PER_TICK = 5;
   private static final int WARMUP_TICKS = 200;
   private static final int TIMED_TICKS = 500;

   @Test
   public void benchmarkUpdateCompute()
   {
      // Same tick workload (buffer positions + sample values) reused for every run, so "before" and
      // "after" are timed against identical input -- only the algorithm differs.
      List<List<BufferSample<double[]>>> perVariableTicks = new ArrayList<>(NUMBER_OF_VARIABLES);
      Random generatorRandom = new Random(2026_07_22L);
      for (int v = 0; v < NUMBER_OF_VARIABLES; v++)
         perVariableTicks.add(generateTickSequence(generatorRandom, BUFFER_SIZE, WARMUP_TICKS + TIMED_TICKS));

      // Warm up the JIT on both code paths before timing anything.
      for (List<BufferSample<double[]>> ticks : perVariableTicks)
      {
         runAlwaysFullRebuild(ticks.subList(0, WARMUP_TICKS), BUFFER_SIZE);
         runIncrementalWherePossible(ticks.subList(0, WARMUP_TICKS), BUFFER_SIZE);
      }

      long beforeTotalNanos = 0;
      for (List<BufferSample<double[]>> ticks : perVariableTicks)
         beforeTotalNanos += runAlwaysFullRebuild(ticks.subList(WARMUP_TICKS, WARMUP_TICKS + TIMED_TICKS), BUFFER_SIZE);

      long afterTotalNanos = 0;
      for (List<BufferSample<double[]>> ticks : perVariableTicks)
         afterTotalNanos += runIncrementalWherePossible(ticks.subList(WARMUP_TICKS, WARMUP_TICKS + TIMED_TICKS), BUFFER_SIZE);

      long beforeUs = beforeTotalNanos / 1_000;
      long afterUs = afterTotalNanos / 1_000;
      double beforeUsPerChartTick = beforeUs / (double) TIMED_TICKS;
      double afterUsPerChartTick = afterUs / (double) TIMED_TICKS;

      System.out.println("[benchmark] Parameters: bufferSize=" + BUFFER_SIZE + ", variables=" + NUMBER_OF_VARIABLES + ", ticks=" + TIMED_TICKS
                          + ", samples/tick=[" + MIN_SAMPLES_PER_TICK + "," + MAX_SAMPLES_PER_TICK + "]");
      System.out.println("[benchmark] BEFORE (always full O(bufferSize) rebuild): " + beforeUs + " us total, " + String.format("%.1f", beforeUsPerChartTick)
                          + " us per ChartDataManager tick (all " + NUMBER_OF_VARIABLES + " variables) -- budget is 100000 us");
      System.out.println("[benchmark] AFTER  (incremental where eligible): " + afterUs + " us total, " + String.format("%.1f", afterUsPerChartTick)
                          + " us per ChartDataManager tick (all " + NUMBER_OF_VARIABLES + " variables) -- budget is 100000 us");
      System.out.println("[benchmark] Speedup: " + String.format("%.1fx", beforeUs / (double) Math.max(1, afterUs)));
   }

   /** Generates a realistic forward-advancing ring-buffer tick sequence, matching steady live playback. */
   private static List<BufferSample<double[]>> generateTickSequence(Random random, int bufferSize, int tickCount)
   {
      List<BufferSample<double[]>> ticks = new ArrayList<>(tickCount);
      int outPoint = -1; // sentinel: increment(-1, 1, size) correctly yields 0 for the first tick.
      int activeLength = 0;

      for (int tick = 0; tick < tickCount; tick++)
      {
         int insertCount = MIN_SAMPLES_PER_TICK + random.nextInt(MAX_SAMPLES_PER_TICK - MIN_SAMPLES_PER_TICK + 1);
         double[] newValues = new double[insertCount];
         for (int i = 0; i < insertCount; i++)
            newValues[i] = random.nextDouble() * 20.0 - 10.0;

         int from = SharedMemoryTools.increment(outPoint, 1, bufferSize);
         int newOutPoint = SharedMemoryTools.computeToIndex(from, insertCount, bufferSize);
         int newActiveLength = Math.min(bufferSize, activeLength + insertCount);
         int newInPoint = SharedMemoryTools.computeFromIndex(newOutPoint, newActiveLength, bufferSize);

         ticks.add(new BufferSample<>(from, newValues, insertCount, new TestBufferProperties(bufferSize, newOutPoint, newInPoint, newOutPoint)));

         outPoint = newOutPoint;
         activeLength = newActiveLength;
      }
      return ticks;
   }

   /** Mirrors what every tick did prior to this change: always take the full O(bufferSize) rebuild path. */
   private static long runAlwaysFullRebuild(List<BufferSample<double[]>> ticks, int bufferSize)
   {
      YoVariableChartData.DoubleArray dataSet = null;

      long start = System.nanoTime();
      for (BufferSample<double[]> tick : ticks)
      {
         MonotonicIndexDeque maxCandidates = new MonotonicIndexDeque(bufferSize, true);
         MonotonicIndexDeque minCandidates = new MonotonicIndexDeque(bufferSize, false);
         dataSet = YoVariableChartData.rebuildEntireDataSet(dataSet, tick, maxCandidates, minCandidates);
      }
      return System.nanoTime() - start;
   }

   /** Mirrors current {@code publishForCharts()} behavior: incremental patch whenever eligible. */
   private static long runIncrementalWherePossible(List<BufferSample<double[]>> ticks, int bufferSize)
   {
      double[] values = null;
      MonotonicIndexDeque maxCandidates = null;
      MonotonicIndexDeque minCandidates = null;
      int appliedSize = -1, appliedInPoint = -1, appliedOutPoint = -1;

      long start = System.nanoTime();
      for (BufferSample<double[]> tick : ticks)
      {
         if (YoVariableChartData.canApplyIncrementally(appliedSize, appliedInPoint, appliedOutPoint, values != null, tick))
         {
            YoVariableChartData.incrementallyPatchValuesAndBounds(values, tick, appliedInPoint, maxCandidates, minCandidates);
         }
         else
         {
            maxCandidates = new MonotonicIndexDeque(bufferSize, true);
            minCandidates = new MonotonicIndexDeque(bufferSize, false);
            YoVariableChartData.DoubleArray rebuilt = YoVariableChartData.rebuildEntireDataSet(values == null ? null : wrap(values, bufferSize), tick,
                                                                                                 maxCandidates, minCandidates);
            values = rebuilt.values;
         }
         appliedSize = bufferSize;
         appliedInPoint = tick.getBufferProperties().getInPoint();
         appliedOutPoint = tick.getBufferProperties().getOutPoint();
      }
      return System.nanoTime() - start;
   }

   private static YoVariableChartData.DoubleArray wrap(double[] values, int size)
   {
      YoVariableChartData.DoubleArray wrapped = new YoVariableChartData.DoubleArray(size);
      System.arraycopy(values, 0, wrapped.values, 0, size);
      return wrapped;
   }
}

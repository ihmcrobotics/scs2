package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionProperties;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.sharedMemory.FillBufferRequest;
import us.ihmc.scs2.sharedMemory.LinkedYoBoolean;
import us.ihmc.scs2.sharedMemory.LinkedYoDouble;
import us.ihmc.scs2.sharedMemory.LinkedYoEnum;
import us.ihmc.scs2.sharedMemory.LinkedYoInteger;
import us.ihmc.scs2.sharedMemory.LinkedYoLong;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Function;

public class YoVariableChartData
{
   private final LinkedYoVariable<?> linkedYoVariable;

   private SessionMode lastSessionModeStatus = null;
   private final AtomicReference<SessionMode> currentSessionMode;
   private YoBufferPropertiesReadOnly lastProperties = null;

   @SuppressWarnings("rawtypes")
   private final AtomicReference<BufferSample> rawDataProperty = new AtomicReference<>(null);
   private final AtomicBoolean hasChartData = new AtomicBoolean(false);

   private int lastUpdateEndIndex = -1;
   private DoubleArray lastDataSet;
   private ChartDataUpdate lastChartDataUpdate;

   private MonotonicIndexDeque maxCandidates, minCandidates;
   // Bookkeeping for the max and min candidates
   private int previousSize = -1, previousInPoint = -1, previousOutPoint = -1;

   /** Monotonic count of new samples ever applied to {@link #lastDataSet}, across this instance's whole lifetime. */
   private long totalSamplesPublished = 0;
   /**
    * Bumped on every full rebuild, not on incremental rebuilds. If this differs from what a caller
    * last observed, ring positions outside the naive "trailing N samples" window may have changed too,
    * so a full repaint is required rather than a partial patch.
    */
   private long fullBufferRebuildCounter = 0;

   private final Queue<Object> callerIDs = new ConcurrentLinkedQueue<>();
   private final Map<Object, ChartDataUpdate> newChartDataUpdate = new ConcurrentHashMap<>();

   private final AtomicBoolean requestEntireBuffer = new AtomicBoolean(true);
   private final AtomicBoolean requestUpdateBounds = new AtomicBoolean(false);

   @SuppressWarnings("rawtypes")
   private final Function<BufferSample<double[]>, BufferSample> bufferConverterFunction;

   private final Consumer<CropBufferRequest> cropRequestListener = m -> requestEntireBuffer.set(true);
   private final Consumer<FillBufferRequest> fillRequestListener = m -> requestEntireBuffer.set(true);
   private final Runnable forceListenerUpdateListener = () -> requestEntireBuffer.set(true);
   private final Consumer<YoBufferPropertiesReadOnly> propertiesListener;
   private final Consumer<SessionProperties> sessionPropertiesListener;

   private final Session session;

   public YoVariableChartData(Session session, LinkedYoVariable<?> linkedYoVariable)
   {
      this.session = session;
      this.linkedYoVariable = linkedYoVariable;
      linkedYoVariable.addUser(this);
      currentSessionMode = new AtomicReference<>(SessionMode.PAUSE);
      sessionPropertiesListener = properties -> currentSessionMode.set(properties.getActiveMode());
      session.addSessionPropertiesListener(sessionPropertiesListener);
      currentSessionMode.set(session.getSessionProperties().getActiveMode());

      if (linkedYoVariable instanceof LinkedYoBoolean)
         bufferConverterFunction = in -> booleanToDoubleBuffer(in);
      else if (linkedYoVariable instanceof LinkedYoDouble)
         bufferConverterFunction = in -> in;
      else if (linkedYoVariable instanceof LinkedYoEnum<?>)
         bufferConverterFunction = in -> byteToDoubleBuffer(in);
      else if (linkedYoVariable instanceof LinkedYoInteger)
         bufferConverterFunction = in -> integerToDoubleBuffer(in);
      else if (linkedYoVariable instanceof LinkedYoLong)
         bufferConverterFunction = in -> longToDoubleBuffer(in);
      else
         throw new UnsupportedOperationException("Unsupported YoVariable type: " + linkedYoVariable.getLinkedYoVariable().getClass().getSimpleName());

      propertiesListener = m ->
      {
         if (lastProperties == null)
         { // First time requesting data.
            requestEntireBuffer.set(true);
         }
         else if (lastProperties.getSize() != m.getSize())
         { // Buffer was either resized or cropped, data has been shifted around, need to get a complete update.
            requestEntireBuffer.set(true);
         }
         else if (m.getInPoint() != lastProperties.getInPoint())
         {
            if (m.getOutPoint() != lastProperties.getOutPoint())
            { // When cropping without actually changing the size of the buffer, the data is still being shifted around.
               linkedYoVariable.requestEntireBuffer();
            }
            else
            {
               requestUpdateBounds.set(lastDataSet != null);
            }
         }
         else if (m.getOutPoint() != lastProperties.getOutPoint())
         {
            requestUpdateBounds.set(lastDataSet != null);
         }
         lastProperties = m;
      };

      session.addCropBufferRequestListener(cropRequestListener);
      session.addFillBufferRequestListener(fillRequestListener);
      session.addBufferListenerForceUpdateListener(forceListenerUpdateListener);
      session.addCurrentBufferPropertiesListener(propertiesListener);
   }

   public void dispose()
   {
      linkedYoVariable.removeUser(this);
      session.removeSessionPropertiesListener(sessionPropertiesListener);
      session.removeCropBufferRequestListener(cropRequestListener);
      session.removeFillBufferRequestListener(fillRequestListener);
      session.removeBufferListenerForceUpdateListener(forceListenerUpdateListener);
      session.removeCurrentBufferPropertiesListener(propertiesListener);
   }

   public boolean updateVariableData()
   {
      return linkedYoVariable.pull();
   }

   @SuppressWarnings("rawtypes")
   public void updateBufferData()
   {
      // Always prepare new data
      BufferSample newRawData = linkedYoVariable.pollRequestedBufferSample();
      if (newRawData != null)
      {
         rawDataProperty.set(newRawData);
         hasChartData.set(true);
         lastUpdateEndIndex = newRawData.getBufferProperties().getOutPoint();
      }

      // Now check if a new request should be submitted.
      if (lastSessionModeStatus == SessionMode.RUNNING && currentSessionMode.get() != SessionMode.RUNNING)
      { // The session just stopped running, need to ensure we have all the data up to the out-point.
         linkedYoVariable.requestBufferStartingFrom(lastUpdateEndIndex);
      }
      else if (callerIDs.stream().anyMatch(callerID -> !hasNewChartData(callerID)))
      {// Only request data if JFX is keeping up with the rendering.
         if (requestEntireBuffer.getAndSet(false))
         {
            linkedYoVariable.requestEntireBuffer();
         }
         else if (lastSessionModeStatus != SessionMode.RUNNING && currentSessionMode.get() == SessionMode.RUNNING)
         { // The session just start running, need to ensure we have all the data since it started running.
            linkedYoVariable.requestActiveBufferOnly();
         }
         else if (currentSessionMode.get() == SessionMode.RUNNING)
         { // Request data from the last update point to the most recent out-point.
            if (lastUpdateEndIndex == -1)
               linkedYoVariable.requestActiveBufferOnly();
            else
               linkedYoVariable.requestBufferStartingFrom(lastUpdateEndIndex);
         }
      }

      lastSessionModeStatus = currentSessionMode.get();

      publishForCharts();
   }

   private void publishForCharts()
   {
      boolean updateBounds = requestUpdateBounds.getAndSet(false);
      if (!hasChartData.get() && !updateBounds)
         return;

      @SuppressWarnings("rawtypes") BufferSample rawData = rawDataProperty.get();
      if (rawData == null || rawData.getSampleLength() == 0)
         return;

      @SuppressWarnings("unchecked") BufferSample<double[]> newBufferSample = bufferConverterFunction.apply(rawData);
      if (lastDataSet != null && newBufferSample.getBufferProperties().getSize() != lastDataSet.size)
      {
         lastDataSet = null;
         previousSize = -1; // Force a full rebuild (and fresh deques) on the next hasChartData tick.
      }

      DoubleArray publishedDataSet;

      if (hasChartData.get())
      {
         YoBufferPropertiesReadOnly newProperties = newBufferSample.getBufferProperties();
         boolean incrementalUpdate = applyIncrementalUpdate(previousSize, previousInPoint, previousOutPoint, lastDataSet == null, newBufferSample);

         if (incrementalUpdate)
         {
            incrementallyPatchValuesAndBounds(lastDataSet, newBufferSample, previousInPoint, maxCandidates, minCandidates);
            previousInPoint = newProperties.getInPoint();
            previousOutPoint = newProperties.getOutPoint();
            totalSamplesPublished += newBufferSample.getSampleLength();

            // Deep copy to the original data can change without this published data changing
            publishedDataSet = lastDataSet.deepCopy();
         }
         else
         {
            MonotonicIndexDeque newMaxCandidates = new MonotonicIndexDeque(newProperties.getSize(), true);
            MonotonicIndexDeque newMinCandidates = new MonotonicIndexDeque(newProperties.getSize(), false);
            DoubleArray rebuilt = rebuildEntireDataSet(lastDataSet, newBufferSample, newMaxCandidates, newMinCandidates);

            if (rebuilt != null)
            {
               lastDataSet = rebuilt;
               maxCandidates = newMaxCandidates;
               minCandidates = newMinCandidates;
               previousSize = newProperties.getSize();
               previousInPoint = newProperties.getInPoint();
               previousOutPoint = newProperties.getOutPoint();
               totalSamplesPublished += newBufferSample.getSampleLength();
               fullBufferRebuildCounter++;
            }
            // Safe to publish directly: freshly built, never shared with any caller yet.
            publishedDataSet = rebuilt;
         }
      }
      else if (updateBounds)
      {
         publishedDataSet = updateBounds(lastProperties, lastDataSet);
         if (publishedDataSet != null)
            lastDataSet = publishedDataSet;
      }
      else
         throw new IllegalStateException("Should not get here.");

      ChartDataUpdate chartDataUpdate = new ChartDataUpdate(publishedDataSet, rawData.getBufferProperties(), totalSamplesPublished, fullBufferRebuildCounter);
      lastChartDataUpdate = chartDataUpdate;

      if (publishedDataSet != null)
         callerIDs.forEach(callerID -> newChartDataUpdate.put(callerID, chartDataUpdate));

      hasChartData.set(false);
   }

   public void registerCaller(Object callerID)
   {
      callerIDs.add(callerID);
      if (lastDataSet != null)
         newChartDataUpdate.put(callerID, lastChartDataUpdate);
   }

   public void removeCaller(Object callerID)
   {
      callerIDs.remove(callerID);
      newChartDataUpdate.remove(callerID);
   }

   public boolean hasNewChartData(Object callerID)
   {
      return newChartDataUpdate.get(callerID) != null;
   }

   public ChartDataUpdate pollChartData(Object callerID)
   {
      if (newChartDataUpdate.isEmpty())
         return null;
      else
         return newChartDataUpdate.remove(callerID);
   }

   public YoVariable getYoVariable()
   {
      return linkedYoVariable.getLinkedYoVariable();
   }

   public boolean isCurrentlyInUse()
   {
      return !callerIDs.isEmpty();
   }

   /**
    * This checks whether the full update is required or not for the ChartData
    * @return boolean for whether we can do the incremental update, or we have to do the full update
    */
   static boolean applyIncrementalUpdate(int previousSize, int appliedInPoint, int appliedOutPoint, boolean dataIsNull,
                                         BufferSample<double[]> newBufferSample)
   {
      if (dataIsNull || previousSize < 0)
         return false;

      YoBufferPropertiesReadOnly newProperties = newBufferSample.getBufferProperties();
      int newSize = newProperties.getSize();
      if (previousSize != newSize)
         return false;

      int expectedFrom = SharedMemoryTools.increment(appliedOutPoint, 1, newSize);
      if (newBufferSample.getFrom() != expectedFrom)
         return false;

      if (newBufferSample.getTo() != newProperties.getOutPoint())
         return false;

      int evictionSteps = SharedMemoryTools.computeSubLength(appliedInPoint, newProperties.getInPoint(), newSize) - 1;
      return evictionSteps <= newBufferSample.getSampleLength();
   }

   /**
    * Updates the values and the two sliding-window deques in place for exactly what changed this tick.
    * Indices that aged out of the active window are removed, then the newly-arrived sample range is adde.
    * Caller must have already verified eligibility via {@link #applyIncrementalUpdate}. Patches {@code previousData.valueMin}/{@code valueMax} in place.
    */
   static void incrementallyPatchValuesAndBounds(DoubleArray previousData, BufferSample<double[]> newBufferSample, int previousInPoint,
                                                  MonotonicIndexDeque maxCandidates, MonotonicIndexDeque minCandidates)
   {
      double[] previousValues = previousData.values;
      YoBufferPropertiesReadOnly bufferProperties = newBufferSample.getBufferProperties();
      int bufferSize = bufferProperties.getSize();
      int newInPoint = bufferProperties.getInPoint();

      int evictionSteps = SharedMemoryTools.computeSubLength(previousInPoint, newInPoint, bufferSize) - 1;
      int index = previousInPoint;
      for (int i = 0; i < evictionSteps; i++)
      {
         maxCandidates.evictIfFront(index);
         minCandidates.evictIfFront(index);
         index = SharedMemoryTools.increment(index, 1, bufferSize);
      }

      double[] sample = newBufferSample.getSample();
      int insertIndex = newBufferSample.getFrom();

      for (int i = 0; i < newBufferSample.getSampleLength(); i++)
      {
         double value = sample[i];
         if (!Double.isFinite(value))
            value = 0.0;

         previousValues[insertIndex] = value;
         maxCandidates.pushCandidate(insertIndex, value, previousValues);
         minCandidates.pushCandidate(insertIndex, value, previousValues);
         insertIndex = SharedMemoryTools.increment(insertIndex, 1, bufferSize);
      }

      previousData.valueMin = previousValues[minCandidates.peekFrontIndex()];
      previousData.valueMax = previousValues[maxCandidates.peekFrontIndex()];
   }

   /**
    * Full O(bufferSize) rebuild of everything as we couldn't do an incremental update.
    */
   static DoubleArray rebuildEntireDataSet(DoubleArray previousDataSet, BufferSample<double[]> bufferSample, MonotonicIndexDeque maxCandidates,
                                            MonotonicIndexDeque minCandidates)
   {
      if (bufferSample == null || bufferSample.getSampleLength() == 0)
         return null;

      YoBufferPropertiesReadOnly bufferProperties = bufferSample.getBufferProperties();
      int bufferSize = bufferProperties.getSize();

      DoubleArray dataSet = new DoubleArray(bufferSize);

      for (int i = 0; i < bufferSize; i++)
         dataSet.values[i] = getValueAt(i, previousDataSet, bufferSample);

      maxCandidates.clear();
      minCandidates.clear();

      int index = bufferProperties.getInPoint();
      maxCandidates.pushCandidate(index, dataSet.values[index], dataSet.values);
      minCandidates.pushCandidate(index, dataSet.values[index], dataSet.values);

      for (int i = 1; i < bufferProperties.getActiveBufferLength(); i++)
      {
         index = SharedMemoryTools.increment(index, 1, bufferSize);
         maxCandidates.pushCandidate(index, dataSet.values[index], dataSet.values);
         minCandidates.pushCandidate(index, dataSet.values[index], dataSet.values);
      }

      dataSet.valueMin = dataSet.values[minCandidates.peekFrontIndex()];
      dataSet.valueMax = dataSet.values[maxCandidates.peekFrontIndex()];
      return dataSet;
   }

   private static DoubleArray updateBounds(YoBufferPropertiesReadOnly bufferProperties, DoubleArray dataSet)
   {
      if (bufferProperties.getSize() != dataSet.size)
         return null;

      int index = bufferProperties.getInPoint();
      double yCurrent = dataSet.values[index];
      double yMin = yCurrent;
      double yMax = yCurrent;

      for (int i = 1; i < bufferProperties.getActiveBufferLength(); i++)
      {
         index = SharedMemoryTools.increment(index, 1, bufferProperties.getSize());
         yCurrent = dataSet.values[index];
         yMin = Math.min(yMin, yCurrent);
         yMax = Math.max(yMax, yCurrent);
      }

      dataSet.valueMin = yMin;
      dataSet.valueMax = yMax;
      return dataSet;
   }

   private static double getValueAt(int index, DoubleArray completeDataSet, BufferSample<double[]> partialBufferSample)
   {
      double[] sample = partialBufferSample.getSample();
      int sampleStart = partialBufferSample.getFrom();
      int sampleEnd = partialBufferSample.getTo();
      int bufferSize = partialBufferSample.getBufferProperties().getSize();

      double y = 0.0;

      if (sampleStart <= sampleEnd)
      {
         if (index >= sampleStart && index <= sampleEnd)
            y = sample[index - sampleStart];
         else if (completeDataSet != null)
            y = completeDataSet.values[index];
      }
      else
      {
         if (index <= sampleEnd)
            y = sample[index - sampleStart + bufferSize];
         else if (index >= sampleStart)
            y = sample[index - sampleStart];
         else if (completeDataSet != null)
            y = completeDataSet.values[index];
      }

      // TODO Need to check if chart-fx handles NaN.
      if (!Double.isFinite(y))
         y = 0.0;

      return y;
   }

   public static class ChartDataUpdate
   {
      /** Package-private (rather than private) so it's directly testable from same-package unit tests. */
      final DoubleArray dataSet;
      private final YoBufferPropertiesReadOnly bufferProperties;
      private final long totalSamplesPublished;
      private final long rebuildCounter;

      public ChartDataUpdate(DoubleArray dataSet, YoBufferPropertiesReadOnly bufferProperties, long totalSamplesPublished, long rebuildCounter)
      {
         this.dataSet = dataSet;
         this.bufferProperties = bufferProperties;
         this.totalSamplesPublished = totalSamplesPublished;
         this.rebuildCounter = rebuildCounter;
      }

      /**
       * Copies {@link #dataSet} into {@code chartDataSet}'s {@code Point2D} list, patching only the ring
       * positions that changed since the caller's last call (tracked via {@code lastConsumedTotalSamples}/
       * {@code lastConsumedRebuildCounter}, which the caller should persist from
       * {@link #getTotalSamplesPublished()}/{@link #getRebuildCounter()} after each call) whenever that's
       * provably safe, falling back to a full rewrite otherwise.
       */
      public void readUpdate(NumberSeries chartDataSet, long lastConsumedTotalSamples, long lastConsumedRebuildCounter)
      {
         chartDataSet.getLock().writeLock().lock();

         try
         {
            boolean sizeMatches = chartDataSet.getData().size() == dataSet.size;

            // Resizing the cart data
            while (chartDataSet.getData().size() < dataSet.size)
               chartDataSet.getData().add(new Point2D());
            while (chartDataSet.getData().size() > dataSet.size)
               chartDataSet.getData().remove(chartDataSet.getData().size() - 1);

            long newSamplesSinceLastConsumed = totalSamplesPublished - lastConsumedTotalSamples;

            boolean updateIncrementally = canPatchIncrementally(sizeMatches, newSamplesSinceLastConsumed, lastConsumedRebuildCounter);

            if (updateIncrementally)
            {
               int index = bufferProperties.getOutPoint();
               for (int i = 0; i < newSamplesSinceLastConsumed; i++)
               {
                  chartDataSet.getData().get(index).set(index, dataSet.values[index]);
                  index = SharedMemoryTools.decrement(index, 1, dataSet.size);
               }
            }
            else
            {
               for (int i = 0; i < dataSet.size; i++)
                  chartDataSet.getData().get(i).set(i, dataSet.values[i]);
            }

            chartDataSet.bufferCurrentIndexProperty().set(bufferProperties.getCurrentIndex());
            chartDataSet.xBoundsProperty().setValue(new ChartIntegerBounds(0, dataSet.size));
            chartDataSet.yBoundsProperty().setValue(new ChartDoubleBounds(dataSet.valueMin, dataSet.valueMax));
         }
         finally
         {
            chartDataSet.getLock().writeLock().unlock();
            chartDataSet.markDirty();
         }
      }

      public long getTotalSamplesPublished()
      {
         return totalSamplesPublished;
      }

      public long getRebuildCounter()
      {
         return rebuildCounter;
      }

      /**
       * Whether {@link #dataSet} can be folded into {@code chartDataSet} by patching just the trailing
       * {@code newSamplesSinceLastConsumed} ring positions, instead of rewriting every point.
       */
      private boolean canPatchIncrementally(boolean sizeMatches, long newSamplesSinceLastConsumed, long lastConsumedRebuildCounter)
      {
         if (!sizeMatches)
            return false; // chart series is a different length than last render; ring positions can't be assumed to line up.
         if (rebuildCounter != lastConsumedRebuildCounter)
            return false; // a full rebuild happened since the caller's last render; ring layout may have changed arbitrarily.
         if (newSamplesSinceLastConsumed < 0)
            return false; // totalSamplesPublished went backward -- should never happen, guards against stale/bogus caller state.
         if (newSamplesSinceLastConsumed >= dataSet.size)
            return false; // more new samples arrived than the buffer holds; "just patch the trailing window" no longer covers everything that changed.
         return true;
      }
   }

   /** Package-private (rather than private) so its fields are directly testable from same-package unit tests. */
   static class DoubleArray
   {
      final int size;
      final double[] values;
      double valueMin, valueMax;

      DoubleArray(int size)
      {
         this.size = size;
         values = new double[size];
      }

      /** Deep copy, used to publish a safe-to-read snapshot while the source keeps being mutated in place. */
      DoubleArray deepCopy()
      {
         DoubleArray copy = new DoubleArray(size);
         System.arraycopy(values, 0, copy.values, 0, size);
         copy.valueMin = valueMin;
         copy.valueMax = valueMax;
         return copy;
      }
   }

   private static BufferSample<double[]> booleanToDoubleBuffer(BufferSample<?> yoVariableBuffer)
   {
      int from = yoVariableBuffer.getFrom();
      YoBufferPropertiesReadOnly bufferProperties = yoVariableBuffer.getBufferProperties();
      double[] sample = SharedMemoryTools.toDoubleArray((boolean[]) yoVariableBuffer.getSample());
      int sampleLength = yoVariableBuffer.getSampleLength();
      return new BufferSample<>(from, sample, sampleLength, bufferProperties);
   }

   private static BufferSample<double[]> byteToDoubleBuffer(BufferSample<?> yoVariableBuffer)
   {
      int from = yoVariableBuffer.getFrom();
      YoBufferPropertiesReadOnly bufferProperties = yoVariableBuffer.getBufferProperties();
      double[] sample = SharedMemoryTools.toDoubleArray((byte[]) yoVariableBuffer.getSample());
      int sampleLength = yoVariableBuffer.getSampleLength();
      return new BufferSample<>(from, sample, sampleLength, bufferProperties);
   }

   private static BufferSample<double[]> integerToDoubleBuffer(BufferSample<?> yoVariableBuffer)
   {
      int from = yoVariableBuffer.getFrom();
      YoBufferPropertiesReadOnly bufferProperties = yoVariableBuffer.getBufferProperties();
      double[] sample = SharedMemoryTools.toDoubleArray((int[]) yoVariableBuffer.getSample());
      int sampleLength = yoVariableBuffer.getSampleLength();
      return new BufferSample<>(from, sample, sampleLength, bufferProperties);
   }

   private static BufferSample<double[]> longToDoubleBuffer(BufferSample<?> yoVariableBuffer)
   {
      int from = yoVariableBuffer.getFrom();
      YoBufferPropertiesReadOnly bufferProperties = yoVariableBuffer.getBufferProperties();
      double[] sample = SharedMemoryTools.toDoubleArray((long[]) yoVariableBuffer.getSample());
      int sampleLength = yoVariableBuffer.getSampleLength();
      return new BufferSample<>(from, sample, sampleLength, bufferProperties);
   }
}
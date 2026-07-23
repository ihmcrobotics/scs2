package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.messager.Messager;
import us.ihmc.messager.TopicListener;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
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

   /** Sliding-window min/max candidate deques for {@link #lastDataSet}; {@code null} until the first rebuild. */
   private MonotonicIndexDeque maxCandidates, minCandidates;
   /**
    * Buffer state as of the last successfully-applied update (full rebuild or incremental patch).
    * Deliberately separate from {@link #lastProperties}, which is written asynchronously by
    * {@link #propertiesListener} from a different thread and can already be ahead of the
    * {@link BufferSample} being processed here -- diffing against it directly would silently corrupt the
    * incrementally-tracked min/max. {@code -1} means "not yet initialized", forcing a full rebuild.
    */
   private int appliedSize = -1, appliedInPoint = -1, appliedOutPoint = -1;
   private final Queue<Object> callerIDs = new ConcurrentLinkedQueue<>();
   private final Map<Object, ChartDataUpdate> newChartDataUpdate = new ConcurrentHashMap<>();

   private final AtomicBoolean requestEntireBuffer = new AtomicBoolean(true);
   private final AtomicBoolean requestUpdateBounds = new AtomicBoolean(false);

   @SuppressWarnings("rawtypes")
   private final Function<BufferSample<double[]>, BufferSample> bufferConverterFunction;

   private final TopicListener<CropBufferRequest> cropRequestListener = m -> requestEntireBuffer.set(true);
   private final TopicListener<FillBufferRequest> fillRequestListener = m -> requestEntireBuffer.set(true);
   private final TopicListener<Boolean> forceListenerUpdateListener = m -> requestEntireBuffer.set(true);
   private final TopicListener<YoBufferPropertiesReadOnly> propertiesListener;

   private final Messager messager;
   private final SessionVisualizerTopics topics;

   public YoVariableChartData(Messager messager, SessionVisualizerTopics topics, LinkedYoVariable<?> linkedYoVariable)
   {
      this.messager = messager;
      this.topics = topics;
      this.linkedYoVariable = linkedYoVariable;
      linkedYoVariable.addUser(this);
      currentSessionMode = messager.createInput(topics.getSessionCurrentMode(), SessionMode.PAUSE);

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

      messager.addTopicListener(topics.getYoBufferCropRequest(), cropRequestListener);
      messager.addTopicListener(topics.getYoBufferFillRequest(), fillRequestListener);
      messager.addTopicListener(topics.getYoBufferForceListenerUpdate(), forceListenerUpdateListener);
      messager.addTopicListener(topics.getYoBufferCurrentProperties(), propertiesListener);
   }

   public void dispose()
   {
      linkedYoVariable.removeUser(this);
      messager.removeInput(topics.getSessionCurrentMode(), currentSessionMode);
      messager.removeTopicListener(topics.getYoBufferCropRequest(), cropRequestListener);
      messager.removeTopicListener(topics.getYoBufferFillRequest(), fillRequestListener);
      messager.removeTopicListener(topics.getYoBufferCurrentProperties(), propertiesListener);
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
         appliedSize = -1; // Force a full rebuild (and fresh deques) on the next hasChartData tick.
      }

      DoubleArray publishedDataSet;

      if (hasChartData.get())
      {
         YoBufferPropertiesReadOnly newProperties = newBufferSample.getBufferProperties();

         if (canApplyIncrementally(appliedSize, appliedInPoint, appliedOutPoint, lastDataSet != null, newBufferSample))
         {
            double[] minMax = incrementallyPatchValuesAndBounds(lastDataSet.values, newBufferSample, appliedInPoint, maxCandidates, minCandidates);
            lastDataSet.valueMin = minMax[0];
            lastDataSet.valueMax = minMax[1];
            appliedInPoint = newProperties.getInPoint();
            appliedOutPoint = newProperties.getOutPoint();
            // Publish a snapshot, not the canonical array itself: lastDataSet.values keeps getting mutated
            // in place on future ticks, but a caller may still be reading a previously-published
            // ChartDataUpdate wrapping this same object on another thread.
            publishedDataSet = lastDataSet.snapshot();
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
               appliedSize = newProperties.getSize();
               appliedInPoint = newProperties.getInPoint();
               appliedOutPoint = newProperties.getOutPoint();
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

      ChartDataUpdate chartDataUpdate = new ChartDataUpdate(publishedDataSet, rawData.getBufferProperties());
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
    * Whether {@code newBufferSample} can be folded into the existing canonical dataset via the O(delta)
    * incremental path ({@link #incrementallyPatchValuesAndBounds}) instead of a full O(bufferSize)
    * {@link #rebuildEntireDataSet}. Requires a canonical dataset and primed deques to already exist for
    * the current buffer size, the new sample range to abut exactly where the last applied update's
    * out-point left off (a plain forward append, not a scrub/crop/gap), and the new in-point to be
    * reachable by walking forward from {@code appliedInPoint} in no more steps than samples just arrived
    * (i.e. eviction can never outpace insertion -- a backward/crop jump in the in-point would require an
    * implausibly large number of forward steps to explain, and gets rejected here). Package-private and
    * static so it is unit-testable without constructing a {@link YoVariableChartData} or the JavaFX
    * toolkit, same as {@link #computeDirtyRegion} in {@link NumberSeriesLayer}.
    */
   static boolean canApplyIncrementally(int appliedSize, int appliedInPoint, int appliedOutPoint, boolean canonicalDataSetPresent,
                                         BufferSample<double[]> newBufferSample)
   {
      if (!canonicalDataSetPresent || appliedSize < 0)
         return false;

      YoBufferPropertiesReadOnly newProperties = newBufferSample.getBufferProperties();
      int size = newProperties.getSize();
      if (size != appliedSize)
         return false;

      int expectedFrom = SharedMemoryTools.increment(appliedOutPoint, 1, size);
      if (newBufferSample.getFrom() != expectedFrom)
         return false;
      if (newBufferSample.getTo() != newProperties.getOutPoint())
         return false;

      int evictionSteps = SharedMemoryTools.computeSubLength(appliedInPoint, newProperties.getInPoint(), size) - 1;
      return evictionSteps <= newBufferSample.getSampleLength();
   }

   /**
    * Patches {@code values} and the two sliding-window deques in place for exactly what changed this
    * tick: indices that aged out of the active window (evicted from the deques if they were still a
    * candidate), then the newly-arrived sample range (written into {@code values} and pushed as new
    * candidates), in that order. Caller must have already verified eligibility via
    * {@link #canApplyIncrementally}.
    *
    * @return {@code {newMin, newMax}} read off the deque fronts after applying this tick's changes.
    */
   static double[] incrementallyPatchValuesAndBounds(double[] values, BufferSample<double[]> bufferSample, int appliedInPoint,
                                                       MonotonicIndexDeque maxCandidates, MonotonicIndexDeque minCandidates)
   {
      YoBufferPropertiesReadOnly bufferProperties = bufferSample.getBufferProperties();
      int bufferSize = bufferProperties.getSize();
      int newInPoint = bufferProperties.getInPoint();

      for (int evictIndex = appliedInPoint; evictIndex != newInPoint; evictIndex = SharedMemoryTools.increment(evictIndex, 1, bufferSize))
      {
         maxCandidates.evictIfFront(evictIndex);
         minCandidates.evictIfFront(evictIndex);
      }

      double[] sample = bufferSample.getSample();
      int insertIndex = bufferSample.getFrom();

      for (int i = 0; i < bufferSample.getSampleLength(); i++)
      {
         double value = sample[i];
         if (!Double.isFinite(value))
            value = 0.0;
         values[insertIndex] = value;
         maxCandidates.pushCandidate(insertIndex, value, values);
         minCandidates.pushCandidate(insertIndex, value, values);
         insertIndex = SharedMemoryTools.increment(insertIndex, 1, bufferSize);
      }

      return new double[] {values[minCandidates.peekFrontIndex()], values[maxCandidates.peekFrontIndex()]};
   }

   /**
    * Full O(bufferSize) rebuild, used when {@link #canApplyIncrementally} says the incremental path
    * doesn't apply (first load, buffer resize, or anything other than a plain forward append). Recomputes
    * every slot from scratch via the existing {@link #getValueAt} fallback logic, then primes
    * {@code maxCandidates}/{@code minCandidates} (assumed freshly constructed/cleared by the caller, sized
    * to the current buffer size) in the same pass -- replacing the old separate full rescan in
    * {@link #updateBounds} with equivalent deque-based bookkeeping, so subsequent ticks can use the
    * incremental path immediately.
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
      private final DoubleArray dataSet;
      private final YoBufferPropertiesReadOnly bufferProperties;

      public ChartDataUpdate(DoubleArray dataSet, YoBufferPropertiesReadOnly bufferProperties)
      {
         this.dataSet = dataSet;
         this.bufferProperties = bufferProperties;
      }

      public void readUpdate(NumberSeries chartDataSet, int lastUpdateEndIndex)
      {
         chartDataSet.getLock().writeLock().lock();

         try
         {
            // Resizing the cart data
            while (chartDataSet.getData().size() < dataSet.size)
               chartDataSet.getData().add(new Point2D());
            while (chartDataSet.getData().size() > dataSet.size)
               chartDataSet.getData().remove(chartDataSet.getData().size() - 1);

            for (int i = 0; i < dataSet.size; i++)
            {
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

      public int getUpdateEndIndex()
      {
         return bufferProperties.getOutPoint();
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
      DoubleArray snapshot()
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
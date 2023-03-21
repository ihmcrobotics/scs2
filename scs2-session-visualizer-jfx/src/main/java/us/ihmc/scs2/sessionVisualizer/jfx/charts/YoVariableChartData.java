package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

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
   private final Queue<Object> callerIDs = new ConcurrentLinkedQueue<>();
   private final Map<Object, ChartDataUpdate> newChartDataUpdate = new ConcurrentHashMap<>();

   private final AtomicBoolean requestEntireBuffer = new AtomicBoolean(true);
   private final AtomicBoolean requestUpdateBounds = new AtomicBoolean(false);

   @SuppressWarnings("rawtypes")
   private final Function<BufferSample<double[]>, BufferSample> bufferConverterFunction;

   private final TopicListener<CropBufferRequest> cropRequestListener = m -> requestEntireBuffer.set(true);
   private final TopicListener<FillBufferRequest> fillRequestListener = m -> requestEntireBuffer.set(true);
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

      @SuppressWarnings("rawtypes")
      BufferSample rawData = rawDataProperty.get();
      if (rawData == null || rawData.getSampleLength() == 0)
         return;

      @SuppressWarnings("unchecked")
      BufferSample<double[]> newBufferSample = bufferConverterFunction.apply(rawData);
      if (lastDataSet != null && newBufferSample.getBufferProperties().getSize() != lastDataSet.size)
         lastDataSet = null;

      DoubleArray dataSet;

      if (hasChartData.get())
         dataSet = updateDataSet(lastDataSet, newBufferSample);
      else if (updateBounds)
         dataSet = updateBounds(lastProperties, lastDataSet);
      else
         throw new IllegalStateException("Should not get here.");

      ChartDataUpdate chartDataUpdate = new ChartDataUpdate(dataSet, rawData.getBufferProperties());
      lastChartDataUpdate = chartDataUpdate;

      if (dataSet != null)
      {
         callerIDs.forEach(callerID -> newChartDataUpdate.put(callerID, chartDataUpdate));
         lastDataSet = dataSet;
      }

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

   public static DoubleArray updateDataSet(DoubleArray lastDataSet, BufferSample<double[]> bufferSample)
   {
      int sampleLength = bufferSample.getSampleLength();
      YoBufferPropertiesReadOnly bufferProperties = bufferSample.getBufferProperties();
      int bufferSize = bufferProperties.getSize();

      if (bufferSample == null || sampleLength == 0)
         return null;

      DoubleArray dataSet = new DoubleArray(bufferSize);

      dataSet.values[0] = getValueAt(0, lastDataSet, bufferSample);

      for (int i = 1; i < bufferSize; i++)
      {
         dataSet.values[i] = getValueAt(i, lastDataSet, bufferSample);
      }

      return updateBounds(bufferProperties, dataSet);
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

   private static class DoubleArray
   {
      private final int size;
      private final double[] values;
      private double valueMin, valueMax;

      public DoubleArray(int size)
      {
         this.size = size;
         values = new double[size];
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
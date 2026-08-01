package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.BitSet;

public abstract class YoVariableBuffer<T extends YoVariable>
{
   public static YoVariableBuffer<?> newYoVariableBuffer(YoVariable yoVariable, YoBufferPropertiesReadOnly properties)
   {
      if (yoVariable instanceof YoDouble)
         return new YoDoubleBuffer((YoDouble) yoVariable, properties);
      if (yoVariable instanceof YoInteger)
         return new YoIntegerBuffer((YoInteger) yoVariable, properties);
      if (yoVariable instanceof YoLong)
         return new YoLongBuffer((YoLong) yoVariable, properties);
      if (yoVariable instanceof YoBoolean)
         return new YoBooleanBuffer((YoBoolean) yoVariable, properties);
      if (yoVariable instanceof YoEnum)
         return new YoEnumBuffer<>((YoEnum<?>) yoVariable, properties);
      throw new UnsupportedOperationException("Unsupported YoVariable type: " + yoVariable.getClass().getSimpleName());
   }

   protected final T yoVariable;
   private final YoBufferPropertiesReadOnly properties;
   private final int variableMemorySize;

   /**
    * Supplies this buffer's value at an index it has no real data for yet. {@code null} (the default) means every
    * index must already have gone through {@link #writeBufferAt} or {@link #fillBuffer} - the original behavior. See
    * {@code LogSession}, the only current installer.
    */
   private volatile HistoricalValueBitsSource historicalValueBitsSource;

   /**
    * Tracks which indices already hold real data. Stays {@code null} - and unused - for as long as
    * {@link #historicalValueBitsSource} is {@code null}, since nothing needs tracking while every index is written
    * eagerly as before.
    */
   private BitSet populatedIndices;

   public YoVariableBuffer(T yoVariable, YoBufferPropertiesReadOnly properties)
   {
      this.yoVariable = yoVariable;
      this.properties = properties;
      variableMemorySize = SharedMemoryTools.getVariableMemorySize(yoVariable);
   }

   public int getVariableMemorySize()
   {
      return variableMemorySize;
   }

   /**
    * Installs the source consulted by {@link #copy} / {@link #readBufferAt} for an index that has no real data yet.
    * {@code null} (the default) disables the on-demand path entirely, so every index must go through
    * {@link #writeBufferAt}/{@link #fillBuffer} first, same as before this existed.
    */
   public void setHistoricalValueBitsSource(HistoricalValueBitsSource source)
   {
      historicalValueBitsSource = source;
      if (source != null && populatedIndices == null)
         populatedIndices = new BitSet();
   }

   public final void resizeBuffer(int from, int length)
   {
      resizeBufferRaw(from, length);
      // Conservative: a resize/crop reshuffles which buffer index holds which sample, so rather than try to carry
      // the bookkeeping through that reshuffle, just forget what was known and let it be re-derived from the source
      // on next access.
      if (populatedIndices != null)
         populatedIndices = new BitSet();
   }

   protected abstract void resizeBufferRaw(int from, int length);

   public final void writeBuffer()
   {
      writeBufferAt(properties.getCurrentIndex());
   }

   public final void writeBufferAt(int index)
   {
      writeBufferAtRaw(index);
      markPopulated(index, 1);
   }

   protected abstract void writeBufferAtRaw(int index);

   public final void readBuffer()
   {
      readBufferAt(properties.getCurrentIndex());
   }

   public final void readBufferAt(int index)
   {
      ensurePopulated(index, 1);
      readBufferAtRaw(index);
   }

   protected abstract void readBufferAtRaw(int index);

   public final void fillBuffer(boolean zeroFill, int from, int length)
   {
      fillBufferRaw(zeroFill, from, length);
      markPopulated(from, length);
   }

   protected abstract void fillBufferRaw(boolean zeroFill, int from, int length);

   public T getYoVariable()
   {
      return yoVariable;
   }

   public YoBufferPropertiesReadOnly getProperties()
   {
      return properties;
   }

   long getValueAsLongBits()
   {
      return getValueAsLongBits(properties.getCurrentIndex());
   }

   abstract long getValueAsLongBits(int index);

   @SuppressWarnings("rawtypes")
   public final BufferSample copy(int from, int length, YoBufferPropertiesReadOnly properties)
   {
      ensurePopulated(from, length);
      return copyRaw(from, length, properties);
   }

   @SuppressWarnings("rawtypes")
   protected abstract BufferSample copyRaw(int from, int length, YoBufferPropertiesReadOnly properties);

   abstract LinkedYoVariable<T> newLinkedYoVariable(T variableToLink, Object initialUser);

   public abstract Object getBuffer();

   public abstract double[] getAsDoubleBuffer();

   public abstract void dispose();

   /**
    * For each index in {@code [from, from + length)} (ring-wrapped the same way as e.g.
    * {@link SharedMemoryTools#ringArrayCopy}) not yet known to hold real data, asks {@link #historicalValueBitsSource}
    * for it, stores it through the ordinary {@link #writeBufferAtRaw} path by briefly swapping it onto
    * {@link #yoVariable}, then restores the variable's actual live value. A no-op whenever
    * {@link #historicalValueBitsSource} is {@code null}, i.e. for every buffer except the ones a log session created
    * on demand.
    */
   private void ensurePopulated(int from, int length)
   {
      HistoricalValueBitsSource source = historicalValueBitsSource;
      if (source == null || length <= 0)
         return;

      int size = properties.getSize();

      for (int i = 0; i < length; i++)
      {
         int index = (from + i) % size;
         if (populatedIndices.get(index))
            continue;

         long liveBits = yoVariable.getValueAsLongBits();
         yoVariable.setValueFromLongBits(source.getHistoricalValueBits(yoVariable, index), false);
         writeBufferAtRaw(index);
         yoVariable.setValueFromLongBits(liveBits, false);

         populatedIndices.set(index);
      }
   }

   private void markPopulated(int from, int length)
   {
      if (populatedIndices == null || length <= 0)
         return;

      int size = properties.getSize();
      for (int i = 0; i < length; i++)
         populatedIndices.set((from + i) % size);
   }

   @Override
   public String toString()
   {
      return "%s [yoVariable=%s, properties=%s]".formatted(getClass().getSimpleName(), yoVariable, properties);
   }
}

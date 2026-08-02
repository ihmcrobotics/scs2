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

   /**
    * How many bits are set in {@link #populatedIndices}, kept incrementally so {@link #ensurePopulated} can tell in
    * O(1) whether the whole buffer is already backfilled - once {@code populatedCount == properties.getSize()}, every
    * index this buffer could ever be asked for is already real data, permanently (a ring buffer only ever reuses an
    * index by writing genuine live data over it via {@link #writeBufferAt}, never by un-writing one), so there is
    * nothing left for {@link #historicalValueBitsSource} to ever contribute again. Without this, a buffer that had a
    * source installed keeps paying to walk the requested range and check every bit against {@link #populatedIndices}
    * on every single read for the rest of the session, long after backfilling actually finished - unlike a buffer
    * that never had a source, which always takes the {@code source == null} fast exit.
    */
   private int populatedCount;

   /**
    * Caps how many indices one {@link HistoricalValueBitsSource} call covers when running under a budget, so the
    * budget actually gets consulted as the range is worked through. Large enough that a source's per-call seek is
    * still amortized across the run.
    */
   private static final int MAX_BUDGETED_RUN_LENGTH = 256;

   /**
    * The allowance {@link #ensurePopulatedWithinBudget} works against, shared with every other buffer drawing on a
    * source. {@code null} (the default) means unbudgeted, same as buffers that have no source at all.
    */
   private HistoricalBackfillBudget backfillBudget;

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

   /**
    * Installs the shared per-publish-cycle allowance {@link #ensurePopulatedWithinBudget} spends against.
    * {@code null} (the default) leaves that method unbudgeted, i.e. it always runs the range to completion.
    */
   public void setBackfillBudget(HistoricalBackfillBudget budget)
   {
      backfillBudget = budget;
   }

   public final void resizeBuffer(int from, int length)
   {
      // Captured before resizeBufferRaw: the callers in YoSharedMemory (cropBuffer, resizeBuffer) only push the new
      // size onto properties after this returns, so this is still the pre-resize size.
      int oldSize = properties.getSize();
      resizeBufferRaw(from, length);
      // A resize/crop shuffles the data - new index j now holds what old index (from + j) held - so the bookkeeping
      // has to follow it. Rebuilding it as empty instead would discard the fact that the data resizeBufferRaw just
      // carried over is real, and the whole range would be re-fetched from historicalValueBitsSource on next access,
      // overwriting that correct data with whatever the source resolves those (now different) indices to.
      if (populatedIndices != null)
         remapPopulatedIndices(from, length, oldSize);
   }

   /**
    * Applies the same shift {@link SharedMemoryTools#ringArrayCopy} applies to the data itself: new index {@code j}
    * takes what old index {@code (from + j) % oldSize} had, for as many indices as actually carry over. When growing,
    * the indices past the old size hold no carried-over data (ringArrayCopy leaves them at the default value), so
    * they stay unpopulated and are backfilled on next access like any other never-written index.
    */
   private void remapPopulatedIndices(int from, int length, int oldSize)
   {
      int carriedOver = Math.min(length, oldSize);
      BitSet remapped = new BitSet();
      int remappedCount = 0;

      for (int j = 0; j < carriedOver; j++)
      {
         if (populatedIndices.get((from + j) % oldSize))
         {
            remapped.set(j);
            remappedCount++;
         }
      }

      populatedIndices = remapped;
      populatedCount = remappedCount;
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
      // Unbudgeted: this sets the variable's actual value for the current index, so there is no later cycle to defer
      // to, and it is a single index either way.
      ensurePopulated(index, 1, null);
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

   /**
    * Note this backfills unbudgeted, i.e. it blocks until the whole range is real - callers that can tolerate waiting
    * a cycle should gate on {@link #ensurePopulatedWithinBudget} first, which is what {@link LinkedYoVariable} does.
    */
   @SuppressWarnings("rawtypes")
   public final BufferSample copy(int from, int length, YoBufferPropertiesReadOnly properties)
   {
      ensurePopulated(from, length, null);
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
    * {@link #historicalValueBitsSource} is {@code null} (every buffer except the ones a log session created on
    * demand) or once every index in this buffer has already been populated (see {@link #populatedCount}).
    * <p>
    * Not-yet-populated indices are batched into the longest possible non-wrapping run and handed to
    * {@link #historicalValueBitsSource} in one {@link HistoricalValueBitsSource#getHistoricalValueBits} call rather
    * than one call per index: for a log-backed source, one call per index means one seek (and, for a
    * compressed/batched log, one decompression) per single value, even for values that are consecutive in the log -
    * e.g. backfilling a chart's whole history for a variable just linked. See {@code LogSession}, the only current
    * source, for how it uses the batched range to pay for that seek/decompression once per contiguous run instead.
    * </p>
    *
    * @param budget when non-null, stop early once this cycle's allowance is spent, leaving the rest of the range for
    *               a later call. {@code null} runs the range to completion however long that takes.
    * @return {@code true} if every index in the range now holds real data.
    */
   private boolean ensurePopulated(int from, int length, HistoricalBackfillBudget budget)
   {
      HistoricalValueBitsSource source = historicalValueBitsSource;
      if (source == null || length <= 0 || populatedCount >= properties.getSize())
         return true;

      int size = properties.getSize();
      int remaining = length;
      int cursor = ((from % size) + size) % size;

      while (remaining > 0)
      {
         if (populatedIndices.get(cursor))
         {
            cursor = cursor + 1 == size ? 0 : cursor + 1;
            remaining--;
            continue;
         }

         if (budget != null && budget.isExhausted())
            return false;

         int maxRunLength = Math.min(remaining, size - cursor);
         // Under a budget, cap how much is fetched before it is consulted again. Runs exist to amortize the source's
         // per-call seek, which a few hundred indices already does; letting one run cover the whole buffer would
         // mean the budget is only ever checked once, which defeats it.
         if (budget != null)
            maxRunLength = Math.min(maxRunLength, MAX_BUDGETED_RUN_LENGTH);

         int runLength = 1;
         while (runLength < maxRunLength && !populatedIndices.get(cursor + runLength))
            runLength++;

         int runStart = cursor;
         long liveBits = yoVariable.getValueAsLongBits();
         source.getHistoricalValueBits(yoVariable, runStart, runLength, (index, bits) ->
         {
            yoVariable.setValueFromLongBits(bits, false);
            writeBufferAtRaw(index);
            setPopulated(index);
         });
         yoVariable.setValueFromLongBits(liveBits, false);

         cursor = (cursor + runLength) % size;
         remaining -= runLength;
      }

      return true;
   }

   /**
    * Backfills as much of {@code [from, from + length)} as {@link #backfillBudget} still allows this publish cycle.
    * <p>
    * This is the entry point for consumers that can come back later - see {@link LinkedYoVariable#prepareForPull} -
    * as opposed to {@link #copy} and {@link #readBufferAt}, which need the data immediately and so run the backfill
    * to completion.
    * </p>
    *
    * @return {@code true} if the whole range now holds real data, {@code false} if the caller should ask again next
    *       cycle.
    */
   public boolean ensurePopulatedWithinBudget(int from, int length)
   {
      return ensurePopulated(from, length, backfillBudget);
   }

   private void markPopulated(int from, int length)
   {
      if (populatedIndices == null || length <= 0)
         return;

      int size = properties.getSize();
      for (int i = 0; i < length; i++)
         setPopulated((from + i) % size);
   }

   private void setPopulated(int index)
   {
      if (!populatedIndices.get(index))
      {
         populatedIndices.set(index);
         populatedCount++;
      }
   }

   @Override
   public String toString()
   {
      return "%s [yoVariable=%s, properties=%s]".formatted(getClass().getSimpleName(), yoVariable, properties);
   }
}

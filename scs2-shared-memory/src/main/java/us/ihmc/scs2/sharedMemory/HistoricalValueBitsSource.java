package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Supplies a {@link YoVariableBuffer}'s value for indices it has no real data for yet, in the same on-demand spirit
 * as {@link YoRegistryBuffer#setEagerVariableFilter} skipping buffer allocation up front.
 * <p>
 * Installed per-buffer via {@link YoVariableBuffer#setHistoricalValueBitsSource} - {@code LogSession} is the only
 * current implementor, backing this with the log file itself so a variable buffered on demand well after its log had
 * already played through doesn't show blank history for everything before that point.
 * </p>
 */
public interface HistoricalValueBitsSource
{
   /**
    * Supplies {@code variable}'s value for every index in the contiguous, non-wrapping range
    * {@code [from, from + length)}, invoking {@code writer} once per index. The range is always requested in
    * ascending order, one index apart, which lets an implementation backed by a sequential source (e.g. a log file)
    * service it in a single pass - one seek instead of one per index - rather than the caller looping over
    * {@code getHistoricalValueBits(variable, index)} one at a time, which pathologically forces a fresh seek (and,
    * for a compressed/batched log, a fresh decompression) per single value.
    *
    * @param variable the buffer's own variable, i.e. {@link YoVariableBuffer#getYoVariable()}.
    * @param from     the first buffer index being requested.
    * @param length   how many consecutive indices, starting at {@code from}, are being requested.
    * @param writer   invoked once per index in the range with the raw bits {@code variable} held there, encoded the
    *                 same way as {@link YoVariable#getValueAsLongBits()}.
    */
   void getHistoricalValueBits(YoVariable variable, int from, int length, HistoricalValueBitsWriter writer);
}

package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Supplies a {@link YoVariableBuffer}'s value at an index it has no real data for yet, in the same on-demand spirit
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
    * @param variable the buffer's own variable, i.e. {@link YoVariableBuffer#getYoVariable()}.
    * @param index    the buffer index being requested.
    * @return the raw bits {@code variable} held at {@code index}, encoded the same way as
    *       {@link YoVariable#getValueAsLongBits()}.
    */
   long getHistoricalValueBits(YoVariable variable, int index);
}

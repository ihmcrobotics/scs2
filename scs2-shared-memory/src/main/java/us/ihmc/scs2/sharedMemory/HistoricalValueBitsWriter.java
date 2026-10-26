package us.ihmc.scs2.sharedMemory;

/**
 * Callback {@link HistoricalValueBitsSource#getHistoricalValueBits} invokes once per index in the requested range.
 */
@FunctionalInterface
public interface HistoricalValueBitsWriter
{
   void write(int index, long valueBits);
}

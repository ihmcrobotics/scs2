package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoLong;

public class YoLongBuffer extends YoVariableBuffer<YoLong>
{
   private long[] buffer;

   public YoLongBuffer(YoLong yoLong, YoBufferPropertiesReadOnly properties)
   {
      super(yoLong, properties);
      buffer = new long[properties.getSize()];
   }

   @Override
   protected void resizeBufferRaw(int from, int length)
   {
      if (from == 0 && length == buffer.length)
         return;
      buffer = SharedMemoryTools.ringArrayCopy(buffer, from, length);
   }

   @Override
   protected void writeBufferAtRaw(int index)
   {
      buffer[index] = yoVariable.getValue();
   }

   @Override
   protected void readBufferAtRaw(int index)
   {
      yoVariable.set(buffer[index]);
   }

   @Override
   long getValueAsLongBits(int index)
   {
      return buffer[index];
   }

   @Override
   protected BufferSample<long[]> copyRaw(int from, int length, YoBufferPropertiesReadOnly properties)
   {
      return new BufferSample<>(from, SharedMemoryTools.ringArrayCopy(buffer, from, length), length, properties);
   }

   @Override
   protected void fillBufferRaw(boolean zeroFill, int from, int length)
   {
      SharedMemoryTools.ringArrayFill(buffer, zeroFill ? 0 : yoVariable.getValue(), from, length);
   }

   @Override
   LinkedYoLong newLinkedYoVariable(YoLong variableToLink, Object initialUser)
   {
      return new LinkedYoLong(variableToLink, this, initialUser);
   }

   @Override
   public long[] getBuffer()
   {
      return buffer;
   }

   @Override
   public double[] getAsDoubleBuffer()
   {
      return SharedMemoryTools.toDoubleArray(buffer);
   }

   @Override
   public void dispose()
   {
      buffer = null;
   }
}

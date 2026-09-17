package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;

public class YoBooleanBuffer extends YoVariableBuffer<YoBoolean>
{
   private boolean[] buffer;

   public YoBooleanBuffer(YoBoolean yoBoolean, YoBufferPropertiesReadOnly properties)
   {
      super(yoBoolean, properties);
      buffer = new boolean[properties.getSize()];
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
      return buffer[index] ? 1 : 0;
   }

   @Override
   protected BufferSample<boolean[]> copyRaw(int from, int length, YoBufferPropertiesReadOnly properties)
   {
      return new BufferSample<>(from, SharedMemoryTools.ringArrayCopy(buffer, from, length), length, properties);
   }

   @Override
   protected void fillBufferRaw(boolean zeroFill, int from, int length)
   {
      SharedMemoryTools.ringArrayFill(buffer, zeroFill ? false : yoVariable.getValue(), from, length);
   }

   @Override
   LinkedYoBoolean newLinkedYoVariable(YoBoolean variableToLink, Object initialUser)
   {
      return new LinkedYoBoolean(variableToLink, this, initialUser);
   }

   @Override
   public boolean[] getBuffer()
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

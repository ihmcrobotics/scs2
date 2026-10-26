package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoDoubleBuffer extends YoVariableBuffer<YoDouble>
{
   private double[] buffer;

   public YoDoubleBuffer(YoDouble yoDouble, YoBufferPropertiesReadOnly properties)
   {
      super(yoDouble, properties);
      buffer = new double[properties.getSize()];
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
      return Double.doubleToLongBits(buffer[index]);
   }

   @Override
   protected BufferSample<double[]> copyRaw(int from, int length, YoBufferPropertiesReadOnly properties)
   {
      return new BufferSample<>(from, SharedMemoryTools.ringArrayCopy(buffer, from, length), length, properties);
   }

   @Override
   protected void fillBufferRaw(boolean zeroFill, int from, int length)
   {
      SharedMemoryTools.ringArrayFill(buffer, zeroFill ? 0.0 : yoVariable.getValue(), from, length);
   }

   @Override
   LinkedYoDouble newLinkedYoVariable(YoDouble variableToLink, Object initialUser)
   {
      return new LinkedYoDouble(variableToLink, this, initialUser);
   }

   @Override
   public double[] getBuffer()
   {
      return buffer;
   }

   @Override
   public double[] getAsDoubleBuffer()
   {
      return buffer;
   }

   @Override
   public void dispose()
   {
      buffer = null;
   }
}

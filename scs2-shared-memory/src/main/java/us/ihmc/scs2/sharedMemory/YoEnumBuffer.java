package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoEnumBuffer<E extends Enum<E>> extends YoVariableBuffer<YoEnum<E>>
{
   private byte[] buffer;

   public YoEnumBuffer(YoEnum<E> yoEnum, YoBufferPropertiesReadOnly properties)
   {
      super(yoEnum, properties);
      buffer = new byte[properties.getSize()];
   }

   @Override
   public void resizeBuffer(int from, int length)
   {
      if (from == 0 && length == buffer.length)
         return;
      buffer = SharedMemoryTools.ringArrayCopy(buffer, from, length);
   }

   @Override
   public void writeBufferAt(int index)
   {
      buffer[index] = (byte) yoVariable.getOrdinal();
   }

   @Override
   public void readBufferAt(int index)
   {
      yoVariable.set(buffer[index]);
   }

   @Override
   long getValueAsLongBits(int index)
   {
      return buffer[index];
   }

   @Override
   public BufferSample<byte[]> copy(int from, int length, YoBufferPropertiesReadOnly properties)
   {
      return new BufferSample<>(from, SharedMemoryTools.ringArrayCopy(buffer, from, length), length, properties);
   }

   @Override
   public void fillBuffer(boolean zeroFill, int from, int length)
   {
      SharedMemoryTools.ringArrayFill(buffer, zeroFill ? 0 : (byte) yoVariable.getOrdinal(), from, length);
   }

   @Override
   LinkedYoEnum<E> newLinkedYoVariable(YoEnum<E> variableToLink)
   {
      return new LinkedYoEnum<>(variableToLink, this);
   }

   @Override
   public byte[] getBuffer()
   {
      return buffer;
   }

   @Override
   public void dispose()
   {
      buffer = null;
   }
}

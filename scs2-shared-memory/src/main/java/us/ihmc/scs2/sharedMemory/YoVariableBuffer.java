package us.ihmc.scs2.sharedMemory;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.variable.*;

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

   public YoVariableBuffer(T yoVariable, YoBufferPropertiesReadOnly properties)
   {
      this.yoVariable = yoVariable;
      this.properties = properties;
   }

   public abstract void resizeBuffer(int from, int length);

   public final void writeBuffer()
   {
      writeBufferAt(properties.getCurrentIndex());
   }

   public abstract void writeBufferAt(int index);

   public final void readBuffer()
   {
      readBufferAt(properties.getCurrentIndex());
   }

   public abstract void readBufferAt(int index);

   public abstract void fillBuffer(boolean zeroFill, int from, int length);

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
   public abstract BufferSample copy(int from, int length, YoBufferPropertiesReadOnly properties);

   abstract LinkedYoVariable<T> newLinkedYoVariable(T variableToLink, Object initialUser);

   public abstract Object getBuffer();

   public abstract double[] getAsDoubleBuffer();

   public abstract void dispose();
}

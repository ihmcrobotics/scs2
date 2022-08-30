package us.ihmc.scs2.sessionVisualizer.jfx.properties;

import java.lang.ref.WeakReference;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.DoublePropertyBase;
import javafx.beans.property.Property;
import us.ihmc.scs2.sharedMemory.LinkedYoDouble;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoDoubleProperty extends DoublePropertyBase implements YoVariableProperty<YoDouble, Number>
{
   private final YoDouble yoDouble;
   private final Object bean;
   private final YoVariableChangedListener propertyUpdater = new YoDoublePropertyUpdater(this);

   private LinkedYoDouble linkedBuffer;

   public YoDoubleProperty(YoDouble yoDouble)
   {
      this(yoDouble, null);
   }

   public YoDoubleProperty(YoDouble yoDouble, Object bean)
   {
      this.yoDouble = yoDouble;
      this.bean = bean;
      pullYoDoubleValue();
      yoDouble.addListener(propertyUpdater);
   }

   public void setLinkedBuffer(LinkedYoDouble linkedBuffer)
   {
      if (this.linkedBuffer != null)
         this.linkedBuffer.removeUser(this);

      this.linkedBuffer = linkedBuffer;

      if (linkedBuffer != null)
      {
         linkedBuffer.addUser(this);
         pullYoDoubleValue();
      }
   }

   @Override
   public LinkedYoDouble getLinkedBuffer()
   {
      return linkedBuffer;
   }

   @Override
   public void finalize()
   {
      dispose();
   }

   @Override
   public void dispose()
   {
      try
      {
         yoDouble.removeListener(propertyUpdater);
         if (linkedBuffer != null)
            linkedBuffer.removeUser(this);
      }
      finally
      {
      }
   }

   @Override
   public void set(double newValue)
   {
      super.set(newValue);
      yoDouble.set(newValue);
      if (linkedBuffer != null)
         linkedBuffer.push();
   }

   private void pullYoDoubleValue()
   {
      super.set(yoDouble.getValue());
   }

   public void bindDoubleProperty(Property<Double> property)
   {
      property.setValue(getValue());

      MutableBoolean updatingControl = new MutableBoolean(false);
      MutableBoolean updatingThis = new MutableBoolean(false);

      addListener((o, oldValue, newValue) ->
      { // YoVariable changed, updating control
         if (updatingThis.isTrue())
            return;

         updatingControl.setTrue();
         property.setValue(Double.valueOf(newValue.doubleValue()));
         updatingControl.setFalse();
      });

      property.addListener((o, oldValue, newValue) ->
      {
         if (updatingControl.isTrue())
            return;

         updatingThis.setTrue();
         set(newValue.doubleValue());
         updatingThis.setFalse();
      });
   }

   @Override
   public YoDouble getYoVariable()
   {
      return yoDouble;
   }

   @Override
   public Object getBean()
   {
      return bean;
   }

   @Override
   public String getName()
   {
      return yoDouble.getName();
   }

   private static class YoDoublePropertyUpdater implements YoVariableChangedListener
   {
      private final WeakReference<YoDoubleProperty> propertyRef;

      public YoDoublePropertyUpdater(YoDoubleProperty property)
      {
         propertyRef = new WeakReference<>(property);
      }

      @Override
      public void changed(YoVariable source)
      {
         YoDoubleProperty property = propertyRef.get();
         if (property != null)
            property.pullYoDoubleValue();
      }
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.properties;

import java.lang.ref.WeakReference;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.IntegerPropertyBase;
import javafx.beans.property.Property;
import us.ihmc.scs2.sharedMemory.LinkedYoInteger;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoIntegerProperty extends IntegerPropertyBase implements YoVariableProperty<YoInteger, Number>
{
   private final YoInteger yoInteger;
   private final Object bean;
   private final YoVariableChangedListener propertyUpdater = new YoIntegerPropertyUpdater(this);

   private LinkedYoInteger linkedBuffer;

   public YoIntegerProperty(YoInteger yoInteger)
   {
      this(yoInteger, null);
   }

   public YoIntegerProperty(YoInteger yoInteger, Object bean)
   {
      this.yoInteger = yoInteger;
      this.bean = bean;
      pullYoIntegerValue();
      yoInteger.addListener(propertyUpdater);
   }

   public void setLinkedBuffer(LinkedYoInteger linkedBuffer)
   {
      if (this.linkedBuffer != null)
         this.linkedBuffer.removeUser(this);

      this.linkedBuffer = linkedBuffer;

      if (linkedBuffer != null)
      {
         linkedBuffer.addUser(this);
         pullYoIntegerValue();
      }
   }

   @Override
   public LinkedYoInteger getLinkedBuffer()
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
         unbind();
         yoInteger.removeListener(propertyUpdater);
         if (linkedBuffer != null)
         {
            linkedBuffer.removeUser(this);
            pullYoIntegerValue();
         }
      }
      finally
      {
      }
   }

   @Override
   public void set(int newValue)
   {
      super.set(newValue);
      yoInteger.set(newValue);
      if (linkedBuffer != null)
         linkedBuffer.push();
   }

   private void pullYoIntegerValue()
   {
      super.set(yoInteger.getValue());
   }

   public void bindIntegerProperty(Property<Integer> property)
   {
      property.setValue(getValue());

      MutableBoolean updatingControl = new MutableBoolean(false);
      MutableBoolean updatingThis = new MutableBoolean(false);

      addListener((o, oldValue, newValue) ->
      { // YoVariable changed, updating control
         if (updatingThis.isTrue())
            return;

         updatingControl.setTrue();
         property.setValue(Integer.valueOf(newValue.intValue()));
         updatingControl.setFalse();
      });

      property.addListener((o, oldValue, newValue) ->
      {
         if (updatingControl.isTrue())
            return;

         updatingThis.setTrue();
         set(newValue.intValue());
         updatingThis.setFalse();
      });
   }

   @Override
   public YoInteger getYoVariable()
   {
      return yoInteger;
   }

   @Override
   public Object getBean()
   {
      return bean;
   }

   @Override
   public String getName()
   {
      return yoInteger.getName();
   }

   private static class YoIntegerPropertyUpdater implements YoVariableChangedListener
   {
      private final WeakReference<YoIntegerProperty> propertyRef;

      public YoIntegerPropertyUpdater(YoIntegerProperty property)
      {
         propertyRef = new WeakReference<>(property);
      }

      @Override
      public void changed(YoVariable source)
      {
         YoIntegerProperty property = propertyRef.get();
         if (property != null)
            property.pullYoIntegerValue();
      }
   }
}

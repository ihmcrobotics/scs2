package us.ihmc.scs2.sessionVisualizer.jfx.properties;

import java.lang.ref.WeakReference;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.LongPropertyBase;
import javafx.beans.property.Property;
import us.ihmc.scs2.sharedMemory.LinkedYoLong;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoLongProperty extends LongPropertyBase implements YoVariableProperty<YoLong, Number>
{
   private final YoLong yoLong;
   private final Object bean;
   private final YoVariableChangedListener propertyUpdater = new YoLongPropertyUpdater(this);

   private LinkedYoLong linkedBuffer;

   public YoLongProperty(YoLong yoLong)
   {
      this(yoLong, null);
   }

   public YoLongProperty(YoLong yoLong, Object bean)
   {
      this.yoLong = yoLong;
      this.bean = bean;
      pullYoLongValue();
      yoLong.addListener(propertyUpdater);
   }

   private Object userObject;

   public void setLinkedBuffer(LinkedYoLong linkedBuffer)
   {
      if (this.linkedBuffer != null)
         this.linkedBuffer.removeUser(userObject);

      this.linkedBuffer = linkedBuffer;

      if (userObject == null)
         userObject = new Object();

      if (linkedBuffer != null)
         linkedBuffer.addUser(userObject);
   }

   @Override
   public LinkedYoLong getLinkedBuffer()
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
         yoLong.removeListener(propertyUpdater);
         if (linkedBuffer != null)
         {
            linkedBuffer.removeUser(userObject);
            pullYoLongValue();
         }
      }
      finally
      {
      }
   }

   @Override
   public void set(long newValue)
   {
      super.set(newValue);
      yoLong.set(newValue);
      if (linkedBuffer != null)
         linkedBuffer.push();
   }

   private void pullYoLongValue()
   {
      super.set(yoLong.getValue());
   }

   public void bindLongProperty(Property<Long> property)
   {
      property.setValue(getValue());

      MutableBoolean updatingControl = new MutableBoolean(false);
      MutableBoolean updatingThis = new MutableBoolean(false);

      addListener((o, oldValue, newValue) ->
      { // YoVariable changed, updating control
         if (updatingThis.isTrue())
            return;

         updatingControl.setTrue();
         property.setValue(Long.valueOf(newValue.longValue()));
         updatingControl.setFalse();
      });

      property.addListener((o, oldValue, newValue) ->
      {
         if (updatingControl.isTrue())
            return;

         updatingThis.setTrue();
         set(newValue.longValue());
         updatingThis.setFalse();
      });
   }

   @Override
   public YoLong getYoVariable()
   {
      return yoLong;
   }

   @Override
   public Object getBean()
   {
      return bean;
   }

   @Override
   public String getName()
   {
      return yoLong.getName();
   }

   private static class YoLongPropertyUpdater implements YoVariableChangedListener
   {
      private final WeakReference<YoLongProperty> propertyRef;

      public YoLongPropertyUpdater(YoLongProperty property)
      {
         propertyRef = new WeakReference<>(property);
      }

      @Override
      public void changed(YoVariable source)
      {
         YoLongProperty property = propertyRef.get();
         if (property != null)
            property.pullYoLongValue();
      }
   }
}

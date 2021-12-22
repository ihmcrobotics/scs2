package us.ihmc.scs2.sessionVisualizer.jfx.properties;

import java.lang.ref.WeakReference;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.IntegerPropertyBase;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleIntegerProperty;
import us.ihmc.scs2.sharedMemory.LinkedYoInteger;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoIntegerProperty extends IntegerPropertyBase implements YoVariableProperty<YoInteger, Number>
{
   private final YoInteger yoInteger;
   private final Object bean;
   private final YoVariableChangedListener propertyUpdater = new YoIntegerPropertyUpdater(this);

   private SimpleIntegerProperty lastUserInput;

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

   private Object userObject;

   public void setLinkedBuffer(LinkedYoInteger linkedBuffer)
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
   public LinkedYoInteger getLinkedBuffer()
   {
      return linkedBuffer;
   }

   @Override
   public void finalize()
   {
      try
      {
         yoInteger.removeListener(propertyUpdater);
         if (linkedBuffer != null)
            linkedBuffer.removeUser(userObject);
      }
      finally
      {
      }
   }

   @Override
   public void set(int newValue)
   {
      if (lastUserInput != null)
         lastUserInput.set(newValue);
      super.set(newValue);
      yoInteger.set(newValue);
   }

   public void setAndPush(int newValue)
   {
      set(newValue);
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
         setAndPush(newValue.intValue());
         updatingThis.setFalse();
      });
   }

   @Override
   public IntegerProperty userInputProperty()
   {
      if (lastUserInput == null)
         lastUserInput = new SimpleIntegerProperty(this, getName() + "LastUserInput", get());
      return lastUserInput;
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

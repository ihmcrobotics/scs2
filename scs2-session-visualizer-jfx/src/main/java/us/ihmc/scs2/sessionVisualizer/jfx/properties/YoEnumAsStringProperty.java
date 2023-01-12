package us.ihmc.scs2.sessionVisualizer.jfx.properties;

import java.lang.ref.WeakReference;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.Property;
import javafx.beans.property.StringPropertyBase;
import us.ihmc.scs2.sharedMemory.LinkedYoEnum;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoEnumAsStringProperty<E extends Enum<E>> extends StringPropertyBase implements YoVariableProperty<YoEnum<E>, String>
{
   private final YoEnum<E> yoEnum;
   private final Object bean;
   private final YoVariableChangedListener propertyUpdater = new YoEnumPropertyUpdater(this);
   private final List<String> enumConstants;

   private LinkedYoEnum<E> linkedBuffer;

   public YoEnumAsStringProperty(YoEnum<E> yoEnum)
   {
      this(yoEnum, null);
   }

   public YoEnumAsStringProperty(YoEnum<E> yoEnum, Object bean)
   {
      this.yoEnum = yoEnum;
      this.bean = bean;

      enumConstants = Arrays.asList(yoEnum.getEnumValuesAsString());
      pullYoEnumValue();
      yoEnum.addListener(propertyUpdater);
   }

   public void setLinkedBuffer(LinkedYoEnum<E> linkedBuffer)
   {
      if (this.linkedBuffer != null)
         this.linkedBuffer.removeUser(this);

      this.linkedBuffer = linkedBuffer;

      if (linkedBuffer != null)
      {
         linkedBuffer.addUser(this);
         pullYoEnumValue();
      }
   }

   @Override
   public LinkedYoEnum<E> getLinkedBuffer()
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
         yoEnum.removeListener(propertyUpdater);
         if (linkedBuffer != null)
            linkedBuffer.removeUser(this);
      }
      finally
      {
      }
   }

   @Override
   public void set(String newValue)
   {
      super.set(newValue);
      yoEnum.set(toEnumOrdinal(newValue));
      if (linkedBuffer != null)
         linkedBuffer.push();
   }

   private void pullYoEnumValue()
   {
      super.set(toEnumString(yoEnum.getOrdinal()));
   }

   public void bindStringProperty(Property<String> property)
   {
      property.setValue(getValue());

      MutableBoolean updatingControl = new MutableBoolean(false);
      MutableBoolean updatingThis = new MutableBoolean(false);

      addListener((o, oldValue, newValue) ->
      { // YoVariable changed, updating control
         if (updatingThis.isTrue())
            return;

         updatingControl.setTrue();
         property.setValue(newValue);
         updatingControl.setFalse();
      });

      property.addListener((o, oldValue, newValue) ->
      {
         if (updatingControl.isTrue())
            return;

         updatingThis.setTrue();
         set(newValue);
         updatingThis.setFalse();
      });
   }

   public int toEnumOrdinal(String newValue)
   {
      if (newValue == null || newValue.equals(YoEnum.NULL_VALUE_STRING))
         return YoEnum.NULL_VALUE;

      int indexOf = enumConstants.indexOf(newValue);
      if (indexOf == -1)
         throw new IllegalArgumentException("The value " + newValue + " is not part of the constants of the YoEnum: " + yoEnum.getName());
      return indexOf;
   }

   public String toEnumString(int ordinal)
   {
      if (ordinal == -1)
         return null;
      else
         return enumConstants.get(yoEnum.getOrdinal());
   }

   @Override
   public YoEnum<E> getYoVariable()
   {
      return yoEnum;
   }

   @Override
   public Object getBean()
   {
      return bean;
   }

   @Override
   public String getName()
   {
      return yoEnum.getName();
   }

   private static class YoEnumPropertyUpdater implements YoVariableChangedListener
   {
      private final WeakReference<YoEnumAsStringProperty<?>> propertyRef;

      public YoEnumPropertyUpdater(YoEnumAsStringProperty<?> property)
      {
         propertyRef = new WeakReference<>(property);
      }

      @Override
      public void changed(YoVariable source)
      {
         YoEnumAsStringProperty<?> property = propertyRef.get();
         if (property != null)
            property.pullYoEnumValue();
      }
   }
}

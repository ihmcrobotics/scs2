package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

import javafx.beans.property.Property;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;

public class BufferedJavaFXMessager extends SharedMemoryJavaFXMessager
{
   private final HashMap<Topic<?>, AtomicReference<?>> internalBuffer = new HashMap<>();

   public BufferedJavaFXMessager(MessagerAPI messagerAPI)
   {
      super(messagerAPI);
      for (Topic<?> topic : messagerAPI.getAllTopics())
         internalBuffer.put(topic, super.createInput(topic, null));
   }

   @SuppressWarnings("unchecked")
   public <T> T getLastValue(Topic<T> topic)
   {
      return (T) internalBuffer.get(topic).get();
   }

   @SuppressWarnings("unchecked")
   @Override
   public <T> AtomicReference<T> createInput(Topic<T> topic, T defaultValue)
   {
      if (defaultValue == null)
         return super.createInput(topic, (T) internalBuffer.get(topic).get());
      else
         return super.createInput(topic, defaultValue);
   }

   @SuppressWarnings("unchecked")
   @Override
   public <T> Property<T> createPropertyInput(Topic<T> topic, T initialValue)
   {
      if (initialValue == null)
         return super.createPropertyInput(topic, (T) internalBuffer.get(topic).get());
      else
         return super.createPropertyInput(topic, initialValue);
   }
}

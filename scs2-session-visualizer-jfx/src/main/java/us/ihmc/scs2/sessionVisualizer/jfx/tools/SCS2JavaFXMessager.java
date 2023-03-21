package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.beans.property.Property;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;

public class SCS2JavaFXMessager extends SharedMemoryJavaFXMessager
{
   private final AnimationTimer animationTimer;
   private final HashMap<Topic<?>, AtomicReference<?>> internalBuffer = new HashMap<>();

   public SCS2JavaFXMessager(MessagerAPI messagerAPI)
   {
      super(messagerAPI, true);

      animationTimer = new ObservedAnimationTimer(getClass().getSimpleName())
      {
         @Override
         public void handleImpl(long now)
         {
            updateFXTopicListeners();
         }
      };

      for (Topic<?> topic : messagerAPI.getAllTopics())
         internalBuffer.put(topic, super.createInput(topic, null));
   }

   @Override
   public void startMessager()
   {
      super.startMessager();
      animationTimer.start();
   }

   @Override
   public void closeMessager()
   {
      animationTimer.stop();
      super.closeMessager();
   }

   @Override
   protected void runFXLater(Runnable fxTask)
   {
      JavaFXMissingTools.runLater(getClass(), fxTask);
   }

   @Override
   protected void runFXAndWait(final Runnable fxTask)
   {
      JavaFXMissingTools.runAndWait(getClass(), fxTask);
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

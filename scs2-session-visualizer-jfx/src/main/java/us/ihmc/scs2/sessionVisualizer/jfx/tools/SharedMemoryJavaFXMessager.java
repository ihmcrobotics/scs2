package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;

/**
 * Implementation of {@code JavaFXMessager} using shared memory.
 *
 * @author Sylvain Bertrand
 */
public class SharedMemoryJavaFXMessager extends SharedMemoryMessager implements JavaFXMessager
{
   private final Map<Topic<?>, JavaFXSyncedTopicListeners> javaFXSyncedTopicListeners = new HashMap<>();
   private final AnimationTimer animationTimer;
   private boolean readingListeners = false;

   /**
    * Creates a new messager.
    *
    * @param messagerAPI the API to use with this messager.
    */
   public SharedMemoryJavaFXMessager(MessagerAPI messagerAPI)
   {
      super(messagerAPI);
      animationTimer = new ObservedAnimationTimer(getClass().getSimpleName())
      {
         @Override
         public void handleImpl(long now)
         {
            try
            {
               readingListeners = true;

               javaFXSyncedTopicListeners.entrySet().removeIf(entry -> entry.getValue().isEmpty());

               for (JavaFXSyncedTopicListeners listener : javaFXSyncedTopicListeners.values())
                  listener.notifyListeners();
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
            finally
            {
               readingListeners = false;
            }
         }
      };
   }

   /** {@inheritDoc} */
   @Override
   public <T> void registerJavaFXSyncedTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      JavaFXSyncedTopicListeners topicListeners = javaFXSyncedTopicListeners.get(topic);

      if (topicListeners == null)
      {
         JavaFXSyncedTopicListeners newTopicListeners = new JavaFXSyncedTopicListeners(topic);
         topicListeners = newTopicListeners;

         if (!readingListeners && Platform.isFxApplicationThread())
         { // It appears to not be enough to check for application thread somehow.
            javaFXSyncedTopicListeners.put(topic, newTopicListeners);
         }
         else // The following one can throw an exception if the JavaFX thread has not started yet.
         {
            try
            { // Postpone the entire registration in case JavaFXSyncedTopicListeners has been created by another caller.
               JavaFXMissingTools.runLater(getClass(), () -> registerJavaFXSyncedTopicListener(topic, listener));
            }
            catch (IllegalStateException e)
            { // The JavaFX thread has not started yet, no need to invoke Platform.runLater(...).
               javaFXSyncedTopicListeners.put(topic, newTopicListeners);
            }
         }
      }

      topicListeners.addListener(listener);
   }

   /** {@inheritDoc} */
   @Override
   public <T> boolean removeJavaFXSyncedTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      JavaFXSyncedTopicListeners topicListeners = javaFXSyncedTopicListeners.get(topic);
      if (topicListeners == null)
         return false;

      if (!readingListeners && Platform.isFxApplicationThread())
      {
         return topicListeners.removeListener(listener);
      }
      else
      {
         try
         { // The following one can throw an exception if the JavaFX thread has not started yet.
            MutableBoolean result = new MutableBoolean();
            JavaFXMissingTools.runAndWait(getClass(), () -> result.setValue(topicListeners.removeListener(listener)));
            return result.booleanValue();
         }
         catch (IllegalStateException e)
         { // The JavaFX thread has not started yet, no need to invoke Platform.runLater(...).
            return topicListeners.removeListener(listener);
         }
      }
   }

   /** {@inheritDoc} */
   @Override
   public void startMessager()
   {
      super.startMessager();
      animationTimer.start();
   }

   /** {@inheritDoc} */
   @Override
   public void closeMessager()
   {
      javaFXSyncedTopicListeners.values().forEach(JavaFXSyncedTopicListeners::dispose);
      javaFXSyncedTopicListeners.clear();
      super.closeMessager();
      animationTimer.stop();
   }

   @SuppressWarnings("unchecked")
   private class JavaFXSyncedTopicListeners
   {
      private final ConcurrentLinkedQueue<Object> inputQueue = new ConcurrentLinkedQueue<>();
      private final ConcurrentLinkedQueue<TopicListener<Object>> listeners = new ConcurrentLinkedQueue<>();

      private JavaFXSyncedTopicListeners(Topic<?> topic)
      {
         registerTopicListener(topic, message ->
         {
            if (message != null)
               inputQueue.add(message);
         });
      }

      private void addListener(TopicListener<?> listener)
      {
         listeners.add((TopicListener<Object>) listener);
      }

      private boolean removeListener(TopicListener<?> listener)
      {
         return listeners.remove(listener);
      }

      private void notifyListeners()
      {
         while (!inputQueue.isEmpty())
         {
            Object newData = inputQueue.poll();
            listeners.forEach(listener -> listener.receivedMessageForTopic(newData));
         }
      }

      public boolean isEmpty()
      {
         return listeners.isEmpty();
      }

      public void dispose()
      {
         inputQueue.clear();
         listeners.clear();
      }
   }
}

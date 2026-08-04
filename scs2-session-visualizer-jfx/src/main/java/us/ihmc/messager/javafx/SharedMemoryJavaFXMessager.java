package us.ihmc.messager.javafx;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import us.ihmc.messager.Message;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.SynchronizeHint;
import us.ihmc.messager.TopicListenerBase;

/**
 * Implementation of {@code JavaFXMessager} using shared memory.
 *
 * @author Sylvain Bertrand
 */
public class SharedMemoryJavaFXMessager extends SharedMemoryMessager implements JavaFXMessager
{
   private final Map<Topic<?>, JavaFXTopicListeners> fxTopicListeners = new HashMap<>();
   private final AnimationTimer animationTimer;
   private boolean readingListeners = false;

   /**
    * Creates a new messager.
    *
    * @param messagerAPI the API to use with this messager.
    */
   public SharedMemoryJavaFXMessager(MessagerAPI messagerAPI)
   {
      this(messagerAPI, false);
   }

   /**
    * Creates a new messager.
    *
    * @param messagerAPI the API to use with this messager.
    */
   public SharedMemoryJavaFXMessager(MessagerAPI messagerAPI, boolean managed)
   {
      super(messagerAPI);
      if (managed)
      {
         animationTimer = null;
      }
      else
      {
         animationTimer = new AnimationTimer()
         {
            @Override
            public void handle(long now)
            {
               updateFXTopicListeners();
            }
         };
      }

   }

   public void updateFXTopicListeners()
   {
      try
      {
         readingListeners = true;

         fxTopicListeners.entrySet().removeIf(entry -> entry.getValue().isEmpty());

         for (JavaFXTopicListeners listener : fxTopicListeners.values())
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

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public <T> void submitMessage(Message<T> message)
   {
      super.submitMessage(message);
      JavaFXTopicListeners topicListeners = fxTopicListeners.get(message.getTopic(messagerAPI));
      if (topicListeners != null)
         topicListeners.submitMessage((Message<Object>) message);
   }

   /** {@inheritDoc} */
   @Override
   public <T> void addFXTopicListenerBase(Topic<T> topic, TopicListenerBase<T> listener)
   {
      JavaFXTopicListeners topicListeners = fxTopicListeners.get(topic);

      if (topicListeners != null)
      {
         topicListeners.addListener(listener);
         return;
      }

      if (!readingListeners && Platform.isFxApplicationThread())
      { // It appears to not be enough to check for application thread somehow.
         topicListeners = new JavaFXTopicListeners(topic);
         fxTopicListeners.put(topic, topicListeners);
         topicListeners.addListener(listener);
      }
      else // The following one can throw an exception if the JavaFX thread has not started yet.
      {
         try
         { // Postpone the entire registration in case JavaFXSyncedTopicListeners has been created by another caller.
            runFXLater(() -> addFXTopicListenerBase(topic, listener));
            return;
         }
         catch (IllegalStateException e)
         { // The JavaFX thread has not started yet, no need to invoke Platform.runLater(...).
            topicListeners = new JavaFXTopicListeners(topic);
            fxTopicListeners.put(topic, topicListeners);
            topicListeners.addListener(listener);
         }
      }
   }

   protected void runFXLater(Runnable fxTask)
   {
      Platform.runLater(fxTask);
   }

   protected void runFXAndWait(final Runnable fxTask)
   {
      if (Platform.isFxApplicationThread())
      {
         tryRun(fxTask);
      }
      else
      {
         final CountDownLatch doneLatch = new CountDownLatch(1);

         runFXLater(() ->
         {
            tryRun(fxTask);
            doneLatch.countDown();
         });

         try
         {
            doneLatch.await();
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
         }
      }
   }

   protected void tryRun(Runnable fxTask)
   {
      try
      {
         fxTask.run();
      }
      catch (Throwable t)
      {
         System.err.println("Exception in fxTask");
         t.printStackTrace();
      }
   }

   /** {@inheritDoc} */
   @Override
   public <T> boolean removeFXTopicListener(Topic<T> topic, TopicListenerBase<T> listener)
   {
      JavaFXTopicListeners topicListeners = fxTopicListeners.get(topic);
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
            runFXAndWait(() -> result.setValue(topicListeners.removeListener(listener)));
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
      if (animationTimer != null)
         animationTimer.start();
   }

   /** {@inheritDoc} */
   @Override
   public void closeMessager()
   {
      fxTopicListeners.values().forEach(JavaFXTopicListeners::dispose);
      fxTopicListeners.clear();
      super.closeMessager();
      if (animationTimer != null)
         animationTimer.stop();
   }

   @SuppressWarnings("unchecked")
   protected class JavaFXTopicListeners
   {
      protected final ConcurrentLinkedQueue<Message<Object>> messageQueue = new ConcurrentLinkedQueue<>();
      protected final ConcurrentLinkedQueue<TopicListenerBase<Object>> listeners = new ConcurrentLinkedQueue<>();

      protected JavaFXTopicListeners(Topic<?> topic)
      {
      }

      protected void submitMessage(Message<Object> message)
      {
         if (message.getSynchronizeHint() == SynchronizeHint.SYNCHRONOUS)
            runFXAndWait(() -> listeners.forEach(listener -> listener.receivedMessageForTopic(message)));
         else
            messageQueue.add(message);
      }

      protected void addListener(TopicListenerBase<?> listener)
      {
         listeners.add((TopicListenerBase<Object>) listener);
      }

      protected boolean removeListener(TopicListenerBase<?> listener)
      {
         return listeners.remove(listener);
      }

      protected void notifyListeners()
      {
         while (!messageQueue.isEmpty())
         {
            Message<Object> newMessage = messageQueue.poll();
            listeners.forEach(listener -> listener.receivedMessageForTopic(newMessage));
         }
      }

      protected boolean isEmpty()
      {
         return listeners.isEmpty();
      }

      protected void dispose()
      {
         messageQueue.clear();
         listeners.clear();
      }
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.messager;

import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;

/**
 * A small, in-process, typed pub/sub bus for the JavaFX UI layer, replacing the vendored
 * {@code ihmc-messager}-based stack.
 * <p>
 * Every topic has: a last known value ({@link #getLastValue(Topic)}), plain {@link AtomicReference}
 * inputs ({@link #createInput(Topic)}), and two flavors of listener:
 * </p>
 * <ul>
 * <li>{@link #addTopicListener(Topic, TopicListener)} — invoked synchronously on whichever thread
 * called {@link #submitMessage(Topic, Object)}, with no coalescing.</li>
 * <li>{@link #addFXTopicListener(Topic, TopicListener)} / {@link #addFXTopicListenerBase(Topic,
 * TopicListenerBase)} — delivered on the JavaFX Application Thread, coalesced to at most once per
 * rendering pulse. A {@link SynchronizeHint#SYNCHRONOUS} submit blocks the caller until the FX
 * listeners have run instead of waiting for the next pulse.</li>
 * </ul>
 */
public class SCS2Messager
{
   private final ConcurrentHashMap<Topic<?>, TopicEntry<?>> topicEntries = new ConcurrentHashMap<>();
   private final AtomicBoolean isConnected = new AtomicBoolean(false);
   private final ObservedAnimationTimer animationTimer = new ObservedAnimationTimer(getClass().getSimpleName())
   {
      @Override
      public void handleImpl(long now)
      {
         for (TopicEntry<?> entry : topicEntries.values())
            entry.notifyFXListeners();
      }
   };

   public void startMessager()
   {
      isConnected.set(true);
      animationTimer.start();
   }

   public void closeMessager()
   {
      isConnected.set(false);
      animationTimer.stop();
      topicEntries.values().forEach(TopicEntry::clear);
      topicEntries.clear();
   }

   public boolean isMessagerOpen()
   {
      return isConnected.get();
   }

   public <T> void submitMessage(Topic<T> topic, T messageContent)
   {
      submitMessage(topic, messageContent, SynchronizeHint.NONE);
   }

   public <T> void submitMessage(Topic<T> topic, T messageContent, SynchronizeHint hint)
   {
      if (!isConnected.get())
      {
         LogTools.warn("This messager is closed, message's topic: " + topic.getName());
         return;
      }

      TopicEntry<T> entry = entryFor(topic);
      entry.lastValue.set(messageContent);

      for (AtomicReference<T> input : entry.boundInputs)
         input.set(messageContent);

      Message<T> message = new Message<>(topic, messageContent, hint == null ? SynchronizeHint.NONE : hint);

      for (TopicListenerBase<T> listener : entry.immediateListeners)
         listener.receivedMessageForTopic(message);

      if (message.getSynchronizeHint() == SynchronizeHint.SYNCHRONOUS)
         JavaFXMissingTools.runAndWait(getClass(), () -> entry.fxListeners.forEach(listener -> listener.receivedMessageForTopic(message)));
      else
         entry.pendingFXMessages.add(message);
   }

   public <T> AtomicReference<T> createInput(Topic<T> topic)
   {
      return createInput(topic, null);
   }

   public <T> AtomicReference<T> createInput(Topic<T> topic, T defaultValue)
   {
      TopicEntry<T> entry = entryFor(topic);
      AtomicReference<T> input = new AtomicReference<>(defaultValue != null ? defaultValue : entry.lastValue.get());
      entry.boundInputs.add(input);
      return input;
   }

   public <T> Property<T> createPropertyInput(Topic<T> topic)
   {
      return createPropertyInput(topic, null);
   }

   public <T> Property<T> createPropertyInput(Topic<T> topic, T initialValue)
   {
      TopicEntry<T> entry = entryFor(topic);
      SimpleObjectProperty<T> property = new SimpleObjectProperty<>(this, topic.getName(), initialValue != null ? initialValue : entry.lastValue.get());
      addFXTopicListener(topic, property::setValue);
      return property;
   }

   public <T> void addTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      entryFor(topic).immediateListeners.add(listener);
   }

   public <T> void addTopicListenerBase(Topic<T> topic, TopicListenerBase<T> listener)
   {
      entryFor(topic).immediateListeners.add(listener);
   }

   public <T> void addFXTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      addFXTopicListenerBase(topic, listener);
   }

   public <T> void addFXTopicListenerBase(Topic<T> topic, TopicListenerBase<T> listener)
   {
      entryFor(topic).fxListeners.add(listener);
   }

   public <T> boolean removeFXTopicListener(Topic<T> topic, TopicListenerBase<T> listener)
   {
      TopicEntry<T> entry = entryFor(topic);
      return entry.fxListeners.remove(listener);
   }

   public <T> MessageBidirectionalBinding<T> bindBidirectional(Topic<T> topic, Property<T> property, boolean pushValue)
   {
      MessageBidirectionalBinding<T> binding = new MessageBidirectionalBinding<>(messageContent -> submitMessage(topic, messageContent), property);
      property.addListener(binding);
      addFXTopicListener(topic, binding);
      if (pushValue)
         submitMessage(topic, property.getValue());
      return binding;
   }

   public <T> T getLastValue(Topic<T> topic)
   {
      return entryFor(topic).lastValue.get();
   }

   @SuppressWarnings("unchecked")
   private <T> TopicEntry<T> entryFor(Topic<T> topic)
   {
      return (TopicEntry<T>) topicEntries.computeIfAbsent(topic, t -> new TopicEntry<T>());
   }

   private static class TopicEntry<T>
   {
      private final AtomicReference<T> lastValue = new AtomicReference<>();
      private final Queue<AtomicReference<T>> boundInputs = new ConcurrentLinkedQueue<>();
      private final Queue<TopicListenerBase<T>> immediateListeners = new ConcurrentLinkedQueue<>();
      private final Queue<TopicListenerBase<T>> fxListeners = new ConcurrentLinkedQueue<>();
      private final Queue<Message<T>> pendingFXMessages = new ConcurrentLinkedQueue<>();

      void notifyFXListeners()
      {
         Message<T> message;
         while ((message = pendingFXMessages.poll()) != null)
            for (TopicListenerBase<T> listener : fxListeners)
               listener.receivedMessageForTopic(message);
      }

      void clear()
      {
         boundInputs.clear();
         immediateListeners.clear();
         fxListeners.clear();
         pendingFXMessages.clear();
      }
   }
}

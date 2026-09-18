package us.ihmc.scs2.sessionVisualizer.jfx.messager;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import javafx.beans.property.Property;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

/**
 * Keeps a JavaFX {@link Property} and a messager topic synced in both directions, guarding against
 * an update received from one side re-triggering a submit back to the other.
 *
 * @param <T> the data type shared by the property and the topic.
 */
public class MessageBidirectionalBinding<T> implements TopicListener<T>, ChangeListener<T>
{
   private final AtomicBoolean changedOnMessageReception = new AtomicBoolean(false);
   private final Property<T> boundProperty;
   private final Consumer<T> messagingAction;

   MessageBidirectionalBinding(Consumer<T> messagingAction, Property<T> boundProperty)
   {
      this.messagingAction = messagingAction;
      this.boundProperty = boundProperty;
   }

   @Override
   public void changed(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      if (changedOnMessageReception.getAndSet(false))
         return;
      messagingAction.accept(newValue);
   }

   @Override
   public void receivedMessageForTopic(T messageContent)
   {
      boolean updateProperty = boundProperty.getValue() == null ? messageContent != null : !boundProperty.getValue().equals(messageContent);
      changedOnMessageReception.set(updateProperty);
      if (updateProperty)
         boundProperty.setValue(messageContent);
   }
}

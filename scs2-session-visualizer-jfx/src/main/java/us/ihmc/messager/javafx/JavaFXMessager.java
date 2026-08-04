package us.ihmc.messager.javafx;

import javafx.beans.InvalidationListener;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListenerBase;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.javafx.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

/**
 * This extends a {@code Messager} to include convenience methods for binding {@link Property}s.
 *
 * @author Sylvain Bertrand
 */
public interface JavaFXMessager extends Messager
{
   /**
    * Creates a property which is to be automatically updated when this messager receives data destined
    * to the given topic.
    *
    * @param topic the topic to listen to.
    * @return a property that is updated automatically when receiving new data.
    */
   default <T> Property<T> createPropertyInput(Topic<T> topic)
   {
      return createPropertyInput(topic, null);
   }

   /**
    * Creates a property which is to be automatically updated when this messager receives data destined
    * to the given topic.
    *
    * @param topic        the topic to listen to.
    * @param initialValue the initial value of the newly created property.
    * @return a property that is updated automatically when receiving new data.
    */
   default <T> Property<T> createPropertyInput(Topic<T> topic, T initialValue)
   {
      SimpleObjectProperty<T> input = new SimpleObjectProperty<>(this, topic.getName(), initialValue);
      bindPropertyToTopic(topic, input);
      return input;
   }

   /**
    * Binds a topic to a property such that a message is sent whenever the observable-value changes.
    *
    * @param topicToBind     the topic to bind to the observable value.
    * @param observableValue the value which is to trigger sending message.
    * @return the topic's binding.
    */
   default <T> InvalidationListener bindTopic(Topic<T> topicToBind, ObservableValue<T> observableValue)
   {
      InvalidationListener listener = (observable) -> submitMessage(topicToBind, observableValue.getValue());
      observableValue.addListener(listener);
      return listener;
   }

   /**
    * Binds an existing property to a topic, such that when this messager receives data for the topic,
    * it will update the property.
    *
    * @param topic          the topic which is to trigger updating the property.
    * @param propertyToBind the property to bind to the given topic.
    * @return the property's binding.
    */
   default <T> TopicListener<T> bindPropertyToTopic(Topic<T> topic, Property<T> propertyToBind)
   {
      TopicListener<T> listener = messageContent -> propertyToBind.setValue(messageContent);
      addFXTopicListener(topic, listener);
      return listener;
   }

   /**
    * Creates a bidirectional binding between the given topic and property, i.e. they are synced.
    *
    * @param topic     the topic to bind.
    * @param property  the property to bind.
    * @param pushValue whether the current property value should be should be submitted to the
    *                  messager.
    * @return the new binding between the topic and property.
    */
   default <T> MessageBidirectionalBinding<T, T> bindBidirectional(Topic<T> topic, Property<T> property, boolean pushValue)
   {
      MessageBidirectionalBinding<T, T> bind = MessageBidirectionalBinding.createSingleTypedBinding(messageContent -> submitMessage(topic, messageContent),
                                                                                                    property);
      property.addListener(bind);
      addFXTopicListener(topic, bind);
      if (pushValue)
         submitMessage(topic, property.getValue());
      return bind;
   }

   /**
    * Creates a bidirectional binding between the given topic and property, i.e. they are synced.
    *
    * @param topic     the topic to bind.
    * @param property  the property to bind.
    * @param converter protocol to convert between the topic data type and the property data type.
    * @param pushValue whether the current property value should be should be submitted to the
    *                  messager.
    * @return the new binding between the topic and property.
    */
   default <M, P> MessageBidirectionalBinding<M, P> bindBidirectional(Topic<M> topic,
                                                                      Property<P> property,
                                                                      PropertyToMessageTypeConverter<M, P> converter,
                                                                      boolean pushValue)
   {
      MessageBidirectionalBinding<M, P> bind = new MessageBidirectionalBinding<>(messageContent -> submitMessage(topic, messageContent), property, converter);
      property.addListener(bind);
      addFXTopicListener(topic, bind);
      if (pushValue)
         submitMessage(topic, converter.convert(property.getValue()));
      return bind;
   }

   /**
    * Same as {@link #addTopicListener(Topic, TopicListener)} but the listener only get notified on the
    * next rendering thread tick.
    * <p>
    * This implementation is to be used whenever the listener is to update some UI controls or scene.
    * </p>
    * <p>
    * The {@code TopicListener} interface is a little more user friendly by first unpacking the message
    * content. Note that by doing so, it hides other potential fields associated to the message itself.
    * </p>
    *
    * @param topic    the topic to listen to.
    * @param listener the listener to be registered.
    */
   default <T> void addFXTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      addFXTopicListenerBase(topic, listener);
   }

   /**
    * Same as {@link #addTopicListenerBase(Topic, TopicListenerBase)} but the listener only get
    * notified on the next rendering thread tick.
    * <p>
    * This implementation is to be used whenever the listener is to update some UI controls or scene.
    * </p>
    *
    * @param topic    the topic to listen to.
    * @param listener the listener to be registered.
    */
   <T> void addFXTopicListenerBase(Topic<T> topic, TopicListenerBase<T> listener);

   /**
    * Removes a listener that was previously registered to this messager.
    *
    * @param topic    the topic the listener is listening to.
    * @param listener the listener to be removed.
    * @return {@code true} if the internal list of inputs was modified by this operation, {@code false}
    *         otherwise.
    */
   <T> boolean removeFXTopicListener(Topic<T> topic, TopicListenerBase<T> listener);
}

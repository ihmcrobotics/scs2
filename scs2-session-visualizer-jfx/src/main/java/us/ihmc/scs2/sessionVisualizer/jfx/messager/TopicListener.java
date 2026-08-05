package us.ihmc.scs2.sessionVisualizer.jfx.messager;

/**
 * Listener notified with just the message content when {@link SCS2Messager} receives data for a
 * given topic.
 *
 * @param <T> the data type.
 */
public interface TopicListener<T> extends TopicListenerBase<T>
{
   @Override
   default void receivedMessageForTopic(Message<T> message)
   {
      receivedMessageForTopic(message.getMessageContent());
   }

   void receivedMessageForTopic(T messageContent);
}

package us.ihmc.scs2.sessionVisualizer.jfx.messager;

/**
 * Listener notified with the full {@link Message} (including its {@link SynchronizeHint}) when
 * {@link SCS2Messager} receives data for a given topic.
 *
 * @param <T> the data type.
 */
public interface TopicListenerBase<T>
{
   void receivedMessageForTopic(Message<T> message);
}

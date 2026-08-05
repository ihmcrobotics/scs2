package us.ihmc.scs2.sessionVisualizer.jfx.messager;

/**
 * A single message sent through {@link SCS2Messager}, pairing a topic's payload with the
 * {@link SynchronizeHint} it was submitted with.
 *
 * @param <T> the type of data this message carries.
 */
public final class Message<T>
{
   private final Topic<T> topic;
   private final T messageContent;
   private final SynchronizeHint synchronizeHint;

   Message(Topic<T> topic, T messageContent, SynchronizeHint synchronizeHint)
   {
      this.topic = topic;
      this.messageContent = messageContent;
      this.synchronizeHint = synchronizeHint;
   }

   public Topic<T> getTopic()
   {
      return topic;
   }

   public T getMessageContent()
   {
      return messageContent;
   }

   public SynchronizeHint getSynchronizeHint()
   {
      return synchronizeHint;
   }
}

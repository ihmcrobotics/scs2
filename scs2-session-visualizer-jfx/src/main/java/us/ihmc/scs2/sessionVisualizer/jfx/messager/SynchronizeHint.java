package us.ihmc.scs2.sessionVisualizer.jfx.messager;

/**
 * Hint on how a message should be delivered to FX-thread listeners, provided on a best-effort
 * basis when submitting a message via {@link SCS2Messager#submitMessage(Topic, Object, SynchronizeHint)}.
 */
public enum SynchronizeHint
{
   /** The message is queued and delivered on the next JavaFX pulse. */
   NONE,
   /** The submitting thread blocks until the FX-thread listeners have processed the message. */
   SYNCHRONOUS
}

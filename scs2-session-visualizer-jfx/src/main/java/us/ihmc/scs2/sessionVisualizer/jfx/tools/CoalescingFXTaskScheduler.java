package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

/**
 * Coalesces repeated {@link #request()} calls into at most one pending task at a time.
 * <p>
 * Useful when a high-frequency source (e.g. a {@code Session} publishing buffer properties up to 100x/sec, even when
 * idle) needs to schedule work on the JavaFX Application Thread. Naively calling {@code Platform.runLater(...)} once
 * per source event queues one Runnable per event - if the source publishes much faster than the UI actually needs to
 * refresh, that flood of queued tasks competes with rendering on the FX thread and can measurably drop the frame
 * rate. This class ensures only one task is ever queued at a time: further requests while one is pending are no-ops,
 * and the next request after the pending task runs schedules a new one.
 * </p>
 */
public class CoalescingFXTaskScheduler
{
   private final AtomicBoolean pending = new AtomicBoolean(false);
   private final Runnable task;
   private final Consumer<Runnable> runLater;

   /**
    * @param task     the work to perform on the FX thread. Should read whatever live state it needs at execution
    *                 time rather than closing over it at request-time, since execution may be delayed.
    * @param runLater schedules a {@link Runnable} to run on the FX thread, e.g. {@code Platform::runLater}.
    */
   public CoalescingFXTaskScheduler(Runnable task, Consumer<Runnable> runLater)
   {
      this.task = task;
      this.runLater = runLater;
   }

   /**
    * Requests that the task run on the FX thread. If a request is already pending, this is a no-op - the already
    * queued task will pick up the latest state when it runs.
    */
   public void request()
   {
      if (pending.compareAndSet(false, true))
      {
         runLater.accept(() ->
         {
            pending.set(false);
            task.run();
         });
      }
   }
}

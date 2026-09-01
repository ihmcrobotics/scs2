package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.LongSupplier;

/**
 * Coalesces repeated {@link #request()} calls into at most one pending task at a time, optionally also capping how
 * often the task actually runs.
 * <p>
 * Useful when a high-frequency source (e.g. a {@code Session} publishing buffer properties up to 100x/sec, even when
 * idle) needs to schedule work on the JavaFX Application Thread. Naively calling {@code Platform.runLater(...)} once
 * per source event queues one Runnable per event - if the source publishes much faster than the UI actually needs to
 * refresh, that flood of queued tasks competes with rendering on the FX thread and can measurably drop the frame
 * rate. This class ensures only one task is ever queued at a time: further requests while one is pending are no-ops,
 * and the next request after the pending task runs schedules a new one.
 * </p>
 * <p>
 * Coalescing alone still lets the task run on every single pulse while requests keep arriving faster than the FX
 * thread can service them (e.g. while a log is actively playing back) - each run is cheap in isolation, but the
 * layout/CSS work it triggers (e.g. repositioning a slider thumb) adds up and can be enough on its own to push a
 * busy window's per-pulse cost over budget. Passing a {@code minIntervalNanos} rate-limits how often the task is
 * actually allowed to run, independently of how often {@link #request()} is called; a request that arrives before
 * the interval has elapsed is dropped rather than queued; since a high-frequency source keeps calling
 * {@link #request()}, the next one to arrive after the interval elapses will go through, so no persistent update is
 * ever lost - it's merely delayed by at most {@code minIntervalNanos}.
 * </p>
 */
public class CoalescingFXTaskScheduler
{
   private final AtomicBoolean pending = new AtomicBoolean(false);
   private final Runnable task;
   private final Consumer<Runnable> runLater;
   private final long minIntervalNanos;
   private final LongSupplier nanoTimeSource;
   private volatile boolean hasDispatched = false;
   private volatile long lastDispatchNanos;

   /**
    * @param task     the work to perform on the FX thread. Should read whatever live state it needs at execution
    *                 time rather than closing over it at request-time, since execution may be delayed.
    * @param runLater schedules a {@link Runnable} to run on the FX thread, e.g. {@code Platform::runLater}.
    */
   public CoalescingFXTaskScheduler(Runnable task, Consumer<Runnable> runLater)
   {
      this(task, runLater, 0L);
   }

   /**
    * @param task             the work to perform on the FX thread. Should read whatever live state it needs at
    *                         execution time rather than closing over it at request-time, since execution may be
    *                         delayed.
    * @param runLater         schedules a {@link Runnable} to run on the FX thread, e.g. {@code Platform::runLater}.
    * @param minIntervalNanos the minimum time that must elapse between two runs of {@code task}; a request arriving
    *                         sooner than that is dropped instead of queued. 0 (or negative) disables rate-limiting,
    *                         leaving only the coalescing behavior.
    */
   public CoalescingFXTaskScheduler(Runnable task, Consumer<Runnable> runLater, long minIntervalNanos)
   {
      this(task, runLater, minIntervalNanos, System::nanoTime);
   }

   /**
    * Package-visible overload for tests that need to control the passage of time deterministically.
    */
   CoalescingFXTaskScheduler(Runnable task, Consumer<Runnable> runLater, long minIntervalNanos, LongSupplier nanoTimeSource)
   {
      this.task = task;
      this.runLater = runLater;
      this.minIntervalNanos = minIntervalNanos;
      this.nanoTimeSource = nanoTimeSource;
   }

   /**
    * Requests that the task run on the FX thread. If a request is already pending, this is a no-op - the already
    * queued task will pick up the latest state when it runs. If rate-limiting is enabled and the minimum interval
    * hasn't elapsed since the task last ran, this is also a no-op.
    */
   public void request()
   {
      if (minIntervalNanos > 0 && hasDispatched && nanoTimeSource.getAsLong() - lastDispatchNanos < minIntervalNanos)
         return;

      if (pending.compareAndSet(false, true))
      {
         runLater.accept(() ->
         {
            pending.set(false);
            lastDispatchNanos = nanoTimeSource.getAsLong();
            hasDispatched = true;
            task.run();
         });
      }
   }
}

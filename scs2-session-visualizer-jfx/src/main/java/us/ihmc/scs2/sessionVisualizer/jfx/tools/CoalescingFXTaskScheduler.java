package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.LongSupplier;

/**
 * Coalesces repeated {@link #request()} calls into at most one pending FX-thread task at a time, so a high-frequency
 * source doesn't flood {@code Platform.runLater(...)} and compete with rendering. An optional
 * {@code minIntervalNanos} additionally rate-limits how often the task is allowed to run, dropping requests that
 * arrive before the interval elapses without ever losing an update permanently.
 */
public class CoalescingFXTaskScheduler
{
   private final AtomicBoolean pending = new AtomicBoolean(false);
   private final Runnable task;
   private final Consumer<Runnable> runLater;
   private final long intervalNanos;
   private final LongSupplier nanoTimeSource;
   private volatile boolean hasDispatched = false;
   private volatile long lastDispatchNanos;

   /**
    * @param task     work to perform on the FX thread; should read live state at execution time, not request time.
    * @param runLater schedules a {@link Runnable} on the FX thread, e.g. {@code Platform::runLater}.
    */
   public CoalescingFXTaskScheduler(Runnable task, Consumer<Runnable> runLater)
   {
      this(task, runLater, 0L);
   }

   /**
    * @param task             work to perform on the FX thread; should read live state at execution time, not request time.
    * @param runLater         schedules a {@link Runnable} on the FX thread, e.g. {@code Platform::runLater}.
    * @param intervalNanos minimum time between two runs of {@code task}; earlier requests are dropped instead of
    *                         queued. 0 (or negative) disables rate-limiting.
    */
   public CoalescingFXTaskScheduler(Runnable task, Consumer<Runnable> runLater, long intervalNanos)
   {
      this(task, runLater, intervalNanos, System::nanoTime);
   }

   /**
    * Package-visible overload for tests that need to control the passage of time deterministically.
    */
   CoalescingFXTaskScheduler(Runnable task, Consumer<Runnable> runLater, long intervalNanos, LongSupplier nanoTimeSource)
   {
      this.task = task;
      this.runLater = runLater;
      this.intervalNanos = intervalNanos;
      this.nanoTimeSource = nanoTimeSource;
   }

   /**
    * Requests that the task run on the FX thread; a no-op if a request is already pending or the minimum interval
    * hasn't elapsed since the task last ran.
    */
   public void request()
   {
      if (intervalNanos > 0 && hasDispatched && nanoTimeSource.getAsLong() - lastDispatchNanos < intervalNanos)
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

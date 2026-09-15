package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import javafx.application.Platform;

/**
 * Coalesces updates coming from a potentially high-frequency, off-FX-thread source (e.g. a
 * {@code Session} thread ticking faster than the JavaFX render loop) into at most one pending
 * {@link Platform#runLater(Runnable)} task at a time, applying only the latest value.
 * <p>
 * Without this, a listener that schedules a new {@code runLater} on every update can flood the
 * JavaFX application thread when the source ticks much faster than the ~60Hz render pulse,
 * causing the UI to lag or its frame rate to drop.
 * </p>
 */
public class FXCoalescedUpdater<T>
{
   private final AtomicReference<T> pendingValue = new AtomicReference<>();
   private final AtomicBoolean updateScheduled = new AtomicBoolean(false);
   private final Consumer<T> fxUpdate;

   public FXCoalescedUpdater(Consumer<T> fxUpdate)
   {
      this.fxUpdate = fxUpdate;
   }

   public void update(T value)
   {
      pendingValue.set(value);

      if (updateScheduled.compareAndSet(false, true))
      {
         Platform.runLater(() ->
         {
            T latest = pendingValue.get();
            updateScheduled.set(false);
            fxUpdate.accept(latest);
         });
      }
   }
}

package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

/**
 * Regression test for the FPS drop reported when the Log Session Manager window is open: before the fix, the log position
 * slider was synced via a raw {@code Session.addCurrentBufferPropertiesListener} callback that queued a fresh
 * {@code Platform.runLater} task on every single publish - which, at the session's ~100Hz pause-tick publish rate, floods
 * the FX thread with far more queued work than the UI can usefully consume in a frame, and competes with rendering.
 * <p>
 * {@link CoalescingFXTaskScheduler} fixes this by ensuring at most one task is queued at a time. This test simulates a
 * burst of rapid-fire {@code request()} calls (standing in for the session's publish callback) arriving before the FX
 * thread (standing in for {@code Platform.runLater}) gets a chance to run anything, and verifies only one task is queued.
 * </p>
 */
public class CoalescingFXTaskSchedulerTest
{
   @Test
   public void testBurstOfRequestsQueuesOnlyOneTask()
   {
      AtomicInteger taskRunCount = new AtomicInteger();
      List<Runnable> queuedFXTasks = new ArrayList<>();
      CoalescingFXTaskScheduler scheduler = new CoalescingFXTaskScheduler(taskRunCount::incrementAndGet, queuedFXTasks::add);

      // Simulate the session publishing buffer properties 100 times (its ~100Hz idle publish rate) before the FX
      // thread gets a chance to process anything - e.g. a slow pulse, or simply many publishes between two pulses.
      for (int i = 0; i < 100; i++)
         scheduler.request();

      assertEquals(1, queuedFXTasks.size(), "100 requests before the FX thread runs anything should queue exactly one task, not one per request");
      assertEquals(0, taskRunCount.get(), "The task should not run until the FX thread actually executes the queued Runnable");
   }

   @Test
   public void testTaskRunsOnceThenCanBeRequestedAgain()
   {
      AtomicInteger taskRunCount = new AtomicInteger();
      List<Runnable> queuedFXTasks = new ArrayList<>();
      CoalescingFXTaskScheduler scheduler = new CoalescingFXTaskScheduler(taskRunCount::incrementAndGet, queuedFXTasks::add);

      for (int i = 0; i < 10; i++)
         scheduler.request();
      assertEquals(1, queuedFXTasks.size());

      queuedFXTasks.remove(0).run(); // Simulate the FX thread draining its queue.
      assertEquals(1, taskRunCount.get());

      // Once the pending task has run, a new burst should queue exactly one more task, not be permanently suppressed.
      for (int i = 0; i < 10; i++)
         scheduler.request();
      assertEquals(1, queuedFXTasks.size(), "A fresh burst after the pending task ran should queue a new task");

      queuedFXTasks.remove(0).run();
      assertEquals(2, taskRunCount.get());
   }
}

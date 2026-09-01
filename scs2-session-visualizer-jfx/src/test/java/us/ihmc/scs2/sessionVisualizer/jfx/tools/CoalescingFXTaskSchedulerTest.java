package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import org.junit.jupiter.api.Test;

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

   @Test
   public void testMinIntervalDropsRequestsWithinTheCooldownWindow()
   {
      AtomicInteger taskRunCount = new AtomicInteger();
      List<Runnable> queuedFXTasks = new ArrayList<>();
      AtomicLong fakeNanoTime = new AtomicLong(0);
      long minIntervalNanos = 33_000_000L; // ~30Hz, matching the log position slider's cap.
      CoalescingFXTaskScheduler scheduler = new CoalescingFXTaskScheduler(taskRunCount::incrementAndGet,
                                                                          queuedFXTasks::add,
                                                                          minIntervalNanos,
                                                                          fakeNanoTime::get);

      // First request always goes through - there's nothing to rate-limit against yet.
      scheduler.request();
      assertEquals(1, queuedFXTasks.size());
      queuedFXTasks.remove(0).run();
      assertEquals(1, taskRunCount.get()); // dispatched at t=0

      // Simulate a log playing back: a fresh request arrives on (almost) every pulse (~16ms at 60Hz). The next two
      // (t=16ms, t=32ms) fall inside the 33ms cooldown since the last dispatch (t=0) and should be dropped.
      fakeNanoTime.set(16_000_000L);
      scheduler.request();
      assertEquals(0, queuedFXTasks.size(), "A request 16ms after the last dispatch is within the 33ms cooldown and should be dropped");

      fakeNanoTime.set(32_000_000L);
      scheduler.request();
      assertEquals(0, queuedFXTasks.size(), "A request 32ms after the last dispatch is within the 33ms cooldown and should be dropped");
      assertEquals(1, taskRunCount.get(), "No update should have been lost - the task just hasn't been asked to run again yet");

      // t=48ms: the cooldown since the last dispatch (t=0) has now elapsed, so this request should go through -
      // demonstrating that a steady stream of requests still gets serviced at roughly the capped rate (~30Hz here),
      // not starved forever.
      fakeNanoTime.set(48_000_000L);
      scheduler.request();
      assertEquals(1, queuedFXTasks.size(), "A request after the cooldown elapses should be queued");
      queuedFXTasks.remove(0).run();
      assertEquals(2, taskRunCount.get()); // dispatched at t=48ms

      // Immediately after that new dispatch, a request 16ms later (t=64ms) is inside the cooldown again and should
      // be dropped, same as before.
      fakeNanoTime.set(64_000_000L);
      scheduler.request();
      assertEquals(0, queuedFXTasks.size(), "A request 16ms after the latest dispatch is within the cooldown and should be dropped");
      assertEquals(2, taskRunCount.get());
   }
}

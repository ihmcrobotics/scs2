package us.ihmc.scs2.session;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.function.Consumer;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.sharedMemory.LinkedYoDouble;
import us.ihmc.scs2.sharedMemory.YoDoubleBuffer;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Verifies the final shape of the fix for a high-CPU-usage bug found while investigating log playback looping.
 * <p>
 * The expensive part is {@code sharedBuffer.readBuffer()} in {@link Session#finalizePlaybackTick()}: it reads
 * every {@link YoVariable} in the whole registry (tens of thousands for a real robot log) into its live value,
 * but that only matters right before the data is actually handed to a consumer (chart, live 3D pose, watch
 * panel, ...) via the throttled {@code sharedBuffer.prepareLinkedBuffersForPull()} (period set by
 * {@link Session#setDesiredBufferPublishPeriod(long)}, default 30Hz). So {@code readBuffer()} is called only
 * inside that same throttle, not on every tick.
 * <p>
 * An earlier version of this fix instead slowed the whole playback tick (via
 * {@link Session#computePlaybackTaskPeriod()}) down to the publish rate. That did cut CPU usage, but it also
 * slowed {@code incrementBufferIndex()}/{@code publishBufferProperties()} down to 30Hz, which feed UI elements
 * like the log scrub bar position ({@code LogSessionManagerController.logPositionUpdateListener}) that used to
 * update at ~500Hz-1kHz - visibly laggy scrubbing/playback. The current fix keeps the tick itself fast
 * (untouched {@link Session#computePlaybackTaskPeriod()}, same as before any of this work) and throttles only
 * the expensive read, so the buffer index and UI-facing buffer properties stay smooth while {@code readBuffer()}
 * still only runs ~30 times/sec.
 * <p>
 * This test calls {@link Session#playbackTick()} directly, in a tight loop from the test thread itself, instead
 * of going through {@link Session#startSessionThread()}'s real background {@code PeriodicTaskWrapper}. That's
 * deliberate: {@code Session} explicitly supports driving ticks manually without its internal thread (see
 * {@code scheduleSessionTask}'s "allow running simulation without using the internal session thread" branch), and
 * doing so here removes OS thread-scheduling jitter as a source of flakiness - an earlier version of this test
 * drove ticks through the real background thread and polled with {@code Thread.sleep(1)}, which occasionally
 * (e.g. under JIT warm-up or system load) let real ticks fall far behind their nominal ~2ms period; when that
 * happens every tick's elapsed wall-clock time since the last publish already exceeds the ~33ms publish period on
 * its own, so the throttle in {@link Session#finalizePlaybackTick()} looks satisfied on every tick even though the
 * fix's logic is correct - a false failure unrelated to the code being tested. Driving ticks synchronously as fast
 * as possible from a single thread avoids that: the throttle is checked against real {@code System.nanoTime()}
 * either way, but there's no separate thread whose scheduling can fall behind.
 * <p>
 * Two things are checked, using only public API (no reflection, no production instrumentation):
 * <ol>
 * <li>Ticks (loop iterations) comfortably outnumber publishes ({@link LinkedYoDouble#pull()} reporting fresh
 * data) - proving the buffer index/UI-facing properties are NOT throttled down to the publish rate.
 * <li><b>The actual regression guard</b>: {@code sharedBuffer.readBuffer()} is the only thing that copies buffer
 * data into the backing {@link YoVariable}'s live value, so polling {@code variable.getValue()} directly
 * (bypassing the pull/publish path entirely) and counting how often it changes is a direct proxy for how often
 * {@code readBuffer()} actually runs. The buffer is filled with distinct random values at every index specifically
 * so each {@code readBuffer()} call is visible as a value change. This must stay close to the publish rate, not
 * the tick rate - if {@code readBuffer()} regresses to running on every tick (as it did before the fix), this
 * count jumps from ~30/s to being on the same order as the tick count and the test fails. (An earlier version of
 * this test only checked point 1 above, which is unaffected by where {@code readBuffer()} is called - it would
 * have passed identically whether the fix was applied or not.)
 * </ol>
 */
public class SessionTest
{
   private static final double SESSION_DT_SECONDS = 0.001; // matches the real robot log's dt
   private static final long TEST_DURATION_NANOS = Conversions.secondsToNanoseconds(1.0);

   private static class MinimalSession extends Session
   {
      @Override
      protected double doSpecificRunTick()
      {
         return 0;
      }

      @Override
      public void addGraphicsAddedCallback(Consumer<List<YoGraphicDefinition>> addedGraphicsConsumer)
      {
      }

      @Override
      public String getSessionName()
      {
         return "SessionTest";
      }

      @Override
      public List<RobotDefinition> getRobotDefinitions()
      {
         return Collections.emptyList();
      }

      @Override
      public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
      {
         return Collections.emptyList();
      }
   }

   @Test
   public void testPlaybackTicksStayFastWhilePublishesStayThrottled()
   {
      MinimalSession session = new MinimalSession();
      YoDouble variable = new YoDouble("var", session.getRootRegistry());

      int bufferSize = session.getBufferProperties().getSize();
      YoDoubleBuffer variableBuffer = (YoDoubleBuffer) session.getBuffer().getRegistryBuffer().findYoVariableBuffer(variable);

      Random random = new Random(3453);
      double[] values = variableBuffer.getBuffer();
      for (int i = 0; i < values.length; i++)
         values[i] = random.nextDouble();

      session.getBuffer().setInPoint(0);
      session.getBuffer().setOutPoint(bufferSize - 1);
      session.getBuffer().setCurrentIndex(0);

      session.setSessionDTSeconds(SESSION_DT_SECONDS);
      session.setDesiredBufferPublishPeriod(Conversions.secondsToNanoseconds(1.0 / 30.0));

      // A non-null "user" must be registered, or LinkedBufferArray.prepareForPull() treats this link as inactive
      // (LinkedYoVariable.isActive() == !users.isEmpty()) and prunes it before ever calling prepareForPull() on it -
      // exactly like a real consumer (a chart, a watch-panel control) registers itself as the owning user.
      LinkedYoDouble linkedVariable = session.getBuffer().newLinkedYoVariable(variable, this);

      // No startSessionThread(): drive ticks directly from this thread, see the class javadoc for why.
      session.setSessionMode(SessionMode.PLAYBACK);

      try
      {
         int ticks = 0;
         int successfulPulls = 0;
         int readBufferValueChanges = 0;
         double lastSeenValue = variable.getValue();
         long deadlineNanos = System.nanoTime() + TEST_DURATION_NANOS;

         while (System.nanoTime() < deadlineNanos)
         {
            ThreadTools.sleep(1);
            session.playbackTick();
            ticks++;

            if (linkedVariable.pull())
               successfulPulls++;

            // Bypasses the pull/publish path entirely: this is the raw backend value, only ever changed by
            // sharedBuffer.readBuffer(). Counting distinct values seen is a direct proxy for how often
            // readBuffer() actually ran, independent of the tick loop or the publish throttle.
            double currentValue = variable.getValue();
            if (currentValue != lastSeenValue)
            {
               readBufferValueChanges++;
               lastSeenValue = currentValue;
            }
         }

         // Compute both checks before printing anything, so the printout always reflects this run's actual
         // outcome instead of a fixed block of numbers that looks identical whether the test passes or fails.
         boolean ticksOutpacePublishes = ticks > successfulPulls * 3;
         boolean readBufferThrottled = readBufferValueChanges < successfulPulls * 3 + 10;

         System.out.println("=== Playback tick vs. publish rate ===");
         System.out.println("Playback ticks (driven directly, no background thread)     : " + ticks);
         System.out.println("Successful pulls (throttled, expensive readBuffer()+publish): " + successfulPulls);
         System.out.println("Backing YoVariable value changes (readBuffer() call proxy)  : " + readBufferValueChanges);
         System.out.printf("Ratio (ticks / successful pulls)                            : %.1fx%n", ticks / (double) Math.max(1, successfulPulls));
         System.out.printf("[%-4s] ticks (%d) > successfulPulls*3 (%d)                -> ticking stays fast, decoupled from the publish throttle%n",
                            ticksOutpacePublishes ? "PASS" : "FAIL", ticks, successfulPulls * 3);
         System.out.printf("[%-4s] value changes (%d) close to successfulPulls (%d)   -> readBuffer() stays throttled to the publish rate, not the tick rate%n",
                            readBufferThrottled ? "PASS" : "FAIL", readBufferValueChanges, successfulPulls);

         assertTrue(ticksOutpacePublishes,
                    "Expected playback ticks (" + ticks + ") to far exceed successful publishes (" + successfulPulls
                    + ") - ticking should stay fast even though readBuffer() is throttled");

         // The actual regression guard: readBuffer() must track the publish rate, not the (here, essentially
         // unbounded) tick rate. Bound is relative to successfulPulls rather than a fixed number, since both are
         // driven by the same real-time desiredBufferPublishPeriod throttle and should track each other 1:1.
         assertTrue(readBufferThrottled,
                    "Expected the backing YoVariable's value-change count (" + readBufferValueChanges
                    + ") to track successful publishes (" + successfulPulls
                    + "), not the tick count (" + ticks + ") - readBuffer() appears to be running on every tick again");
      }
      finally
      {
         session.setSessionMode(SessionMode.PAUSE);
         session.shutdownSession();
      }
   }
}

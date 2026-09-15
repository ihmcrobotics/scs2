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
 * Verifies that {@link Session#playbackTick()} ticks (and publishes, via {@link LinkedYoDouble#pull()}) far more
 * often than {@code sharedBuffer.readBuffer()} actually runs, by polling a {@link YoVariable}'s value directly and
 * counting how often it changes.
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

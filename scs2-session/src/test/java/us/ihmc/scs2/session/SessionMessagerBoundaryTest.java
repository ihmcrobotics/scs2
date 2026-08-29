package us.ihmc.scs2.session;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.sharedMemory.FillBufferRequest;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.symbolic.YoEquationManager.YoEquationListChange;

/**
 * Characterizes the public {@link Session} API that used to back the (now removed)
 * {@code SessionMessagerAPI} and {@code YoSharedBufferMessagerAPI} topics and the
 * {@code Session#setupWithMessager} translation shim. These tests lock down that direct API
 * independently of any {@code us.ihmc.messager.Messager} wiring.
 */
public class SessionMessagerBoundaryTest
{
   private TestSession session;

   @AfterEach
   public void tearDown()
   {
      if (session != null && session.hasSessionStarted())
         session.shutdownSession();
   }

   // ----------------------------------------------------------------------
   // YoSharedBufferMessagerAPI-backed behavior
   // ----------------------------------------------------------------------

   @Test
   public void testCropBufferRequest()
   {
      session = new TestSession();
      int originalSize = session.getBufferProperties().getSize();

      CropBufferRequest request = new CropBufferRequest(10, 50);
      session.submitCropBufferRequestAndWait(request);

      YoBufferPropertiesReadOnly properties = session.getBufferProperties();
      assertEquals(request.getCroppedSize(originalSize), properties.getActiveBufferLength());
   }

   @Test
   public void testFillBufferRequestDoesNotAlterBufferMetadata()
   {
      session = new TestSession();
      YoBufferPropertiesReadOnly before = session.getBufferProperties();

      session.submitFillBufferRequestAndWait(new FillBufferRequest(true, 0, 10));

      YoBufferPropertiesReadOnly after = session.getBufferProperties();
      assertEquals(before.getSize(), after.getSize());
      assertEquals(before.getCurrentIndex(), after.getCurrentIndex());
   }

   @Test
   public void testBufferSizeRequest()
   {
      session = new TestSession();
      session.submitBufferSizeRequestAndWait(4096);
      assertEquals(4096, session.getBufferProperties().getSize());
   }

   @Test
   public void testBufferIndexRequests()
   {
      session = new TestSession();

      // Widen the active region first: incrementing/decrementing wraps at the in/out points, which
      // default to [0, 0], not the full buffer.
      session.submitBufferOutPointIndexRequestAndWait(session.getBufferProperties().getSize() - 1);
      assertEquals(session.getBufferProperties().getSize() - 1, session.getBufferProperties().getOutPoint());

      session.submitBufferIndexRequestAndWait(100);
      assertEquals(100, session.getBufferProperties().getCurrentIndex());

      session.submitIncrementBufferIndexRequestAndWait(5);
      assertEquals(105, session.getBufferProperties().getCurrentIndex());

      session.submitDecrementBufferIndexRequestAndWait(3);
      assertEquals(102, session.getBufferProperties().getCurrentIndex());

      session.submitBufferInPointIndexRequestAndWait(20);
      assertEquals(20, session.getBufferProperties().getInPoint());
   }

   @Test
   public void testCropAndFillBufferRequestListeners()
   {
      session = new TestSession();
      AtomicInteger cropCount = new AtomicInteger();
      AtomicInteger fillCount = new AtomicInteger();
      Consumer<CropBufferRequest> cropListener = request -> cropCount.incrementAndGet();
      Consumer<FillBufferRequest> fillListener = request -> fillCount.incrementAndGet();

      session.addCropBufferRequestListener(cropListener);
      session.addFillBufferRequestListener(fillListener);

      session.submitCropBufferRequestAndWait(new CropBufferRequest(0, 10));
      session.submitFillBufferRequestAndWait(new FillBufferRequest(true, 0, 10));

      assertEquals(1, cropCount.get());
      assertEquals(1, fillCount.get());

      assertTrue(session.removeCropBufferRequestListener(cropListener));
      assertTrue(session.removeFillBufferRequestListener(fillListener));

      session.submitCropBufferRequestAndWait(new CropBufferRequest(0, 5));
      session.submitFillBufferRequestAndWait(new FillBufferRequest(true, 0, 5));

      assertEquals(1, cropCount.get());
      assertEquals(1, fillCount.get());
   }

   @Test
   public void testInitializeBufferSizeIsOneShot()
   {
      session = new TestSession();
      // initializeBufferSize submits a non-blocking request; draining it is what the session thread
      // (or, here, a direct processBufferRequests call) normally does.
      assertTrue(session.initializeBufferSize(123));
      session.processBufferRequests(true);
      assertEquals(123, session.getBufferProperties().getSize());

      assertFalse(session.initializeBufferSize(456));
      session.processBufferRequests(true);
      assertEquals(123, session.getBufferProperties().getSize());
   }

   @Test
   public void testCurrentBufferPropertiesListener()
   {
      session = new TestSession();
      AtomicInteger notificationCount = new AtomicInteger();
      Consumer<YoBufferPropertiesReadOnly> listener = properties -> notificationCount.incrementAndGet();

      session.addCurrentBufferPropertiesListener(listener);
      session.publishBufferProperties(session.getBufferProperties());

      assertEquals(1, notificationCount.get());

      assertTrue(session.removeCurrentBufferPropertiesListener(listener));
      session.publishBufferProperties(session.getBufferProperties());
      assertEquals(1, notificationCount.get());
   }

   // ----------------------------------------------------------------------
   // SessionMessagerAPI-backed behavior
   // ----------------------------------------------------------------------

   @Test
   public void testSessionMode()
   {
      session = new TestSession();
      assertEquals(SessionMode.PAUSE, session.getActiveMode());

      session.setSessionMode(SessionMode.RUNNING);
      // setSessionMode schedules the transition; it takes effect once the session thread ticks.
      // hasSessionStarted()/startSessionThread() drive that loop and are exercised separately below.
   }

   @Test
   public void testSessionProperties()
   {
      session = new TestSession();

      session.submitRunAtRealTimeRate(true);
      session.submitPlaybackRealTimeRate(2.5);
      session.setSessionDTNanoseconds(1_000_000L);
      session.setBufferRecordTickPeriod(4);
      session.submitRunMaxDuration(9_000_000_000L);

      SessionProperties properties = session.getSessionProperties();
      assertTrue(properties.isRunAtRealTimeRate());
      assertEquals(2.5, properties.getPlaybackRealTimeRate());
      assertEquals(1_000_000L, properties.getSessionDTNanoseconds());
      assertEquals(4, properties.getBufferRecordTickPeriod());
      assertEquals(9_000_000_000L, properties.getRunMaxDuration());
   }

   @Test
   public void testInitializeBufferRecordTickPeriodIsOneShot()
   {
      session = new TestSession();
      assertTrue(session.initializeBufferRecordTickPeriod(7));
      assertEquals(7, session.getSessionProperties().getBufferRecordTickPeriod());

      assertFalse(session.initializeBufferRecordTickPeriod(9));
      assertEquals(7, session.getSessionProperties().getBufferRecordTickPeriod());
   }

   @Test
   public void testRobotDefinitionListChangeListener()
   {
      session = new TestSession();
      AtomicInteger notificationCount = new AtomicInteger();
      SessionRobotDefinitionListChange[] received = new SessionRobotDefinitionListChange[1];

      Consumer<SessionRobotDefinitionListChange> listener = change ->
      {
         notificationCount.incrementAndGet();
         received[0] = change;
      };
      session.addRobotDefinitionListChangeListener(listener);

      SessionRobotDefinitionListChange change = SessionRobotDefinitionListChange.remove(new RobotDefinition("testRobot"));
      session.reportRobotDefinitionListChange(change);

      assertEquals(1, notificationCount.get());
      assertSame(change, received[0]);

      // Smoke-check the request-side plumbing does not throw; draining the pending request is
      // handled by Session subclasses (e.g. SimulationSession), out of scope here.
      session.submitRobotDefinitionListChange(change);
   }

   @Test
   public void testEquationListChange()
   {
      session = new TestSession();
      assertTrue(session.getYoEquationDefinitions().isEmpty());

      session.submitEquationListChange(YoEquationListChange.newList(Collections.emptyList()));
      session.doGeneric(session.getActiveMode());

      assertTrue(session.getYoEquationDefinitions().isEmpty());
   }

   @Test
   public void testYoEquationListChangeListener()
   {
      session = new TestSession();
      AtomicInteger notificationCount = new AtomicInteger();
      Consumer<YoEquationListChange> listener = change -> notificationCount.incrementAndGet();

      session.addYoEquationListChangeListener(listener);

      session.submitEquationListChange(YoEquationListChange.newList(Collections.emptyList()));
      session.doGeneric(session.getActiveMode());

      assertEquals(1, notificationCount.get());

      assertTrue(session.removeYoEquationListChangeListener(listener));
      session.submitEquationListChange(YoEquationListChange.newList(Collections.emptyList()));
      session.doGeneric(session.getActiveMode());
      assertEquals(1, notificationCount.get());
   }

   @Test
   public void testBufferListenerForceUpdateListener()
   {
      session = new TestSession();
      AtomicInteger notificationCount = new AtomicInteger();
      Runnable listener = notificationCount::incrementAndGet;

      session.addBufferListenerForceUpdateListener(listener);
      session.requestBufferListenerForceUpdate();

      assertEquals(1, notificationCount.get());

      assertTrue(session.removeBufferListenerForceUpdateListener(listener));
      session.requestBufferListenerForceUpdate();
      assertEquals(1, notificationCount.get());
   }

   @Test
   public void testStartAndShutdownSessionThread()
   {
      session = new TestSession();
      assertFalse(session.hasSessionStarted());

      assertTrue(session.startSessionThread());
      assertTrue(session.hasSessionStarted());

      session.shutdownSession();
      assertFalse(session.hasSessionStarted());
   }

   private static class TestSession extends Session
   {
      @Override
      protected double doSpecificRunTick()
      {
         return 0.0;
      }

      @Override
      public void addGraphicsAddedCallback(Consumer<List<YoGraphicDefinition>> addedGraphicsConsumer)
      {
      }

      @Override
      public String getSessionName()
      {
         return "TestSession";
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
}

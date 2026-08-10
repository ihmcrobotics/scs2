package us.ihmc.scs2.simulation;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimSixDoFJoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class SimulationSessionResetTest
{
   private static final double EPSILON = 1.0e-12;
   private static final double INITIAL_HEIGHT = 1.5;
   private static final int SIMULATION_TICKS = 200;

   private SimulationSession session;
   private SimSixDoFJoint rootJoint;
   private YoDouble kp;
   private YoDouble effortOut;

   private SimulationSession createSession()
   {
      String name = "sphere";
      RobotDefinition sphereRobot = new RobotDefinition(name);
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJointDefinition);

      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
      rigidBody.setMass(2.0);
      rigidBody.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.1));
      rootJointDefinition.setSuccessor(rigidBody);
      sphereRobot.setRootBodyDefinition(rootBody);

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, INITIAL_HEIGHT));
      rootJointDefinition.setInitialJointState(initialState);

      session = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory());
      Robot robot = session.addRobot(sphereRobot);
      rootJoint = (SimSixDoFJoint) robot.getAllJoints().get(0);

      YoRegistry controllerRegistry = new YoRegistry("testController");
      kp = new YoDouble("kp", controllerRegistry);
      kp.set(10.0);
      effortOut = new YoDouble("effortOut", controllerRegistry);

      robot.getControllerManager().addController(new Controller()
      {
         @Override
         public void doControl()
         {
            effortOut.set(session.getTime().getValue() + 1.0);
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            // Note that initialize() deliberately does not touch kp so it behaves like a user-tuned parameter.
            return controllerRegistry;
         }
      });

      return session;
   }

   @Test
   public void testReset()
   {
      createSession();
      SimulationSessionControls controls = session.getSimulationSessionControls();

      double kpInitialValue = kp.getValue();
      assertTrue(controls.simulateNow(SIMULATION_TICKS));
      assertNotEquals(0.0, session.getTime().getValue());
      assertNotEquals(INITIAL_HEIGHT, rootJoint.getJointPose().getPosition().getZ());
      assertNotEquals(0.0, effortOut.getValue());

      int currentIndexBeforeReset = controls.getBufferCurrentIndex();
      assertEquals(controls.getBufferOutPoint(), currentIndexBeforeReset);

      kp.set(20.0); // Change a value mid-run, the reset restores it to its initial value.

      controls.resetToInitialState();

      assertEquals(0.0, session.getTime().getValue(), EPSILON);
      assertEquals(INITIAL_HEIGHT, rootJoint.getJointPose().getPosition().getZ(), EPSILON);
      assertEquals(0.0, rootJoint.getJointTwist().getLinearPart().getZ(), EPSILON);
      assertEquals(kpInitialValue, kp.getValue(), EPSILON);
      assertEquals(0.0, effortOut.getValue(), EPSILON); // Back to its value at the first initialization.
      assertEquals(SessionMode.PAUSE, session.getActiveMode());

      // The reset frame becomes the new start point of the buffer without clearing it.
      assertEquals(currentIndexBeforeReset, controls.getBufferCurrentIndex());
      assertEquals(currentIndexBeforeReset, controls.getBufferInPoint());
      assertEquals(currentIndexBeforeReset, controls.getBufferOutPoint());

      // The recording resumes from the reset frame.
      assertTrue(controls.simulateNow(10));
      assertEquals(currentIndexBeforeReset, controls.getBufferInPoint());
      assertEquals(currentIndexBeforeReset + 10, controls.getBufferOutPoint());
      controls.gotoBufferIndex(currentIndexBeforeReset);
      assertEquals(0.0, session.getTime().getValue(), EPSILON);
   }

   @Test
   public void testResetWhileRunning() throws Exception
   {
      createSession();
      SimulationSessionControls controls = session.getSimulationSessionControls();

      session.startSessionThread();

      try
      {
         controls.simulate();
         long deadline = System.currentTimeMillis() + 30000;
         while (session.getTime().getValue() < 0.005)
         {
            assertTrue(System.currentTimeMillis() < deadline, "Timed out waiting for the simulation to advance.");
            Thread.sleep(10);
         }

         controls.resetToInitialState();

         deadline = System.currentTimeMillis() + 30000;
         while (session.getActiveMode() != SessionMode.PAUSE || session.getTime().getValue() != 0.0)
         {
            assertTrue(System.currentTimeMillis() < deadline, "Timed out waiting for the reset to be performed.");
            Thread.sleep(10);
         }

         assertEquals(INITIAL_HEIGHT, rootJoint.getJointPose().getPosition().getZ(), EPSILON);

         // The session should remain paused after the reset.
         Thread.sleep(100);
         assertEquals(SessionMode.PAUSE, session.getActiveMode());
         assertEquals(0.0, session.getTime().getValue(), EPSILON);
      }
      finally
      {
         session.shutdownSession();
      }
   }

   @Test
   public void testSnapshotToleratesLateVariables()
   {
      createSession();
      SimulationSessionControls controls = session.getSimulationSessionControls();

      assertTrue(controls.simulateNow(SIMULATION_TICKS));

      YoRegistry lateRegistry = new YoRegistry("lateRegistry");
      YoDouble lateVariable = new YoDouble("lateVariable", lateRegistry);
      session.getRootRegistry().addChild(lateRegistry);
      lateVariable.set(5.0);

      assertDoesNotThrow(() -> controls.resetToInitialState());
      assertEquals(5.0, lateVariable.getValue(), EPSILON); // Not captured, thus not restored.
      assertEquals(0.0, session.getTime().getValue(), EPSILON);
   }

   @Test
   public void testResetUnsupportedSessionIsNoOp()
   {
      us.ihmc.scs2.session.Session unsupportedSession = new us.ihmc.scs2.session.Session()
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
            return "unsupportedSession";
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
      };

      assertDoesNotThrow(() -> unsupportedSession.submitSessionResetRequest());
   }
}

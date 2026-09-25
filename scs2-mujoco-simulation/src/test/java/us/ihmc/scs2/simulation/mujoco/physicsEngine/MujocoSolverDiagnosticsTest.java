package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;

/**
 * Covers the runtime option flags: the forward-vs-inverse solver diagnostic, and the disable flags
 * that let a sim be interrogated from the GUI without rebuilding it.
 */
public class MujocoSolverDiagnosticsTest
{
   private static final double DT = 1.0e-3;
   private static final String ROOT_JOINT = "root";
   private static final double SPAWN_HEIGHT = 0.2;

   private SimulationSession session;

   @BeforeAll
   public static void loadNatives()
   {
      assertTrue(MujocoNativeLibrary.load(), "MuJoCo native library failed to load");
   }

   @AfterEach
   public void shutdown()
   {
      if (session != null)
         session.shutdownSession();
      session = null;
   }

   private static RobotDefinition createBoxRobot()
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(ROOT_JOINT);
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition body = new RigidBodyDefinition("box");
      body.setMass(10.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.1));
      body.addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(0.2, 0.2, 0.1)));
      rootJoint.setSuccessor(body);

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, SPAWN_HEIGHT));
      rootJoint.setInitialJointState(initialState);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private static TerrainObjectDefinition createGround()
   {
      TerrainObjectDefinition ground = new TerrainObjectDefinition();
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.getTranslation().set(0.0, 0.0, -0.5);
      ground.addCollisionShapeDefinition(new CollisionShapeDefinition(pose, new Box3DDefinition(10.0, 10.0, 1.0)));
      return ground;
   }

   private Robot createSession()
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame,
                                                                                                rootRegistry,
                                                                                                new MujocoSimulationParameters()));
      session.addTerrainObject(createGround());
      session.addRobot(createBoxRobot());
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, -9.81);
      session.initializeBufferSize(5000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private MujocoPhysicsEngine engine()
   {
      return (MujocoPhysicsEngine) session.getPhysicsEngine();
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   private double rootHeight(Robot robot)
   {
      return ((SimFloatingJointBasics) robot.getFloatingRootJoint()).getJointPose().getZ();
   }

   /** With the flag on, a box settled on the ground should show a small, finite solver discrepancy. */
   @Test
   public void testFwdinvReportsAConvergedSolver()
   {
      Robot robot = createSession();
      engine().getOptions().enableFwdinv.set(true);
      simulate(500);

      double qfrc = engine().getStatistics().solver_fwdinv_qfrc.getValue();
      double efc = engine().getStatistics().solver_fwdinv_efc.getValue();
      assertTrue(Double.isFinite(qfrc), "solver_fwdinv_qfrc is not finite: " + qfrc);
      assertTrue(Double.isFinite(efc), "solver_fwdinv_efc is not finite: " + efc);
      assertTrue(Math.abs(qfrc) < 1.0, "Expected a near-converged solver on a resting box, got qfrc = " + qfrc);
   }

   /** With the flag off MuJoCo never writes the array, so a stale value must not read as converged. */
   @Test
   public void testFwdinvReadsNaNWhenDisabled()
   {
      createSession();
      simulate(100);

      assertTrue(Double.isNaN(engine().getStatistics().solver_fwdinv_qfrc.getValue()), "Expected NaN with enableFwdinv off");
      assertTrue(Double.isNaN(engine().getStatistics().solver_fwdinv_efc.getValue()), "Expected NaN with enableFwdinv off");
   }

   /** Proves the disable bits reach the flags MuJoCo actually reads. */
   @Test
   public void testDisableGravityHoldsTheBoxUp()
   {
      Robot robot = createSession();
      engine().getOptions().disableGravity.set(true);
      simulate(500);

      assertEquals(SPAWN_HEIGHT, rootHeight(robot), 1.0e-6, "The box moved with gravity disabled");
   }

   @Test
   public void testDisableContactDropsTheBoxThroughTheGround()
   {
      Robot robot = createSession();
      engine().getOptions().disableContact.set(true);
      simulate(500);

      assertTrue(rootHeight(robot) < -0.5, "Expected the box to fall through the terrain, z = " + rootHeight(robot));
   }

   /** The control: with nothing disabled the same box rests on the ground. */
   @Test
   public void testBoxRestsWithNothingDisabled()
   {
      Robot robot = createSession();
      simulate(500);

      assertEquals(0.05, rootHeight(robot), 5.0e-3, "The box did not come to rest on the ground");
   }
}

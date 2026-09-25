package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Covers {@code OneDoFJointDefinition.setArmature}, the reflected rotor inertia, reaching MuJoCo's
 * {@code dof_armature} -- and the model-wide {@code <default>} value it falls back to.
 */
public class MujocoJointArmatureTest
{
   private static final double DT = 1.0e-3;
   private static final String JOINT = "hinge";
   private static final double LINK_INERTIA = 0.01;

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

   private static RobotDefinition createRobot(double armature)
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");

      RevoluteJointDefinition joint = new RevoluteJointDefinition(JOINT, new Vector3D(), new Vector3D(0.0, 1.0, 0.0));
      joint.setArmature(armature);
      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(1.0);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(LINK_INERTIA, LINK_INERTIA, LINK_INERTIA));
      joint.setSuccessor(link);
      elevator.addChildJoint(joint);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private Robot createSession(double jointArmature, double defaultArmature)
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      parameters.set_armature(defaultArmature);
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      session.addRobot(createRobot(jointArmature));
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, 0.0);
      session.initializeBufferSize(2000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private MujocoPhysicsEngine engine()
   {
      return (MujocoPhysicsEngine) session.getPhysicsEngine();
   }

   private double dofArmature(Robot robot)
   {
      int dofAddress = engine().getMujocoRobots().get(0).getMujocoMultiBodyRobot().getJointAddress(JOINT).qveladr;
      return engine().getDynamicsWorld().getModel().dof_armature().get(dofAddress);
   }

   private void addConstantTorque(Robot robot, double tau)
   {
      OneDoFJointStateBasics output = robot.getControllerManager().getControllerOutput().getOneDoFJointOutput(JOINT);
      robot.getControllerManager().addController((Controller) () -> output.setEffort(tau));
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   @Test
   public void testPerJointArmatureReachesMujoco()
   {
      Robot robot = createSession(0.05, 0.0);
      simulate(1);
      assertEquals(0.05, dofArmature(robot), 1.0e-12, "The joint's armature did not reach dof_armature");
   }

   @Test
   public void testJointArmatureOverridesTheModelDefault()
   {
      Robot robot = createSession(0.05, 0.01);
      simulate(1);
      assertEquals(0.05, dofArmature(robot), 1.0e-12, "The per-joint value should win over the <default> block");
   }

   @Test
   public void testUnsetArmatureFallsBackToTheModelDefault()
   {
      Robot robot = createSession(0.0, 0.01);
      simulate(1);
      assertEquals(0.01, dofArmature(robot), 1.0e-12, "An unset joint armature should inherit the <default> block");
   }

   /**
    * Armature is inertia, so it has to slow the joint down, not just appear in the model. With
    * armature equal to the link inertia the joint should accelerate at half the rate.
    */
   @Test
   public void testArmatureHalvesTheAccelerationWhenItMatchesTheLinkInertia()
   {
      Robot withoutArmature = createSession(0.0, 0.0);
      addConstantTorque(withoutArmature, 0.1);
      simulate(500);
      double qdWithout = ((OneDoFJointBasics) withoutArmature.getJoint(JOINT)).getQd();
      shutdown();

      Robot withArmature = createSession(LINK_INERTIA, 0.0);
      addConstantTorque(withArmature, 0.1);
      simulate(500);
      double qdWith = ((OneDoFJointBasics) withArmature.getJoint(JOINT)).getQd();

      assertTrue(qdWithout > 0.0, "The torque did not move the joint");
      assertEquals(0.5 * qdWithout, qdWith, 1.0e-3 * qdWithout, "Armature equal to the link inertia should halve the acceleration");
   }
}

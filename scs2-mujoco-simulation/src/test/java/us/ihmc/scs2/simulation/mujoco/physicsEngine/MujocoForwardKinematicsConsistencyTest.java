package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Asserts the robot MuJoCo simulates has the same <i>geometry</i> as the mecano robot SCS2, the
 * controllers and the state estimator use. {@link MujocoMassMatrixConsistencyTest} is the companion
 * for inertial content; it is deliberately fixed-base and compares only joint-space mass, so it
 * cannot see a misplaced joint origin, a mis-signed or rotated joint axis, or a wrong floating-base
 * convention. Those show up as a robot that walks slightly wrong -- or, because the estimator
 * forward-kinematics the measured joint angles to locate the feet, as feet that drift away from
 * where the physics actually put them.
 *
 * <p>The comparison is body world pose from MuJoCo's {@code xpos}/{@code xquat} against the mecano
 * body-fixed frame's transform to world, for the same joint configuration, over a model with
 * deliberately awkward geometry: offset joint origins on all three axes, non-axis-aligned and
 * negative joint axes, a rotated fixed offset, and a floating base at a non-identity orientation.
 */
public class MujocoForwardKinematicsConsistencyTest
{
   private static final double DT = 1.0e-3;
   private static final String ROOT_JOINT = "rootJoint";
   /** MuJoCo compiles doubles straight through, so agreement should be to solver noise. */
   private static final double POSITION_TOLERANCE = 1.0e-9;
   private static final double ORIENTATION_TOLERANCE = 1.0e-9;

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

   private static RigidBodyDefinition link(String name, double mass)
   {
      RigidBodyDefinition body = new RigidBodyDefinition(name);
      body.setMass(mass);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.02, 0.03, 0.04));
      body.setCenterOfMassOffset(0.01, -0.02, 0.03);
      return body;
   }

   /** Floating base, then a chain whose joint origins and axes are all deliberately awkward. */
   private static RobotDefinition createRobot()
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(ROOT_JOINT);
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition pelvis = link("pelvis", 5.0);
      rootJoint.setSuccessor(pelvis);

      // offset on all three axes, axis not aligned to any frame axis
      Vector3D skewAxis = new Vector3D(0.3, -0.5, 0.81);
      skewAxis.normalize();
      RevoluteJointDefinition hip = new RevoluteJointDefinition("hip", new Vector3D(0.07, -0.13, -0.19), skewAxis);
      RigidBodyDefinition thigh = link("thigh", 3.0);
      hip.setSuccessor(thigh);
      pelvis.addChildJoint(hip);

      // negative axis, to catch a sign flip in the MJCF axis emission
      RevoluteJointDefinition knee = new RevoluteJointDefinition("knee", new Vector3D(-0.02, 0.05, -0.41), new Vector3D(0.0, -1.0, 0.0));
      RigidBodyDefinition shin = link("shin", 2.0);
      knee.setSuccessor(shin);
      thigh.addChildJoint(knee);

      RevoluteJointDefinition ankle = new RevoluteJointDefinition("ankle", new Vector3D(0.03, -0.01, -0.38), new Vector3D(1.0, 0.0, 0.0));
      RigidBodyDefinition foot = link("foot", 1.0);
      ankle.setSuccessor(foot);
      shin.addChildJoint(ankle);

      // Floating base at a non-identity orientation, so a world-vs-body mix-up in the base
      // convention cannot hide behind an identity rotation.
      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(new YawPitchRoll(0.4, -0.25, 0.15), new Point3D(0.11, -0.22, 1.35));
      rootJoint.setInitialJointState(initialState);

      setAngle(hip, 0.37);
      setAngle(knee, -0.62);
      setAngle(ankle, 0.23);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private static void setAngle(RevoluteJointDefinition joint, double q)
   {
      OneDoFJointState state = new OneDoFJointState();
      state.setConfiguration(q);
      joint.setInitialJointState(state);
   }

   @Test
   public void testMujocoBodyPosesMatchMecano()
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame,
                                                                                              rootRegistry,
                                                                                              new MujocoSimulationParameters()));
      session.addRobot(createRobot());
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, 0.0);
      session.initializeBufferSize(100);
      Robot robot = (Robot) session.getPhysicsEngine().getRobots().get(0);

      // One tick to push the seeded state into MuJoCo and pull it back, so both sides describe the
      // same configuration. Zero gravity and no contact means nothing else moves the robot.
      assertTrue(session.getSimulationSessionControls().simulateNow(1), "Simulation reported a failure");

      MujocoPhysicsEngine engine = (MujocoPhysicsEngine) session.getPhysicsEngine();
      var model = engine.getDynamicsWorld().getModel();
      var data = engine.getDynamicsWorld().getData();
      // xpos/xquat lag qpos by a step after mj_step, so refresh the forward pass before comparing.
      Mujoco.mj_forward(model, data);

      DoublePointer xpos = data.xpos();
      DoublePointer xquat = data.xquat();

      int compared = 0;
      for (String bodyName : new String[] {"pelvis", "thigh", "shin", "foot"})
      {
         int bodyId;
         try (BytePointer name = new BytePointer("robot_" + bodyName))
         {
            bodyId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_BODY, name);
         }
         assertTrue(bodyId >= 0, "body not in compiled model: robot_" + bodyName);

         // MuJoCo body frame == the SCS2 joint's frame-after-joint (that is what the MJCF builder
         // emits a <body> for), so compare against that rather than the body-fixed CoM frame.
         ReferenceFrame mecanoFrame = robot.getRigidBody(bodyName).getParentJoint().getFrameAfterJoint();
         RigidBodyTransform mecanoToWorld = new RigidBodyTransform(mecanoFrame.getTransformToRoot());

         assertEquals(mecanoToWorld.getTranslationX(), xpos.get(bodyId * 3), POSITION_TOLERANCE, bodyName + " world x");
         assertEquals(mecanoToWorld.getTranslationY(), xpos.get(bodyId * 3 + 1), POSITION_TOLERANCE, bodyName + " world y");
         assertEquals(mecanoToWorld.getTranslationZ(), xpos.get(bodyId * 3 + 2), POSITION_TOLERANCE, bodyName + " world z");

         // MuJoCo quaternion order is (w, x, y, z).
         Quaternion mujocoOrientation = new Quaternion(xquat.get(bodyId * 4 + 1),
                                                       xquat.get(bodyId * 4 + 2),
                                                       xquat.get(bodyId * 4 + 3),
                                                       xquat.get(bodyId * 4));
         Quaternion mecanoOrientation = new Quaternion(mecanoToWorld.getRotation());
         // q and -q are the same rotation, so compare the angle between them rather than components.
         double angle = mecanoOrientation.distance(mujocoOrientation);
         assertEquals(0.0, angle, ORIENTATION_TOLERANCE, bodyName + " world orientation differs by " + angle + " rad");
         compared++;
      }
      assertEquals(4, compared, "expected to compare all four bodies");

      // Guard the discriminating power: the chain must actually be rotated away from identity,
      // otherwise an axis or offset bug could pass unnoticed.
      OneDoFJointBasics knee = (OneDoFJointBasics) robot.getJoint("knee");
      assertTrue(Math.abs(knee.getQ()) > 0.1, "knee angle collapsed to zero; the test is no longer discriminating");
   }
}

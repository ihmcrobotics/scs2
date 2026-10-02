package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.bytedeco.javacpp.DoublePointer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
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
 * Pins down the reference point of MuJoCo's com-based {@code c*} family ({@code cvel},
 * {@code cacc}, {@code cfrc_*}), which {@link MujocoRobot#pullStateFromMujoco} depends on to convert
 * the floating root's spatial acceleration into the mecano joint acceleration that feeds
 * {@code SimIMUSensor} and, through it, the state estimator's accelerometer path.
 *
 * <p>The reference point is {@code subtree_com[body_rootid[bodyId]]}: the centre of mass of the
 * subtree rooted at the body's top-level ancestor, which for a floating robot is the centre of mass
 * of the <i>entire robot</i>. It is <b>not</b> the body's own centre of mass. The two coincide for a
 * single-body robot, which is exactly why a simple test model hides a wrong shift; the model here is
 * deliberately a light root with a heavy offset child so the two points are ~1 m apart.
 *
 * <p>The velocity form is asserted rather than the acceleration form because a spatial velocity can
 * be predicted exactly from {@code qpos}/{@code qvel} by {@code v_S = v_O + w x (S - O)}, with no
 * finite differencing and no tolerance to tune. The acceleration obeys the same frame convention.
 */
public class MujocoComFrameReferenceTest
{
   private static final double DT = 1.0e-3;
   private static final String ROOT_JOINT = "rootJoint";
   private static final String SPINE_JOINT = "spine";
   /** Exact to solver precision: this is an algebraic identity, not a simulation comparison. */
   private static final double TOLERANCE = 1.0e-10;

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

   /**
    * Light root whose own centre of mass sits at its body frame origin, plus a heavy child 1 m up.
    * That puts the whole-robot centre of mass ~0.98 m from both the root's origin and the root's own
    * centre of mass, so confusing the two is a ~1 m lever-arm error rather than a rounding error.
    */
   private static RobotDefinition createRobot()
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(ROOT_JOINT);
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition pelvis = new RigidBodyDefinition("pelvis");
      pelvis.setMass(1.0);
      pelvis.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      rootJoint.setSuccessor(pelvis);

      RevoluteJointDefinition spine = new RevoluteJointDefinition(SPINE_JOINT, new Vector3D(0.0, 0.0, 1.0), new Vector3D(0.0, 1.0, 0.0));
      RigidBodyDefinition torso = new RigidBodyDefinition("torso");
      torso.setMass(50.0);
      torso.setMomentOfInertia(new MomentOfInertiaDefinition(1.0, 1.0, 1.0));
      spine.setSuccessor(torso);
      pelvis.addChildJoint(spine);

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, 2.0));
      // Distinctive twist: the linear part alone would satisfy the body-origin prediction, so the
      // angular part is what makes the two candidate reference points disagree.
      initialState.setVelocity(new Vector3D(0.0, 2.0, 0.0), new Vector3D(0.3, 0.0, 0.0));
      rootJoint.setInitialJointState(initialState);

      OneDoFJointState spineState = new OneDoFJointState();
      spineState.setConfiguration(0.4);
      spineState.setVelocity(0.7);
      spine.setInitialJointState(spineState);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private Robot createSession()
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame,
                                                                                              rootRegistry,
                                                                                              new MujocoSimulationParameters()));
      session.addRobot(createRobot());
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, 0.0);   // isolate the kinematics; gravity is reference-point independent
      session.initializeBufferSize(1000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   @Test
   public void testComBasedFrameIsReferencedAtSubtreeComNotBodyCom()
   {
      Robot robot = createSession();
      assertTrue(session.getSimulationSessionControls().simulateNow(20), "Simulation reported a failure");

      MujocoPhysicsEngine engine = (MujocoPhysicsEngine) session.getPhysicsEngine();
      var model = engine.getDynamicsWorld().getModel();
      var data = engine.getDynamicsWorld().getData();

      // mj_step integrates qpos/qvel but does NOT recompute the position-dependent arrays, so after
      // a step xpos, subtree_com, cvel and cacc all still describe the state at the START of that
      // step while qpos/qvel describe its end. Re-run the forward pass so every array below refers
      // to one instant; otherwise the identity being asserted is off by a tick (~1.5e-4 here) and
      // the test would be measuring staleness instead of the frame convention.
      Mujoco.mj_forward(model, data);

      int bodyId;
      try (org.bytedeco.javacpp.BytePointer name = new org.bytedeco.javacpp.BytePointer("robot_pelvis"))
      {
         bodyId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_BODY, name);
      }
      assertTrue(bodyId >= 0, "pelvis body not found in the compiled model");

      int rootBodyId = model.body_rootid().get(bodyId);
      DoublePointer xpos = data.xpos();
      DoublePointer xipos = data.xipos();
      DoublePointer subtreeCom = data.subtree_com();
      DoublePointer qvel = data.qvel();
      DoublePointer cvel = data.cvel();

      Vector3D bodyOrigin = new Vector3D(xpos.get(bodyId * 3), xpos.get(bodyId * 3 + 1), xpos.get(bodyId * 3 + 2));
      Vector3D bodyOwnCom = new Vector3D(xipos.get(bodyId * 3), xipos.get(bodyId * 3 + 1), xipos.get(bodyId * 3 + 2));
      Vector3D subtreeComPoint = new Vector3D(subtreeCom.get(rootBodyId * 3),
                                              subtreeCom.get(rootBodyId * 3 + 1),
                                              subtreeCom.get(rootBodyId * 3 + 2));

      // Guard the discriminating power of the model itself: if these points ever coincide the
      // assertions below stop distinguishing the two conventions and the test silently goes blind.
      Vector3D comSeparation = new Vector3D();
      comSeparation.sub(subtreeComPoint, bodyOwnCom);
      assertTrue(comSeparation.norm() > 0.5,
                 "Test model is degenerate: whole-robot CoM and root-body CoM are " + comSeparation.norm()
                 + " m apart, so this test cannot tell the two reference points apart.");

      // Root joint state. MuJoCo freejoint qvel is split: linear in world, angular in body frame,
      // so the angular part has to be rotated out to world to build the world-frame prediction.
      SixDoFJointBasics rootJointState = (SixDoFJointBasics) robot.getJoint(ROOT_JOINT);
      Vector3D angularVelocityWorld = new Vector3D(qvel.get(3), qvel.get(4), qvel.get(5));
      rootJointState.getJointPose().getOrientation().transform(angularVelocityWorld);
      Vector3D originVelocityWorld = new Vector3D(qvel.get(0), qvel.get(1), qvel.get(2));

      // v_S = v_O + w x (S - O)
      Vector3D originToSubtreeCom = new Vector3D();
      originToSubtreeCom.sub(subtreeComPoint, bodyOrigin);
      Vector3D predictedAtSubtreeCom = new Vector3D();
      predictedAtSubtreeCom.cross(angularVelocityWorld, originToSubtreeCom);
      predictedAtSubtreeCom.add(originVelocityWorld);

      // cvel layout is (rot:lin), so the linear part starts at +3.
      Vector3D actual = new Vector3D(cvel.get(bodyId * 6 + 3), cvel.get(bodyId * 6 + 4), cvel.get(bodyId * 6 + 5));

      assertEquals(predictedAtSubtreeCom.getX(), actual.getX(), TOLERANCE, "cvel linear x is not referenced at subtree_com");
      assertEquals(predictedAtSubtreeCom.getY(), actual.getY(), TOLERANCE, "cvel linear y is not referenced at subtree_com");
      assertEquals(predictedAtSubtreeCom.getZ(), actual.getZ(), TOLERANCE, "cvel linear z is not referenced at subtree_com");

      // And confirm the body-origin reading really is wrong, so a future change that makes both
      // predictions agree cannot quietly pass.
      Vector3D originError = new Vector3D();
      originError.sub(actual, originVelocityWorld);
      assertTrue(originError.norm() > 0.1,
                 "cvel matched the body origin too, so this test no longer discriminates the reference point");

      // The angular part is a free vector: identical at every reference point.
      Vector3D actualAngular = new Vector3D(cvel.get(bodyId * 6), cvel.get(bodyId * 6 + 1), cvel.get(bodyId * 6 + 2));
      assertEquals(angularVelocityWorld.getX(), actualAngular.getX(), TOLERANCE, "cvel angular x");
      assertEquals(angularVelocityWorld.getY(), actualAngular.getY(), TOLERANCE, "cvel angular y");
      assertEquals(angularVelocityWorld.getZ(), actualAngular.getZ(), TOLERANCE, "cvel angular z");
   }
}

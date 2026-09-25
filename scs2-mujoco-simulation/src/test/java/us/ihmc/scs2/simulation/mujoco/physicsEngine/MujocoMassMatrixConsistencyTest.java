package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.bytedeco.javacpp.DoublePointer;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Asserts the model MuJoCo compiled has the same inertial content as the mecano model SCS2 and the
 * controllers use, by comparing joint-space mass matrices.
 *
 * <p>This is the cheap safety net for the MJCF builder. Mass, centre-of-mass offset, the inertia
 * tensor and its frame rotation, armature, and whether an ignored subtree kept its inertia all land
 * in the mass matrix, so a single comparison covers the lot -- and a builder bug shows up here as a
 * number rather than as a robot that walks slightly wrong.
 *
 * <p>Deliberately fixed-base: for a free joint MuJoCo and mecano order and frame the six base
 * velocity components differently, and untangling that would test the test rather than the builder.
 * Everything the builder can get wrong about a body's inertia still reaches the joint-space block.
 */
public class MujocoMassMatrixConsistencyTest
{
   private static final double DT = 1.0e-3;
   private static final String IGNORED_JOINT = "ignored";
   private static final double TOLERANCE = 1.0e-9;

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
    * A fixed-base three-link arm with deliberately awkward inertias: off-centre CoMs, a non-diagonal
    * tensor, a rotated inertia frame, and a tip link on a joint the controller ignores.
    */
   private static RobotDefinition createArm(boolean ignoreTipJoint, double armature)
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");

      RigidBodyDefinition parent = elevator;
      String[] jointNames = {"shoulder", "elbow", "wrist"};
      Vector3D[] axes = {new Vector3D(0.0, 1.0, 0.0), new Vector3D(1.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 1.0)};

      for (int i = 0; i < jointNames.length; i++)
      {
         RevoluteJointDefinition joint = new RevoluteJointDefinition(jointNames[i], new Vector3D(0.0, 0.0, -0.3), axes[i]);
         joint.setArmature(armature);
         RigidBodyDefinition link = new RigidBodyDefinition("link" + i);
         link.setMass(2.0 + i);
         link.setCenterOfMassOffset(0.02 * (i + 1), -0.03, -0.15);
         MomentOfInertiaDefinition inertia = new MomentOfInertiaDefinition(0.05 + 0.01 * i, 0.06, 0.04);
         // Off-diagonal terms and a rotated inertia frame, both of which the builder has to carry.
         inertia.setM01(0.004);
         inertia.setM10(0.004);
         link.setMomentOfInertia(inertia);
         link.getInertiaPose().getRotation().set(new YawPitchRoll(0.3, -0.2, 0.1));
         joint.setSuccessor(link);
         parent.addChildJoint(joint);
         parent = link;
      }

      // A tip link hanging off a joint the controller does not drive. The other engines keep its
      // inertia rigidly attached, so MuJoCo has to weld it rather than drop it.
      RevoluteJointDefinition tipJoint = new RevoluteJointDefinition(IGNORED_JOINT, new Vector3D(0.0, 0.0, -0.25), new Vector3D(0.0, 1.0, 0.0));
      tipJoint.setArmature(armature);
      RigidBodyDefinition tip = new RigidBodyDefinition("tip");
      tip.setMass(1.5);
      tip.setCenterOfMassOffset(0.0, 0.0, -0.08);
      tip.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      tipJoint.setSuccessor(tip);
      parent.addChildJoint(tipJoint);

      robot.setRootBodyDefinition(elevator);
      if (ignoreTipJoint)
         robot.addJointToIgnore(IGNORED_JOINT);
      return robot;
   }

   private Robot createSession(boolean ignoreTipJoint, double armature)
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame,
                                                                                                rootRegistry,
                                                                                                new MujocoSimulationParameters()));
      session.addRobot(createArm(ignoreTipJoint, armature));
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, -9.81);
      session.initializeBufferSize(1000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private MujocoPhysicsEngine engine()
   {
      return (MujocoPhysicsEngine) session.getPhysicsEngine();
   }

   /** Puts the arm somewhere non-trivial so the comparison is not made at a degenerate configuration. */
   private void poseArm(Robot robot)
   {
      double[] angles = {0.4, -0.7, 0.25};
      String[] jointNames = {"shoulder", "elbow", "wrist"};
      for (int i = 0; i < jointNames.length; i++)
         ((OneDoFJointBasics) robot.getJoint(jointNames[i])).setQ(angles[i]);
      robot.updateFrames();
      assertTrue(session.getSimulationSessionControls().simulateNow(1), "Simulation reported a failure");
   }

   /** MuJoCo's dense joint-space inertia matrix, with armature removed so it is pure rigid-body inertia. */
   private DMatrixRMaj readMujocoMassMatrix(boolean subtractArmature)
   {
      var model = engine().getDynamicsWorld().getModel();
      var data = engine().getDynamicsWorld().getData();
      // qM is computed during the forward pass at the start of mj_step, so straight after a step it
      // belongs to the configuration the step started from, not the one it landed in. Recompute it
      // for the current qpos, which is what the SCS2 joints now hold.
      Mujoco.mj_forward(model, data);
      int nv = (int) model.nv();
      DoublePointer dense = new DoublePointer(nv * (long) nv);
      Mujoco.mj_fullM(model, data, dense);

      DMatrixRMaj massMatrix = new DMatrixRMaj(nv, nv);
      for (int row = 0; row < nv; row++)
      {
         for (int col = 0; col < nv; col++)
            massMatrix.set(row, col, dense.get(row * (long) nv + col));
         if (subtractArmature)
            massMatrix.add(row, row, -model.dof_armature().get(row));
      }
      return massMatrix;
   }

   /** mecano's mass matrix for the same robot, with ignored subtrees rigidly attached as the other engines treat them. */
   private DMatrixRMaj computeMecanoMassMatrix(Robot robot, List<JointReadOnly> jointOrderToFill)
   {
      List<JointReadOnly> jointsToIgnore = new ArrayList<>();
      for (JointReadOnly joint : robot.getRootBody().childrenSubtreeIterable())
      {
         if (IGNORED_JOINT.equals(joint.getName()) && robot.getRobotDefinition().getNameOfJointsToIgnore().contains(IGNORED_JOINT))
            jointsToIgnore.add(joint);
      }

      MultiBodySystemReadOnly input = MultiBodySystemReadOnly.toMultiBodySystemInput(robot.getRootBody(), jointsToIgnore);
      CompositeRigidBodyMassMatrixCalculator calculator = new CompositeRigidBodyMassMatrixCalculator(input);
      calculator.reset();
      jointOrderToFill.addAll(input.getJointsToConsider());
      return calculator.getMassMatrix();
   }

   private void assertMassMatricesMatch(Robot robot, boolean subtractArmature)
   {
      List<JointReadOnly> mecanoJointOrder = new ArrayList<>();
      DMatrixRMaj mecano = computeMecanoMassMatrix(robot, mecanoJointOrder);
      DMatrixRMaj mujoco = readMujocoMassMatrix(subtractArmature);

      assertEquals(mecanoJointOrder.size(), mujoco.getNumRows(), "MuJoCo and mecano disagree on the number of degrees of freedom");

      // The two models order their degrees of freedom independently, so map through joint names.
      int[] mujocoIndex = new int[mecanoJointOrder.size()];
      for (int i = 0; i < mecanoJointOrder.size(); i++)
      {
         String jointName = mecanoJointOrder.get(i).getName();
         var address = engine().getMujocoRobots().get(0).getMujocoMultiBodyRobot().getJointAddress(jointName);
         assertTrue(address != null, "MuJoCo has no joint named " + jointName);
         mujocoIndex[i] = address.qveladr;
      }

      for (int row = 0; row < mecano.getNumRows(); row++)
      {
         for (int col = 0; col < mecano.getNumCols(); col++)
         {
            assertEquals(mecano.get(row, col),
                         mujoco.get(mujocoIndex[row], mujocoIndex[col]),
                         TOLERANCE,
                         "Mass matrix mismatch at (" + mecanoJointOrder.get(row).getName() + ", " + mecanoJointOrder.get(col).getName() + ")");
         }
      }
   }

   /**
    * The baseline: awkward inertias, off-centre CoMs, off-diagonal terms and a rotated inertia
    * frame all have to survive the trip into MJCF.
    */
   @Test
   public void testMassMatrixMatchesMecano()
   {
      Robot robot = createSession(false, 0.0);
      poseArm(robot);
      assertMassMatricesMatch(robot, false);
   }

   /**
    * The case that used to be wrong: a subtree the controller ignores must keep its inertia, welded
    * to its parent, rather than disappearing from the model.
    */
   @Test
   public void testIgnoredSubtreeKeepsItsInertia()
   {
      Robot robot = createSession(true, 0.0);
      poseArm(robot);
      assertMassMatricesMatch(robot, false);
   }

   /** Armature is MuJoCo-side inertia mecano does not model, so it must appear exactly on the diagonal. */
   @Test
   public void testArmatureAppearsOnlyOnTheDiagonal()
   {
      double armature = 0.037;
      Robot robot = createSession(false, armature);
      poseArm(robot);

      // With armature subtracted the two agree again, which pins it to the diagonal alone.
      assertMassMatricesMatch(robot, true);

      DMatrixRMaj withArmature = readMujocoMassMatrix(false);
      DMatrixRMaj withoutArmature = readMujocoMassMatrix(true);
      for (int row = 0; row < withArmature.getNumRows(); row++)
         assertEquals(armature, withArmature.get(row, row) - withoutArmature.get(row, row), TOLERANCE, "Armature missing from dof " + row);
   }
}

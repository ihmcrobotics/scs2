package us.ihmc.scs2.simulation.physicsEngine.contactPointBased;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator.JointSourceMode;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.scs2.simulation.RobotJointWrenchCalculator;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.FrameShapePosePredictor;
import us.ihmc.scs2.simulation.physicsEngine.YoMatrix;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.controller.RobotOneDoFJointDampingCalculator;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.scs2.simulation.screwTools.SimJointStateType;
import us.ihmc.scs2.simulation.screwTools.SimMultiBodySystemTools;
import us.ihmc.scs2.simulation.screwTools.SingleRobotFirstOrderIntegrator;

import java.util.List;
import java.util.stream.Collectors;

public class ContactPointBasedRobotPhysics
{
   private final RobotInterface owner;
   private final ReferenceFrame inertialFrame;

   private final RobotOneDoFJointDampingCalculator robotOneDoFJointDampingCalculator;
   private final RobotOneDoFJointSoftLimitCalculator robotOneDoFJointSoftLimitCalculator;

   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry = new RigidBodyWrenchRegistry();

   private final List<Collidable> collidables;

   // TODO Following fields are specific to the type of engine used, they need interfacing.
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;
   private RobotJointWrenchCalculator jointWrenchCalculator;

   private final JointMatrixIndexProvider indexProvider;

   /** The joint torques imposed by physics simulation, consisting of damping + soft enforcement of joint limits. */
   private final YoMatrix jointsTauLowLevelController;

   /** The resultant joint torques to be used in the forward dynamics calculation (i.e. the calculation for the result of the simulation). */
   private final YoMatrix jointsTau;

   private final SingleRobotFirstOrderIntegrator integrator;

   private final RobotPhysicsOutput physicsOutput;

   public ContactPointBasedRobotPhysics(RobotInterface owner)
   {
      this.owner = owner;
      inertialFrame = owner.getInertialFrame();

      robotOneDoFJointDampingCalculator = new RobotOneDoFJointDampingCalculator(owner);
      owner.getRegistry().addChild(robotOneDoFJointDampingCalculator.getRegistry());
      robotOneDoFJointSoftLimitCalculator = new RobotOneDoFJointSoftLimitCalculator(owner);
      owner.getRegistry().addChild(robotOneDoFJointSoftLimitCalculator.getRegistry());

      SimRigidBodyBasics rootBody = owner.getRootBody();
      collidables = rootBody.subtreeStream().flatMap(body -> body.getCollidables().stream()).collect(Collectors.toList());

      forwardDynamicsCalculator = new ForwardDynamicsCalculator(owner);
      FrameShapePosePredictor frameShapePosePredictor = new FrameShapePosePredictor(forwardDynamicsCalculator);
      collidables.forEach(collidable -> collidable.setFrameShapePosePredictor(frameShapePosePredictor));

      indexProvider = JointMatrixIndexProvider.toIndexProvider(owner.getAllJoints().toArray(new JointBasics[0]));
      int nDoFs = indexProvider.getIndexedJointsInOrder().stream().map(joint -> joint.getDegreesOfFreedom()).reduce(0, Integer::sum);

      String[] rowNames = getRowNames(owner, nDoFs);
      jointsTauLowLevelController = new YoMatrix("tau_llc", "Joint torque contribution from low-level control",
                                                 nDoFs, 1, rowNames, null, owner.getRegistry());
      jointsTau = new YoMatrix("tau_total",
                               "Total joint torque, sum of controller contribution and low-level control contribution",
                               nDoFs, 1, rowNames, null, owner.getRegistry());

      integrator = new SingleRobotFirstOrderIntegrator();

      physicsOutput = new RobotPhysicsOutput(forwardDynamicsCalculator.getAccelerationProvider(), null, rigidBodyWrenchRegistry, null);
   }

   public void enableJointWrenchCalculator()
   {
      if (jointWrenchCalculator != null)
         return;

      jointWrenchCalculator = new RobotJointWrenchCalculator(physicsOutput, forwardDynamicsCalculator, owner.getRegistry());
   }

   public void resetCalculators()
   {
      forwardDynamicsCalculator.setExternalWrenchesToZero();
      rigidBodyWrenchRegistry.reset();
   }

   public void computeJointLowLevelControl()
   {
      jointsTauLowLevelController.zero();  // this variable is appended to, not overwritten, so it is imperative that it is first zeroed here
      robotOneDoFJointDampingCalculator.compute(jointsTauLowLevelController);
      robotOneDoFJointSoftLimitCalculator.compute(jointsTauLowLevelController);
   }

   public void addRigidBodyExternalWrench(RigidBodyReadOnly target, WrenchReadOnly wrenchToAdd)
   {
      rigidBodyWrenchRegistry.addWrench(target, wrenchToAdd);
   }

   public void updateCollidableBoundingBoxes()
   {
      collidables.forEach(collidable -> collidable.updateBoundingBox(inertialFrame));
   }

   public List<Collidable> getCollidables()
   {
      return collidables;
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculator()
   {
      return forwardDynamicsCalculator;
   }

   /**
    * Compute the forward dynamics of the robot subject to {@code gravity}. All joint torques from controllers and low-level are expected to have been computed.
    * <p>
    * NOTE: by the time this method is called, the joint torques from low-level control will have already been computed (see
    * {@link ContactPointBasedPhysicsEngine#simulate(double, double, Vector3DReadOnly)}), therefore jointsTauLowLevelControl will have been populated.
    * </p>
    *
    * @param gravity the gravitational acceleration to use for the forward dynamics.
    */
   public void doForwardDynamics(Vector3DReadOnly gravity)
   {
      forwardDynamicsCalculator.setGravitationalAcceleration(gravity);
      forwardDynamicsCalculator.setJointSourceModes(joint ->
                                                    {
                                                       SimJointBasics simJoint = (SimJointBasics) joint;
                                                       if (simJoint.isPinned())
                                                       {
                                                          simJoint.setJointTwistToZero();
                                                          simJoint.setJointAccelerationToZero();
                                                       }
                                                       return simJoint.isPinned() ? JointSourceMode.ACCELERATION_SOURCE : JointSourceMode.EFFORT_SOURCE;
                                                    });
      // As the joint torques from low-level control have already been computed, we can now sum them with the joint torques from controllers
      sumJointTauContributions();
      forwardDynamicsCalculator.compute(jointsTau);
   }

   public void writeJointAccelerations()
   {
      List<? extends SimJointBasics> joints = owner.getJointsToConsider();
      DMatrixRMaj jointAccelerationMatrix = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj jointTauMatrix = forwardDynamicsCalculator.getJointTauMatrix();
      SimMultiBodySystemTools.insertJointsStateWithBackup(joints,
                                                          SimJointBasics::isPinned,
                                                          SimJointStateType.EFFORT,
                                                          jointTauMatrix,
                                                          Double.POSITIVE_INFINITY,
                                                          false,
                                                          SimJointStateType.ACCELERATION,
                                                          jointAccelerationMatrix,
                                                          1.0e12,
                                                          true);
   }

   public void computeJointWrenches(double dt)
   {
      if (jointWrenchCalculator == null)
         return;

      jointWrenchCalculator.update(dt);
   }

   public void integrateState(double dt)
   {
      physicsOutput.setDT(dt);
      integrator.integrate(dt, owner);
   }

   public RobotPhysicsOutput getPhysicsOutput()
   {
      return physicsOutput;
   }

   private void sumJointTauContributions()
   {
      for (JointBasics joint : owner.getJointsToConsider())
      {
         int[] jointIndices = indexProvider.getJointDoFIndices(joint);

         // Pack with joint torques from controller
         joint.getJointTau(jointIndices[0], jointsTau);

         // Elementwise, add joint torques from low-level control
         for (int jointIndex : jointIndices)
         {
            jointsTau.set(jointIndex, 0, jointsTau.get(jointIndex, 0) + jointsTauLowLevelController.get(jointIndex, 0));
         }
      }
   }

   private String[] getRowNames(RobotInterface owner, int nDoFs)
   {
      String[] rowNames = new String[nDoFs];
      int index = 0;
      for (JointReadOnly joint : owner.getAllJoints())
      {
         if (joint.getDegreesOfFreedom() > 1)
         {
            for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
               rowNames[index + i] = joint.getName() + "_" + i;
         }
         else
         {
            rowNames[index] = joint.getName();
         }
         index += joint.getDegreesOfFreedom();
      }
      return rowNames;
   }
}

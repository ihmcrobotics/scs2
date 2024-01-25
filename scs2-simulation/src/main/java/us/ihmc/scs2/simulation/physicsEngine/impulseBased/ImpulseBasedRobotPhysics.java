package us.ihmc.scs2.simulation.physicsEngine.impulseBased;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator.JointSourceMode;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.simulation.RobotJointWrenchCalculator;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.FrameShapePosePredictor;
import us.ihmc.scs2.simulation.physicsEngine.YoMatrix;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.controller.RobotOneDoFJointDampingCalculator;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.*;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class ImpulseBasedRobotPhysics
{
   private static final String ContactCalculatorNameSuffix = SingleContactImpulseCalculator.class.getSimpleName();

   private final RobotInterface owner;
   private final ReferenceFrame inertialFrame;

   private final YoRegistry environmentContactCalculatorRegistry = new YoRegistry("Environment" + ContactCalculatorNameSuffix);
   private final YoRegistry interRobotContactCalculatorRegistry = new YoRegistry("InterRobot" + ContactCalculatorNameSuffix);
   private final YoRegistry selfContactCalculatorRegistry = new YoRegistry("Self" + ContactCalculatorNameSuffix);

   private final DMatrixRMaj jointDeltaVelocityMatrix;
   private final RigidBodyDeltaTwistCalculator rigidBodyDeltaTwistCalculator;
   private final RigidBodyTwistProvider rigidBodyDeltaTwistProvider;
   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry = new RigidBodyWrenchRegistry();
   private final RigidBodyImpulseRegistry rigidBodyImpulseRegistry = new RigidBodyImpulseRegistry();

   private final List<Collidable> collidables;

   // TODO Following fields are specific to the type of engine used, they need interfacing.
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;
   private RobotJointWrenchCalculator jointWrenchCalculator;
   private final RobotJointLimitImpulseBasedCalculator jointLimitConstraintCalculator;
   private final YoSingleContactImpulseCalculatorPool environmentContactConstraintCalculatorPool;
   private final YoSingleContactImpulseCalculatorPool selfContactConstraintCalculatorPool;
   private final Map<RigidBodyBasics, YoSingleContactImpulseCalculatorPool> interRobotContactConstraintCalculatorPools = new HashMap<>();

   private final RobotOneDoFJointDampingCalculator robotOneDoFJointDampingCalculator;

   private final JointMatrixIndexProvider indexProvider;

   /** The joint torques imposed by physics simulation, consisting of damping + soft enforcement of joint limits. */
   private final YoMatrix jointsTauLowLevelController;

   /** The resultant joint torques to be used in the forward dynamics calculation (i.e. the calculation for the result of the simulation). */
   private final YoMatrix jointsTau;

   private final SingleRobotFirstOrderIntegrator integrator;

   private final RobotPhysicsOutput physicsOutput;

   public ImpulseBasedRobotPhysics(RobotInterface owner, YoRegistry robotPhysicsRegistry)
   {
      this.owner = owner;
      inertialFrame = owner.getInertialFrame();

      jointDeltaVelocityMatrix = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(owner.getJointsToConsider()), 1);
      rigidBodyDeltaTwistCalculator = new RigidBodyDeltaTwistCalculator(inertialFrame, owner.getJointMatrixIndexProvider(), jointDeltaVelocityMatrix);
      rigidBodyDeltaTwistProvider = rigidBodyDeltaTwistCalculator.getDeltaTwistProvider();

      SimRigidBodyBasics rootBody = owner.getRootBody();
      collidables = rootBody.subtreeStream()
                            .filter(body -> owner.getJointsToConsider().contains(body.getParentJoint()))
                            .flatMap(body -> body.getCollidables().stream())
                            .collect(Collectors.toList());

      forwardDynamicsCalculator = new ForwardDynamicsCalculator(owner);
      FrameShapePosePredictor frameShapePosePredictor = new FrameShapePosePredictor(forwardDynamicsCalculator);
      collidables.forEach(collidable -> collidable.setFrameShapePosePredictor(frameShapePosePredictor));

      YoRegistry jointLimitConstraintCalculatorRegistry = new YoRegistry(RobotJointLimitImpulseBasedCalculator.class.getSimpleName());
      jointLimitConstraintCalculator = new YoRobotJointLimitImpulseBasedCalculator(owner, forwardDynamicsCalculator, jointLimitConstraintCalculatorRegistry);

      robotOneDoFJointDampingCalculator = new RobotOneDoFJointDampingCalculator(owner);

      environmentContactConstraintCalculatorPool = new YoSingleContactImpulseCalculatorPool(20,
                                                                                            owner.getName() + "Single",
                                                                                            inertialFrame,
                                                                                            rootBody,
                                                                                            forwardDynamicsCalculator,
                                                                                            null,
                                                                                            null,
                                                                                            environmentContactCalculatorRegistry);

      selfContactConstraintCalculatorPool = new YoSingleContactImpulseCalculatorPool(8,
                                                                                     owner.getName() + "Self",
                                                                                     inertialFrame,
                                                                                     rootBody,
                                                                                     forwardDynamicsCalculator,
                                                                                     rootBody,
                                                                                     forwardDynamicsCalculator,
                                                                                     selfContactCalculatorRegistry);

      indexProvider = JointMatrixIndexProvider.toIndexProvider(owner.getAllJoints().toArray(new JointBasics[0]));
      int nDoFs = indexProvider.getIndexedJointsInOrder().stream().map(joint -> joint.getDegreesOfFreedom()).reduce(0, Integer::sum);

      String[] rowNames = getRowNames(owner, nDoFs);
      jointsTauLowLevelController = new YoMatrix("tau_llc", "Joint torque contribution from low-level control",
                                                 nDoFs, 1, rowNames, null, owner.getRegistry());
      jointsTau = new YoMatrix("tau_total", "Total joint torque, sum of controller contribution and low-level control contribution",
                               nDoFs, 1, rowNames, null, owner.getRegistry());

      integrator = new SingleRobotFirstOrderIntegrator();

      physicsOutput = new RobotPhysicsOutput(forwardDynamicsCalculator.getAccelerationProvider(),
                                             rigidBodyDeltaTwistProvider,
                                             rigidBodyWrenchRegistry,
                                             rigidBodyImpulseRegistry);

      robotPhysicsRegistry.addChild(jointLimitConstraintCalculatorRegistry);
      robotPhysicsRegistry.addChild(environmentContactCalculatorRegistry);
      robotPhysicsRegistry.addChild(interRobotContactCalculatorRegistry);
      robotPhysicsRegistry.addChild(selfContactCalculatorRegistry);
      robotPhysicsRegistry.addChild(robotOneDoFJointDampingCalculator.getRegistry());
   }

   public void enableJointWrenchCalculator()
   {
      if (jointWrenchCalculator != null)
         return;

      jointWrenchCalculator = new RobotJointWrenchCalculator(physicsOutput, forwardDynamicsCalculator, owner.getRegistry());
   }

   public void resetCalculators()
   {
      jointDeltaVelocityMatrix.zero();
      forwardDynamicsCalculator.setExternalWrenchesToZero();
      rigidBodyDeltaTwistCalculator.reset();
      rigidBodyWrenchRegistry.reset();
      rigidBodyImpulseRegistry.reset();
      environmentContactConstraintCalculatorPool.clear();
      selfContactConstraintCalculatorPool.clear();
      interRobotContactConstraintCalculatorPools.forEach((rigidBodyBasics, calculators) -> calculators.clear());
   }

   public void computeJointLowLevelControl()
   {
      jointsTauLowLevelController.zero();  // this variable is appended to, not overwritten, so it is imperative that it is first zeroed here
      robotOneDoFJointDampingCalculator.compute();
   }

   private void computeJointDamping()
   {
      robotOneDoFJointDampingCalculator.compute(jointsTauLowLevelController);
   }

   public void addJointVelocityChange(DMatrixRMaj velocityChange)
   {
      if (velocityChange == null)
         return;
      CommonOps_DDRM.addEquals(jointDeltaVelocityMatrix, velocityChange);
   }

   public void addRigidBodyExternalWrench(RigidBodyReadOnly target, WrenchReadOnly wrenchToAdd)
   {
      rigidBodyWrenchRegistry.addWrench(target, wrenchToAdd);
   }

   public void addRigidBodyExternalImpulse(RigidBodyReadOnly target, SpatialImpulseReadOnly wrenchToAdd)
   {
      rigidBodyImpulseRegistry.addImpulse(target, wrenchToAdd);
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

   public void writeJointDeltaVelocities()
   {
      SimMultiBodySystemTools.insertJointsState(owner.getJointsToConsider(), SimJointStateType.VELOCITY_CHANGE, jointDeltaVelocityMatrix, 1.0e7, true);
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

   public RobotJointLimitImpulseBasedCalculator getJointLimitConstraintCalculator()
   {
      return jointLimitConstraintCalculator;
   }

   public SingleContactImpulseCalculator getOrCreateEnvironmentContactConstraintCalculator()
   {
      return environmentContactConstraintCalculatorPool.nextAvailable();
   }

   public SingleContactImpulseCalculator getOrCreateSelfContactConstraintCalculator()
   {
      return selfContactConstraintCalculatorPool.nextAvailable();
   }

   public SingleContactImpulseCalculator getOrCreateInterRobotContactConstraintCalculator(ImpulseBasedRobot otherRobot)
   {
      if (otherRobot == null)
         return getOrCreateEnvironmentContactConstraintCalculator();
      if (otherRobot == owner)
         return getOrCreateSelfContactConstraintCalculator();

      YoSingleContactImpulseCalculatorPool calculators = interRobotContactConstraintCalculatorPools.get(otherRobot.getRootBody());

      if (calculators == null)
      {
         calculators = new YoSingleContactImpulseCalculatorPool(8,
                                                                owner.getName() + otherRobot.getName() + "Dual",
                                                                inertialFrame,
                                                                owner.getRootBody(),
                                                                forwardDynamicsCalculator,
                                                                otherRobot.getRootBody(),
                                                                otherRobot.getForwardDynamicsCalculator(),
                                                                interRobotContactCalculatorRegistry);
         interRobotContactConstraintCalculatorPools.put(otherRobot.getRootBody(), calculators);
      }

      return calculators.nextAvailable();
   }

   public RigidBodyTwistProvider getRigidBodyTwistChangeProvider()
   {
      return rigidBodyDeltaTwistProvider;
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

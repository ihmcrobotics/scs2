package us.ihmc.scs2.simulation.physicsEngine.contactPointBased;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator.JointSourceMode;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
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

   /**
    * The joint torques from the robot's controller manager.
    */
   private final YoMatrix tau;

   /**
    * The joint torques imposed by physics simulation, consisting of damping + soft enforcement of joint limits
    */
   private final YoMatrix tauSim;

   /**
    * The resultant joint torques to be used in the forward dynamics calculation (i.e. the calculation for the result of the simulation)
    */
   private final YoMatrix tauTotal;

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

      tau = new YoMatrix("tau", nDoFs, 1, owner.getRegistry());
      tauSim = new YoMatrix("tauSim", nDoFs, 1, owner.getRegistry());
      tauTotal = new YoMatrix("tauTotal", nDoFs, 1, owner.getRegistry());

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

   // TODO: bad name
   public void computeJointSimulationEffects()
   {
      tauSim.zero();  // tauSim is appended to, not overwritten, so it is imperative we zero it here
      computeJointDamping();
      computeJointSoftLimits();
   }

   private void computeJointDamping()
   {
      robotOneDoFJointDampingCalculator.compute(tauSim);
//      robotOneDoFJointDampingCalculator.compute();  // TODO remove eventually
   }

   private void computeJointSoftLimits()
   {
      robotOneDoFJointSoftLimitCalculator.compute(tauSim);
//      robotOneDoFJointSoftLimitCalculator.compute();  // TODO remove eventually
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
      getJointTauFromControllers();
      // TODO note about how by now, joint taus from limits and damping will have been called
      sumJointTauContributions();
      forwardDynamicsCalculator.compute(tauTotal);
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

   private void getJointTauFromControllers()
   {
      for (JointBasics joint : owner.getJointsToConsider())
      {
         JointStateBasics jointOutput = owner.getControllerOutput().getJointOutput(joint);

         if (jointOutput.hasOutputFor(JointStateType.EFFORT))
         {
            int jointIndex = indexProvider.getJointDoFIndices(joint)[0];
            jointOutput.getEffort(jointIndex, tau);
         }
      }
   }

   private void sumJointTauContributions()
   {
      for (int i = 0; i < tau.getNumRows(); i++)
      {
         tauTotal.set(i, 0, tau.get(i, 0) + tauSim.get(i, 0));
      }

   }
}

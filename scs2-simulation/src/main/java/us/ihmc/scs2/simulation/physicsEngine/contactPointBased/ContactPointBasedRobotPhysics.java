package us.ihmc.scs2.simulation.physicsEngine.contactPointBased;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator.JointSourceMode;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.scs2.simulation.RobotJointWrenchCalculator;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.FrameShapePosePredictor;
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

   public void computeJointDamping()
   {
      robotOneDoFJointDampingCalculator.compute();
   }

   public void computeJointSoftLimits()
   {
      robotOneDoFJointSoftLimitCalculator.compute();
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
      forwardDynamicsCalculator.compute();
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
}

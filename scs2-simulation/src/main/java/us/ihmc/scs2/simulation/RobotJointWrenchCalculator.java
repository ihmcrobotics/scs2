package us.ihmc.scs2.simulation;

import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * This class is used to compute the wrenches acting on each joint of a robot.
 * <p>
 * It relies on the {@link ForwardDynamicsCalculator} to compute the wrenches due to the robot's mass and the joint accelerations. It also relies on the
 * {@link RobotPhysicsOutput} to compute the wrenches due to external impulses.
 * </p>
 */
public class RobotJointWrenchCalculator
{
   private final YoRegistry registry;

   private final Map<JointReadOnly, FixedFrameWrenchBasics> jointWrenchMap;
   private final RobotPhysicsOutput robotPhysicsOutput;
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;

   /**
    * Creates a new calculator.
    *
    * @param physicsOutput             the output of the physics engine. It is used to compute the wrenches due to external impulses.
    * @param forwardDynamicsCalculator the calculator used to compute the wrenches due to the robot's mass and the joint accelerations.
    * @param parentRegistry            the registry to which the calculator's variables are registered.
    */
   public RobotJointWrenchCalculator(RobotPhysicsOutput physicsOutput, ForwardDynamicsCalculator forwardDynamicsCalculator, YoRegistry parentRegistry)
   {
      this.robotPhysicsOutput = physicsOutput;
      this.forwardDynamicsCalculator = forwardDynamicsCalculator;
      registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      jointWrenchMap = new LinkedHashMap<>();

      for (JointReadOnly joint : forwardDynamicsCalculator.getInput().getJointsToConsider())
      {
         MovingReferenceFrame successorFrame = joint.getSuccessor().getBodyFixedFrame();
         MovingReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();
         YoFrameVector3D torquePart = new YoFrameVector3D(joint.getName() + "FullTorque", frameAfterJoint, registry);
         YoFrameVector3D forcePart = new YoFrameVector3D(joint.getName() + "FullForce", frameAfterJoint, registry);
         jointWrenchMap.put(joint, new YoFixedFrameWrench(successorFrame, new YoFixedFrameSpatialVector(torquePart, forcePart)));
      }
   }

   /**
    * Updates the wrenches acting on each joint.
    *
    * @param dt the time step used to compute the wrenches due to external impulses.
    */
   public void update(double dt)
   {
      if (jointWrenchMap == null)
         return;

      jointWrenchMap.forEach((joint, wrench) -> wrench.set(forwardDynamicsCalculator.getJointWrench(joint)));

      if (robotPhysicsOutput.getExternalImpulseProvider() != null)
      {
         Map<JointReadOnly, SpatialImpulseReadOnly> externalImpulseMap = new LinkedHashMap<>();
         computeExternalImpulseMap(forwardDynamicsCalculator.getInput().getRootBody().getChildrenJoints(), externalImpulseMap);

         SpatialForce tempWrench = new SpatialForce();

         for (JointReadOnly joint : externalImpulseMap.keySet())
         {
            FixedFrameWrenchBasics jointWrench = jointWrenchMap.get(joint);

            SpatialImpulseReadOnly externalImpulse = externalImpulseMap.get(joint);
            if (externalImpulse == null)
               continue;

            tempWrench.setIncludingFrame(externalImpulse);
            tempWrench.changeFrame(jointWrench.getReferenceFrame());
            tempWrench.scale(dt);
            jointWrench.add(tempWrench);
         }
      }
   }

   /**
    * Gets the impulse acting on the given joints and their subtrees.
    *
    * @param joints                   the joints to get the impulse for.
    * @param externalImpulseMapToPack the map in which the impulse is stored.
    */
   private void computeExternalImpulseMap(Collection<? extends JointReadOnly> joints, Map<JointReadOnly, SpatialImpulseReadOnly> externalImpulseMapToPack)
   {
      for (JointReadOnly joint : joints)
      {
         computeExternalImpulseMap(joint, externalImpulseMapToPack);
      }
   }

   /**
    * Recursively computes the external impulse acting on each joint.
    *
    * @param joint                    the joint to compute the external impulse for.
    * @param externalImpulseMapToPack the map in which the external impulse is stored.
    */
   private void computeExternalImpulseMap(JointReadOnly joint, Map<JointReadOnly, SpatialImpulseReadOnly> externalImpulseMapToPack)
   {
      for (JointReadOnly childJoint : joint.getPredecessor().getChildrenJoints())
      {
         computeExternalImpulseMap(childJoint, externalImpulseMapToPack);
      }

      SpatialImpulse subtreeImpulse = null;
      SpatialImpulseReadOnly externalImpulse = robotPhysicsOutput.getExternalImpulseProvider().apply(joint.getSuccessor());

      if (externalImpulse != null)
      {
         subtreeImpulse = new SpatialImpulse(externalImpulse);
         subtreeImpulse.changeFrame(joint.getFrameAfterJoint());
      }

      SpatialImpulse tempImpulse = new SpatialImpulse();

      for (JointReadOnly childJoint : joint.getSuccessor().getChildrenJoints())
      {
         SpatialImpulseReadOnly childImpulse = externalImpulseMapToPack.get(childJoint);

         if (childImpulse != null)
         {
            if (subtreeImpulse == null)
            {
               subtreeImpulse = new SpatialImpulse(childImpulse);
               subtreeImpulse.changeFrame(joint.getFrameAfterJoint());
            }
            else
            {
               tempImpulse.setIncludingFrame(childImpulse);
               tempImpulse.changeFrame(subtreeImpulse.getReferenceFrame());
               subtreeImpulse.add(tempImpulse);
            }
         }
      }
   }
}

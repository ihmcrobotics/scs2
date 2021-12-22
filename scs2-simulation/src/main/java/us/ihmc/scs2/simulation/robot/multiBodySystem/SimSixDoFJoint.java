package us.ihmc.scs2.simulation.robot.multiBodySystem;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoSixDoFJoint;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.simulation.robot.SimJointAuxiliaryData;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimSixDoFJoint extends YoSixDoFJoint implements SimJointBasics, SimFloatingJointBasics
{
   private final YoRegistry registry;
   private final SimJointAuxiliaryData auxiliaryData;
   private final FixedFrameTwistBasics jointDeltaTwist;

   public SimSixDoFJoint(SixDoFJointDefinition definition, SimRigidBodyBasics predecessor)
   {
      this(definition.getName(), predecessor, definition.getTransformToParent());
   }

   public SimSixDoFJoint(String name, SimRigidBodyBasics predecessor)
   {
      this(name, predecessor, null);
   }

   public SimSixDoFJoint(String name, SimRigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, predecessor, transformToParent, predecessor.getRegistry());
      registry = predecessor.getRegistry();
      auxiliaryData = new SimJointAuxiliaryData(this);
      jointDeltaTwist = new YoFixedFrameTwist(name + "DeltaTwist", afterJointFrame, beforeJointFrame, afterJointFrame, registry);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public SimJointAuxiliaryData getAuxialiryData()
   {
      return auxiliaryData;
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      if (successor instanceof SimRigidBodyBasics)
         super.setSuccessor(successor);
      else
         throw new IllegalArgumentException("Can only set a " + SimRigidBodyBasics.class.getSimpleName() + " as successor of a " + getClass().getSimpleName());
   }

   @Override
   public SimRigidBodyBasics getPredecessor()
   {
      return (SimRigidBodyBasics) super.getPredecessor();
   }

   @Override
   public SimRigidBodyBasics getSuccessor()
   {
      return (SimRigidBodyBasics) super.getSuccessor();
   }

   @Override
   public FixedFrameTwistBasics getJointDeltaTwist()
   {
      return jointDeltaTwist;
   }

   @Override
   public int getJointDeltaVelocity(int rowStart, DMatrix matrixToPack)
   {
      getJointTwist().get(rowStart, matrixToPack);
      return rowStart + getDegreesOfFreedom();
   }

   @Override
   public void setJointDeltaTwist(JointReadOnly other)
   {
      setJointDeltaTwist(MecanoTools.checkTypeAndCast(other, SimSixDoFJoint.class));
   }

   public void setJointDeltaTwist(SimSixDoFJoint other)
   {
      // Cast to frameless object so we don't perform frame checks which would automatically fail.
      Vector3DReadOnly otherAngularDeltaVelocity = other.getJointDeltaTwist().getAngularPart();
      Vector3DReadOnly otherLinearDeltaVelocity = other.getJointDeltaTwist().getLinearPart();

      setJointAngularDeltaVelocity(otherAngularDeltaVelocity);
      setJointLinearDeltaVelocity(otherLinearDeltaVelocity);
   }

   @Override
   public int setJointDeltaVelocity(int rowStart, DMatrix matrix)
   {
      getJointDeltaTwist().set(rowStart, matrix);
      return rowStart + getDegreesOfFreedom();
   }
}

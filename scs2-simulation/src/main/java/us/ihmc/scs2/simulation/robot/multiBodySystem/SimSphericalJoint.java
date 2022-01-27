package us.ihmc.scs2.simulation.robot.multiBodySystem;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoSphericalJoint;
import us.ihmc.scs2.definition.robot.SphericalJointDefinition;
import us.ihmc.scs2.simulation.robot.SimJointAuxiliaryData;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimSphericalJoint extends YoSphericalJoint implements SimJointBasics
{
   private final YoRegistry registry;
   private final SimJointAuxiliaryData auxiliaryData;
   private final YoFrameVector3D jointAngularDeltaVelocity;
   private final TwistReadOnly jointDeltaTwist;

   public SimSphericalJoint(SphericalJointDefinition definition, SimRigidBodyBasics predecessor)
   {
      this(definition.getName(), predecessor, definition.getTransformToParent());
   }

   public SimSphericalJoint(String name, SimRigidBodyBasics predecessor, Tuple3DReadOnly jointOffset)
   {
      this(name, predecessor, new RigidBodyTransform(new Quaternion(), jointOffset));
   }

   public SimSphericalJoint(String name, SimRigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, predecessor, transformToParent, predecessor.getRegistry());

      registry = predecessor.getRegistry();
      auxiliaryData = new SimJointAuxiliaryData(this);

      String varName = !name.isEmpty() ? "_" + name + "_" : "_";
      jointAngularDeltaVelocity = new YoFrameVector3D("qd_delta" + varName + "w", afterJointFrame, registry);
      jointDeltaTwist = MecanoFactories.newTwistReadOnly(afterJointFrame, beforeJointFrame, jointAngularDeltaVelocity, new FrameVector3D(afterJointFrame));
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

   public FixedFrameVector3DBasics getJointAngularDeltaVelocity()
   {
      return jointAngularDeltaVelocity;
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
   public TwistReadOnly getJointDeltaTwist()
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
   public void setJointDeltaTwistToZero()
   {
      jointAngularDeltaVelocity.setToZero();
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
      setJointAngularDeltaVelocity(otherAngularDeltaVelocity);
   }

   @Override
   public int setJointDeltaVelocity(int rowStart, DMatrix matrix)
   {
      jointAngularDeltaVelocity.set(rowStart, matrix);
      return rowStart + getDegreesOfFreedom();
   }

   @Override
   public void setJointAngularDeltaVelocity(Vector3DReadOnly jointAngularDeltaVelocity)
   {
      this.jointAngularDeltaVelocity.set(jointAngularDeltaVelocity);
   }

   @Override
   public void setJointLinearDeltaVelocity(Vector3DReadOnly jointLinearDeltaVelocity)
   {
   }
}

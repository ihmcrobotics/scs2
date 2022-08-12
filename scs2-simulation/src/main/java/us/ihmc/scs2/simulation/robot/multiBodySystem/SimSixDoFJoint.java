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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimSixDoFJoint extends YoSixDoFJoint implements SimJointBasics, SimFloatingJointBasics
{
   private final YoRegistry registry;
   private final SimJointAuxiliaryData auxiliaryData;
   private final FixedFrameTwistBasics jointDeltaTwist;
   private final YoBoolean isPinned;

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

      String varName = !name.isEmpty() ? "_" + name + "_" : "_";
      jointDeltaTwist = new YoFixedFrameTwist(afterJointFrame,
                                              beforeJointFrame,
                                              new YoFrameVector3D("qd_delta" + varName + "w", afterJointFrame, registry),
                                              new YoFrameVector3D("qd_delta" + varName, afterJointFrame, registry));
      isPinned = new YoBoolean("is" + varName + "pinned", registry);
      getJointPose().attachVariableChangedListener(v ->
      {
         if (!Double.isFinite(((YoDouble) v).getValue()))
            throw new IllegalStateException("Invalid joint configuration: " + getJointPose());
      });
      getJointTwist().getAngularPart().attachVariableChangedListener(v ->
      {
         if (!Double.isFinite(((YoDouble) v).getValue()))
            throw new IllegalStateException("Invalid joint twist: " + getJointTwist());
      });
      getJointTwist().getLinearPart().attachVariableChangedListener(v ->
      {
         if (!Double.isFinite(((YoDouble) v).getValue()))
            throw new IllegalStateException("Invalid joint twist: " + getJointTwist());
      });
      getJointAcceleration().getAngularPart().attachVariableChangedListener(v ->
      {
         if (!Double.isFinite(((YoDouble) v).getValue()))
            throw new IllegalStateException("Invalid joint acceleration: " + getJointAcceleration());
      });
      getJointAcceleration().getLinearPart().attachVariableChangedListener(v ->
      {
         if (!Double.isFinite(((YoDouble) v).getValue()))
            throw new IllegalStateException("Invalid joint acceleration: " + getJointAcceleration());
      });
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public SimJointAuxiliaryData getAuxiliaryData()
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

   @Override
   public void setPinned(boolean isPinned)
   {
      this.isPinned.set(isPinned);
   }

   @Override
   public boolean isPinned()
   {
      return isPinned.getValue();
   }
}

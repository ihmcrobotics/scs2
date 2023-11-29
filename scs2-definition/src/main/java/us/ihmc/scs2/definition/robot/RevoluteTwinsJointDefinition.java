package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;

import java.util.Objects;

public class RevoluteTwinsJointDefinition extends OneDoFJointDefinition
{
   private String jointNameA;
   private String jointNameB;
   private YawPitchRollTransformDefinition transformAToPredecessor = new YawPitchRollTransformDefinition();
   private YawPitchRollTransformDefinition transformBToA = new YawPitchRollTransformDefinition();
   private RigidBodyDefinition bodyAB = new RigidBodyDefinition();

   private int actuatedJointIndex;
   private double constraintRatio;
   private double constraintOffset;

   public RevoluteTwinsJointDefinition()
   {
   }

   public RevoluteTwinsJointDefinition(String name)
   {
      super(name);
      bodyAB.setName(name + "_AB");
   }

   public RevoluteTwinsJointDefinition(String name, Vector3DReadOnly axis)
   {
      this(name);
      setAxis(axis);
   }

   public void setJointNameA(String jointNameA)
   {
      this.jointNameA = jointNameA;
   }

   public void setJointNameB(String jointNameB)
   {
      this.jointNameB = jointNameB;
   }

   public void setJointNames(String jointNameA, String jointNameB)
   {
      this.jointNameA = jointNameA;
      this.jointNameB = jointNameB;
   }

   public void setTransformAToPredecessor(YawPitchRollTransformDefinition transformAToPredecessor)
   {
      this.transformAToPredecessor = transformAToPredecessor;
   }

   public void setTransformAToPredecessor(RigidBodyTransformReadOnly transformAToPredecessor)
   {
      this.transformAToPredecessor.set(transformAToPredecessor);
   }

   public void setTransformBToA(YawPitchRollTransformDefinition transformBToA)
   {
      this.transformBToA = transformBToA;
   }

   public void setTransformBToA(RigidBodyTransformReadOnly transformBToA)
   {
      this.transformBToA.set(transformBToA);
   }

   public void setJointTransforms(YawPitchRollTransformDefinition transformAToPredecessor, YawPitchRollTransformDefinition transformBToA)
   {
      this.transformAToPredecessor = transformAToPredecessor;
      this.transformBToA = transformBToA;
   }

   public void setJointTransforms(RigidBodyTransformReadOnly transformAToPredecessor, RigidBodyTransformReadOnly transformBToA)
   {
      this.transformAToPredecessor.set(transformAToPredecessor);
      this.transformBToA.set(transformBToA);
   }

   public void setBodyAB(RigidBodyDefinition bodyAB)
   {
      this.bodyAB = bodyAB;
   }

   public void setActuatedJointIndex(int actuatedJointIndex)
   {
      this.actuatedJointIndex = actuatedJointIndex;
   }

   public void setConstraintRatio(double constraintRatio)
   {
      this.constraintRatio = constraintRatio;
   }

   public void setConstraintOffset(double constraintOffset)
   {
      this.constraintOffset = constraintOffset;
   }

   public String getJointNameA()
   {
      return jointNameA;
   }

   public String getJointNameB()
   {
      return jointNameB;
   }

   public YawPitchRollTransformDefinition getTransformAToPredecessor()
   {
      return transformAToPredecessor;
   }

   public YawPitchRollTransformDefinition getTransformBToA()
   {
      return transformBToA;
   }

   public RigidBodyDefinition getBodyAB()
   {
      return bodyAB;
   }

   public int getActuatedJointIndex()
   {
      return actuatedJointIndex;
   }

   public double getConstraintRatio()
   {
      return constraintRatio;
   }

   public double getConstraintOffset()
   {
      return constraintOffset;
   }

   @Override
   public RevoluteTwinsJoint toJoint(RigidBodyBasics predecessor)
   {
      return new RevoluteTwinsJoint(getName(),
                                    predecessor,
                                    jointNameA,
                                    jointNameB,
                                    bodyAB.getName(),
                                    transformAToPredecessor,
                                    transformBToA,
                                    bodyAB.getMomentOfInertia(),
                                    bodyAB.getMass(),
                                    bodyAB.getInertiaPose(),
                                    actuatedJointIndex,
                                    constraintRatio,
                                    constraintOffset,
                                    getAxis());
   }

   @Override
   public RevoluteTwinsJointDefinition copy()
   {
      RevoluteTwinsJointDefinition clone = new RevoluteTwinsJointDefinition(getName(), getAxis());
      clone.jointNameA = jointNameA;
      clone.jointNameB = jointNameB;
      clone.transformAToPredecessor.set(transformAToPredecessor);
      clone.transformBToA.set(transformBToA);
      clone.bodyAB = bodyAB.copy();
      clone.actuatedJointIndex = actuatedJointIndex;
      clone.constraintRatio = constraintRatio;
      clone.constraintOffset = constraintOffset;
      return clone;
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, jointNameA);
      bits = EuclidHashCodeTools.addToHashCode(bits, jointNameB);
      bits = EuclidHashCodeTools.addToHashCode(bits, transformAToPredecessor);
      bits = EuclidHashCodeTools.addToHashCode(bits, transformBToA);
      bits = EuclidHashCodeTools.addToHashCode(bits, bodyAB);
      bits = EuclidHashCodeTools.addToHashCode(bits, actuatedJointIndex);
      bits = EuclidHashCodeTools.addToHashCode(bits, constraintRatio);
      bits = EuclidHashCodeTools.addToHashCode(bits, constraintOffset);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      RevoluteTwinsJointDefinition other = (RevoluteTwinsJointDefinition) object;

      if (!Objects.equals(jointNameA, other.jointNameA))
         return false;
      if (!Objects.equals(jointNameB, other.jointNameB))
         return false;
      if (!Objects.equals(transformAToPredecessor, other.transformAToPredecessor))
         return false;
      if (!Objects.equals(transformBToA, other.transformBToA))
         return false;
      if (!Objects.equals(bodyAB, other.bodyAB))
         return false;
      if (actuatedJointIndex != other.actuatedJointIndex)
         return false;
      if (!EuclidCoreTools.equals(constraintRatio, other.constraintRatio))
         return false;
      if (!EuclidCoreTools.equals(constraintOffset, other.constraintOffset))
         return false;
      return true;
   }
}

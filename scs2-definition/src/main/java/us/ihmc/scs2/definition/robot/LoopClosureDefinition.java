package us.ihmc.scs2.definition.robot;

import java.util.Objects;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;

public class LoopClosureDefinition
{
   private YawPitchRollTransformDefinition transformToSuccessorParent = new YawPitchRollTransformDefinition();
   private Vector3D kpSoftConstraint;
   private Vector3D kdSoftConstraint;

   public LoopClosureDefinition()
   {
   }

   public LoopClosureDefinition(LoopClosureDefinition other)
   {
      transformToSuccessorParent.set(other.transformToSuccessorParent);
      kpSoftConstraint = other.kpSoftConstraint == null ? null : new Vector3D(other.kpSoftConstraint);
      kdSoftConstraint = other.kdSoftConstraint == null ? null : new Vector3D(other.kdSoftConstraint);
   }

   public void setTransformToSuccessorParent(YawPitchRollTransformDefinition transformToSuccessorParent)
   {
      this.transformToSuccessorParent = transformToSuccessorParent;
   }

   public void setTransformToSuccessorParent(RigidBodyTransformReadOnly transformToSuccessorParent)
   {
      this.transformToSuccessorParent.set(transformToSuccessorParent);
   }

   public void setOffsetFromSuccessorParent(Tuple3DReadOnly offsetFromSuccessorParent)
   {
      this.transformToSuccessorParent.setTranslationAndIdentityRotation(offsetFromSuccessorParent);
   }

   public YawPitchRollTransformDefinition getTransformToSuccessorParent()
   {
      return transformToSuccessorParent;
   }

   public void setKpSoftConstraint(double kpSoftConstraint)
   {
      setKpSoftConstraint(new Vector3D(kpSoftConstraint, kpSoftConstraint, kpSoftConstraint));
   }

   public void setKpSoftConstraint(Vector3D kpSoftConstraint)
   {
      this.kpSoftConstraint = kpSoftConstraint;
   }

   public Vector3D getKpSoftConstraint()
   {
      return kpSoftConstraint;
   }

   public void setKdSoftConstraint(double kdSoftConstraint)
   {
      setKdSoftConstraint(new Vector3D(kdSoftConstraint, kdSoftConstraint, kdSoftConstraint));
   }

   public void setKdSoftConstraint(Vector3D kdSoftConstraint)
   {
      this.kdSoftConstraint = kdSoftConstraint;
   }

   public Vector3D getKdSoftConstraint()
   {
      return kdSoftConstraint;
   }

   public LoopClosureDefinition copy()
   {
      return new LoopClosureDefinition(this);
   }

   @Override
   public String toString()
   {
      return "transformToSuccessorParent: " + transformToSuccessorParent;
   }

   // TODO Tools for computing the force/moment subspace given a joint. Should live somewhere elese.

   public static Matrix3D jointForceSubSpace(JointDefinition joint)
   {
      if (joint instanceof RevoluteJointDefinition)
         return identityMatrix3D();
      if (joint instanceof PrismaticJointDefinition)
         return matrix3DOrthogonalToVector3D(((PrismaticJointDefinition) joint).getAxis());
      else
         return null;
   }

   public static Matrix3D jointMomentSubSpace(JointDefinition joint)
   {
      if (joint instanceof RevoluteJointDefinition)
         return matrix3DOrthogonalToVector3D(((RevoluteJointDefinition) joint).getAxis());
      if (joint instanceof PrismaticJointDefinition)
         return identityMatrix3D();
      else
         return null;
   }

   public static Matrix3D identityMatrix3D()
   {
      Matrix3D identity = new Matrix3D();
      identity.setIdentity();
      return identity;
   }

   public static Matrix3D matrix3DOrthogonalToVector3D(Vector3DReadOnly vector3D)
   {
      Matrix3D orthogonalMatrix = new Matrix3D();
      matrix3DOrthogonalToVector3D(vector3D, orthogonalMatrix);
      return orthogonalMatrix;
   }

   public static void matrix3DOrthogonalToVector3D(Vector3DReadOnly vector3D, Matrix3DBasics orthogonalMatrixToPack)
   {
      RotationMatrix R = new RotationMatrix();
      EuclidGeometryTools.orientation3DFromZUpToVector3D(vector3D, R);
      orthogonalMatrixToPack.setIdentity();
      orthogonalMatrixToPack.setM22(0.0);
      R.transform(orthogonalMatrixToPack);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, transformToSuccessorParent);
      bits = EuclidHashCodeTools.addToHashCode(bits, kpSoftConstraint);
      bits = EuclidHashCodeTools.addToHashCode(bits, kdSoftConstraint);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      LoopClosureDefinition other = (LoopClosureDefinition) object;

      if (!Objects.equals(transformToSuccessorParent, other.transformToSuccessorParent))
         return false;
      if (!Objects.equals(kpSoftConstraint, other.kpSoftConstraint))
         return false;
      if (!Objects.equals(kdSoftConstraint, other.kdSoftConstraint))
         return false;
      return true;
   }
}

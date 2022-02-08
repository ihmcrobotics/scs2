package us.ihmc.scs2.simulation.robot.state;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.SphericalJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.SphericalJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.SphericalJointStateReadOnly;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSphericalJointState implements SphericalJointStateBasics
{
   private final YoQuaternion orientation;
   private final YoVector3D angularVelocity;
   private final YoVector3D angularAcceleration;
   private final YoVector3D torque;

   private final DMatrixRMaj temp = new DMatrixRMaj(4, 1);

   public YoSphericalJointState(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      if (namePrefix == null)
         namePrefix = "";
      else if (!namePrefix.isEmpty() && !namePrefix.endsWith("_"))
         namePrefix += "_";

      if (nameSuffix == null)
         nameSuffix = "";
      else if (!nameSuffix.isEmpty() && !nameSuffix.startsWith("_"))
         nameSuffix = "_" + nameSuffix;

      orientation = new YoQuaternion(namePrefix + "q_", nameSuffix, registry);
      angularVelocity = new YoVector3D(namePrefix + "qd_w", nameSuffix, registry);
      angularAcceleration = new YoVector3D(namePrefix + "qdd_w", nameSuffix, registry);
      torque = new YoVector3D(namePrefix + "tau_w", nameSuffix, registry);
   }

   @Override
   public void set(JointStateReadOnly jointStateReadOnly)
   {
      if (jointStateReadOnly instanceof SphericalJointStateReadOnly)
      {
         SphericalJointStateBasics.super.set((SphericalJointStateReadOnly) jointStateReadOnly);
      }
      else
      {
         if (jointStateReadOnly.getConfigurationSize() != getConfigurationSize() || jointStateReadOnly.getDegreesOfFreedom() != getDegreesOfFreedom())
            throw new IllegalArgumentException("Dimension mismatch");

         if (jointStateReadOnly.hasOutputFor(JointStateType.CONFIGURATION))
         {
            jointStateReadOnly.getConfiguration(0, temp);
            setConfiguration(0, temp);
         }
         else
         {
            orientation.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.VELOCITY))
         {
            jointStateReadOnly.getVelocity(0, temp);
            setVelocity(0, temp);
         }
         else
         {
            angularVelocity.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.ACCELERATION))
         {
            jointStateReadOnly.getAcceleration(0, temp);
            setAcceleration(0, temp);
         }
         else
         {
            angularAcceleration.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.EFFORT))
         {
            jointStateReadOnly.getEffort(0, temp);
            setEffort(0, temp);
         }
         else
         {
            torque.setToNaN();
         }
      }
   }

   @Override
   public SphericalJointState copy()
   {
      return new SphericalJointState(this);
   }

   @Override
   public YoQuaternion getOrientation()
   {
      return orientation;
   }

   @Override
   public YoVector3D getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public YoVector3D getAngularAcceleration()
   {
      return angularAcceleration;
   }

   @Override
   public YoVector3D getTorque()
   {
      return torque;
   }

   @Override
   public String toString()
   {
      String ret = "Spherical joint state";
      if (hasOutputFor(JointStateType.CONFIGURATION))
         ret += ", orientaiton: " + orientation.toStringAsYawPitchRoll();
      if (hasOutputFor(JointStateType.VELOCITY))
         ret += ", angular velocity: " + angularVelocity;
      if (hasOutputFor(JointStateType.ACCELERATION))
         ret += ", angular acceleration: " + angularAcceleration;
      if (hasOutputFor(JointStateType.EFFORT))
         ret += ", torque: " + torque;
      return ret;
   }
}

package us.ihmc.scs2.simulation.robot.state;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateReadOnly;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSixDoFJointState implements SixDoFJointStateBasics
{
   private final YoQuaternion orientation;
   private final YoPoint3D position;
   private final YoVector3D angularVelocity;
   private final YoVector3D linearVelocity;
   private final YoVector3D angularAcceleration;
   private final YoVector3D linearAcceleration;
   private final YoVector3D torque;
   private final YoVector3D force;

   private final DMatrixRMaj temp = new DMatrixRMaj(7, 1);

   public YoSixDoFJointState(String namePrefix, String nameSuffix, YoRegistry registry)
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
      position = new YoPoint3D(namePrefix + "q_", nameSuffix, registry);
      angularVelocity = new YoVector3D(namePrefix + "qd_w", nameSuffix, registry);
      linearVelocity = new YoVector3D(namePrefix + "qd_", nameSuffix, registry);
      angularAcceleration = new YoVector3D(namePrefix + "qdd_w", nameSuffix, registry);
      linearAcceleration = new YoVector3D(namePrefix + "qdd_", nameSuffix, registry);
      torque = new YoVector3D(namePrefix + "tau_w", nameSuffix, registry);
      force = new YoVector3D(namePrefix + "tau_", nameSuffix, registry);
   }

   @Override
   public void set(JointStateReadOnly jointStateReadOnly)
   {
      if (jointStateReadOnly instanceof SixDoFJointStateReadOnly)
      {
         SixDoFJointStateBasics.super.set((SixDoFJointStateReadOnly) jointStateReadOnly);
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
            position.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.VELOCITY))
         {
            jointStateReadOnly.getVelocity(0, temp);
            setVelocity(0, temp);
         }
         else
         {
            angularVelocity.setToNaN();
            linearVelocity.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.ACCELERATION))
         {
            jointStateReadOnly.getAcceleration(0, temp);
            setAcceleration(0, temp);
         }
         else
         {
            angularAcceleration.setToNaN();
            linearAcceleration.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.EFFORT))
         {
            jointStateReadOnly.getEffort(0, temp);
            setEffort(0, temp);
         }
         else
         {
            torque.setToNaN();
            force.setToNaN();
         }
      }
   }

   @Override
   public SixDoFJointState copy()
   {
      return new SixDoFJointState(this);
   }

   @Override
   public YoQuaternion getOrientation()
   {
      return orientation;
   }

   @Override
   public YoPoint3D getPosition()
   {
      return position;
   }

   @Override
   public YoVector3D getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public YoVector3D getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   public YoVector3D getAngularAcceleration()
   {
      return angularAcceleration;
   }

   @Override
   public YoVector3D getLinearAcceleration()
   {
      return linearAcceleration;
   }

   @Override
   public YoVector3D getTorque()
   {
      return torque;
   }

   @Override
   public YoVector3D getForce()
   {
      return force;
   }

   @Override
   public String toString()
   {
      String ret = "6-DoF joint state";
      if (hasOutputFor(JointStateType.CONFIGURATION))
         ret += ", orientation: " + orientation.toStringAsYawPitchRoll() + ", position: " + position;
      if (hasOutputFor(JointStateType.VELOCITY))
         ret += ", angular velocity: " + angularVelocity + ", linear velocity: " + linearVelocity;
      if (hasOutputFor(JointStateType.ACCELERATION))
         ret += ", angular acceleration: " + angularAcceleration + ", linear acceleration: " + linearAcceleration;
      if (hasOutputFor(JointStateType.EFFORT))
         ret += ", torqe: " + torque + ", force: " + force;
      return ret;
   }
}

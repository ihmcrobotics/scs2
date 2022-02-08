package us.ihmc.scs2.simulation.robot.state;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoOneDoFJointState implements OneDoFJointStateBasics
{
   private final YoDouble configuration;
   private final YoDouble velocity;
   private final YoDouble acceleration;
   private final YoDouble effort;

   private final DMatrixRMaj temp = new DMatrixRMaj(1, 1);

   public YoOneDoFJointState(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      if (namePrefix == null)
         namePrefix = "";
      else if (!namePrefix.isEmpty() && !namePrefix.endsWith("_"))
         namePrefix += "_";

      if (nameSuffix == null)
         nameSuffix = "";
      else if (!nameSuffix.isEmpty() && !nameSuffix.startsWith("_"))
         nameSuffix = "_" + nameSuffix;

      configuration = new YoDouble(namePrefix + "q" + nameSuffix, registry);
      velocity = new YoDouble(namePrefix + "qd" + nameSuffix, registry);
      acceleration = new YoDouble(namePrefix + "qdd" + nameSuffix, registry);
      effort = new YoDouble(namePrefix + "tau" + nameSuffix, registry);
      clear();
   }

   @Override
   public void set(JointStateReadOnly jointStateReadOnly)
   {
      if (jointStateReadOnly instanceof OneDoFJointStateReadOnly)
      {
         OneDoFJointStateBasics.super.set((OneDoFJointStateReadOnly) jointStateReadOnly);
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
            configuration.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.VELOCITY))
         {
            jointStateReadOnly.getVelocity(0, temp);
            setVelocity(0, temp);
         }
         else
         {
            velocity.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.ACCELERATION))
         {
            jointStateReadOnly.getAcceleration(0, temp);
            setAcceleration(0, temp);
         }
         else
         {
            acceleration.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.EFFORT))
         {
            jointStateReadOnly.getEffort(0, temp);
            setEffort(0, temp);
         }
         else
         {
            effort.setToNaN();
         }
      }
   }

   @Override
   public OneDoFJointState copy()
   {
      return new OneDoFJointState(this);
   }

   @Override
   public double getConfiguration()
   {
      return configuration.getValue();
   }

   @Override
   public double getVelocity()
   {
      return velocity.getValue();
   }

   @Override
   public double getAcceleration()
   {
      return acceleration.getValue();
   }

   @Override
   public double getEffort()
   {
      return effort.getValue();
   }

   @Override
   public void setConfiguration(double q)
   {
      configuration.set(q);
   }

   @Override
   public void setVelocity(double qd)
   {
      velocity.set(qd);
   }

   @Override
   public void setAcceleration(double qdd)
   {
      acceleration.set(qdd);
   }

   @Override
   public void setEffort(double tau)
   {
      effort.set(tau);
   }

   @Override
   public String toString()
   {
      String ret = "1-DoF joint state";
      if (hasOutputFor(JointStateType.CONFIGURATION))
         ret += ", q: " + configuration;
      if (hasOutputFor(JointStateType.VELOCITY))
         ret += ", qd: " + velocity;
      if (hasOutputFor(JointStateType.ACCELERATION))
         ret += ", qdd: " + acceleration;
      if (hasOutputFor(JointStateType.EFFORT))
         ret += ", tau: " + effort;
      return ret;
   }
}

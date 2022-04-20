package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoBulletMultiBodyParameters
{
   private YoBoolean fixedBase;
   private YoBoolean canSleep;
   private YoBoolean hasSelfCollision;
   private YoBoolean useGyroTerm;
   private YoBoolean useGlobalVelocities;
   private YoBoolean useRK4Intergration;
   private YoDouble linearDamping;
   private YoDouble angularDamping;
   private YoDouble jointFriction;
   private YoDouble maxAppliedImpulse;
   private YoDouble maxCoordinateVelocity;

   public YoBulletMultiBodyParameters(String prefix, YoRegistry registry)
   {
      String bulletRobotIsFixedBase;
      String bulletRobotCanSleep;
      String bulletRobotHasSelfCollision;
      String bulletRobotUseGyroTerm;
      String bulletRobotUseGlobalVelocities;
      String bulletRobotAngularDamping;
      String bulletRobotLinearDamping;
      String bulletRobotJointFriction;
      String bulletRobotMaxAppliedImpulse;
      String bulletRobotMaxCoordinateVelocity;
      String bulletRobotUseRK4Intergration;

      if (prefix == null || prefix.isEmpty())
      {
         bulletRobotIsFixedBase = "IsFixedBase";
         bulletRobotCanSleep = "CanSleep";
         bulletRobotHasSelfCollision = "HasSelfCollision";
         bulletRobotUseGyroTerm = "UseGyroTerm";
         bulletRobotUseGlobalVelocities = "UseGlobalVelocities";
         bulletRobotUseRK4Intergration = "UseGlobalRK4Integation";
         bulletRobotAngularDamping = "AngularDamping";
         bulletRobotLinearDamping = "LinearDamping";
         bulletRobotJointFriction = "JointFriction";
         bulletRobotMaxAppliedImpulse = "MaxAppliedImpulse";
         bulletRobotMaxCoordinateVelocity = "MaxCoordinateVelocity";
      }
      else
      {
         bulletRobotIsFixedBase = prefix + "IsFixedBase";
         bulletRobotCanSleep = prefix + "CanSleep";
         bulletRobotHasSelfCollision = prefix + "HasSelfCollision";
         bulletRobotUseGyroTerm = prefix + "UseGyroTerm";
         bulletRobotUseGlobalVelocities = prefix + "UseGlobalVelocities";
         bulletRobotUseRK4Intergration = prefix + "UseGlobalRK4Integation";
         bulletRobotAngularDamping = prefix + "AngularDamping";
         bulletRobotLinearDamping = prefix + "LinearDamping";
         bulletRobotJointFriction = prefix + "JointFriction";
         bulletRobotMaxAppliedImpulse = prefix + "MaxAppliedImpulse";
         bulletRobotMaxCoordinateVelocity = prefix + "MaxCoordinateVelocity";
      }

      fixedBase = new YoBoolean(bulletRobotIsFixedBase, registry);
      canSleep = new YoBoolean(bulletRobotCanSleep, registry);
      hasSelfCollision = new YoBoolean(bulletRobotHasSelfCollision, registry);
      useGyroTerm = new YoBoolean(bulletRobotUseGyroTerm, registry);
      useGlobalVelocities = new YoBoolean(bulletRobotUseGlobalVelocities, registry);
      useRK4Intergration = new YoBoolean(bulletRobotUseRK4Intergration, registry);
      angularDamping = new YoDouble(bulletRobotAngularDamping, registry);
      linearDamping = new YoDouble(bulletRobotLinearDamping, registry);
      jointFriction = new YoDouble(bulletRobotJointFriction, registry);
      maxAppliedImpulse = new YoDouble(bulletRobotMaxAppliedImpulse, registry);
      maxCoordinateVelocity = new YoDouble(bulletRobotMaxCoordinateVelocity, registry);
   }
   
   public void set(BulletMultiBodyParameters parameters)
   {
      setFixedBase(parameters.getFixedBase());
      setCanSleep(parameters.getCanSleep());
      setHasSelfCollision(parameters.getHasSelfCollision());
      setUseGyroTerm(parameters.getUseGyroTerm());
      setUseGlobalVelocities(parameters.getUseGlobalVelocities());
      setUseRK4Integration(parameters.getUseRK4Integration());
      setAngularDamping(parameters.getAngularDamping());
      setLinearDamping(parameters.getLinearDamping());
      setJointFriction(parameters.getJointFriction());
      setMaxAppliedImpulse(parameters.getMaxAppliedImpulse());
      setMaxCoordinateVelocity(parameters.getMaxCoordinateVelocity());
   }

   public void setFixedBase(boolean fixedBase)
   {
      this.fixedBase.set(fixedBase);
   }

   public void setCanSleep(boolean canSleep)
   {
      this.canSleep.set(canSleep);
   }

   public void setHasSelfCollision(boolean hasSelfCollision)
   {
      this.hasSelfCollision.set(hasSelfCollision);
   }

   public void setUseGyroTerm(boolean useGyroTerm)
   {
      this.useGyroTerm.set(useGyroTerm);
   }
   
   public void setUseRK4Integration(boolean useRK4Integration)
   {
      this.useRK4Intergration.set(useRK4Integration);
   }
   
   public void setUseGlobalVelocities(boolean useGlobalVelocities)
   {
      this.useGlobalVelocities.set(useGlobalVelocities);
   }
   
   public void setLinearDamping(double linearDamping)
   {
      this.linearDamping.set(linearDamping);
   }

   public void setAngularDamping(double angularDamping)
   {
      this.angularDamping.set(angularDamping);
   }  
   
   public void setJointFriction(double jointFriction)
   {
      this.jointFriction.set(jointFriction);
   }
   
   public void setMaxAppliedImpulse(double maxAppliedImpulse)
   {
      this.maxAppliedImpulse.set(maxAppliedImpulse);
   }
   
   public void setMaxCoordinateVelocity(double maxCoordinateVelocity)
   {
      this.maxCoordinateVelocity.set(maxCoordinateVelocity);
   }
   
   public boolean getFixedBase()
   {
      return fixedBase.getValue();
   }

   public boolean getCanSleep()
   {
      return canSleep.getValue();
   }

   public boolean getHasSelfCollision()
   {
      return hasSelfCollision.getValue();
   }
   
   public boolean getUseGyroTerm()
   {
      return useGyroTerm.getValue();
   }
   
   public boolean getUseGlobalVelocities()
   {
      return useGlobalVelocities.getValue();
   }
   
   public boolean getUseRK4Integration()
   {
      return useRK4Intergration.getValue();
   }

   public double getLinearDamping()
   {
      return linearDamping.getValue();
   }
   
   public double getAngularDamping()
   {
      return angularDamping.getValue();
   }   
   
   public double getJointFriction()
   {
      return jointFriction.getValue();
   } 
   
   public double getMaxAppliedImpulse()
   {
      return maxAppliedImpulse.getValue();
   }

   public double getMaxCoordinateVelocity()
   {
      return maxCoordinateVelocity.getValue();
   }
}

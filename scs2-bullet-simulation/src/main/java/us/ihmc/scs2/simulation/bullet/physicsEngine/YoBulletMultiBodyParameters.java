package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoBulletMultiBodyParameters
{
   private YoBoolean fixedBase;
   private YoBoolean canSleep;
   private YoBoolean hasSelfCollision;
   private YoDouble linearDamping;
   private YoDouble angularDamping;
   private YoDouble jointFriction;


   public YoBulletMultiBodyParameters(String prefix, YoRegistry registry)
   {

      String bulletRobotIsFixedBase;
      String bulletRobotCanSleep;
      String bulletRobotHasSelfCollision;
      String bulletRobotAngularDamping;
      String bulletRobotLinearDamping;
      String bulletRobotLointFriction;

      if (prefix == null || prefix.isEmpty())
      {
         bulletRobotIsFixedBase = "bulletRobotIsFixedBase";
         bulletRobotCanSleep = "bulletRobotCanSleep";
         bulletRobotHasSelfCollision = "bulletRobotHasSelfCollision";
         bulletRobotAngularDamping = "bulletRobotAngularDamping";
         bulletRobotLinearDamping = "bulletRobotLinearDamping";
         bulletRobotLointFriction = "bulletRobotLointFriction";
      }
      else
      {
         bulletRobotIsFixedBase = prefix + "bulletRobotIsFixedBase";
         bulletRobotCanSleep = prefix + "bulletRobotCanSleep";
         bulletRobotHasSelfCollision = prefix + "bulletRobotHasSelfCollision";
         bulletRobotAngularDamping = prefix + "bulletRobotAngularDamping";
         bulletRobotLinearDamping = prefix + "bulletRobotLinearDamping";
         bulletRobotLointFriction = prefix + "bulletRobotLointFriction";
      }

      fixedBase = new YoBoolean(bulletRobotIsFixedBase, registry);
      canSleep = new YoBoolean(bulletRobotCanSleep, registry);
      hasSelfCollision = new YoBoolean(bulletRobotHasSelfCollision, registry);
      angularDamping = new YoDouble(bulletRobotAngularDamping, registry);
      linearDamping = new YoDouble(bulletRobotLinearDamping, registry);
      jointFriction = new YoDouble(bulletRobotLointFriction, registry);
   }
   
   public void set(BulletMultiBodyParameters parameters)
   {
      setFixedBase(parameters.getFixedBase());
      setCanSleep(parameters.getCanSleep());
      setHasSelfCollision(parameters.getHasSelfCollision());
      setAngularDamping(parameters.getAngularDamping());
      setLinearDamping(parameters.getLinearDamping());
      setJointFriction(parameters.getJointFriction());
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

   public void setLinearDamping(float linearDamping)
   {
      this.linearDamping.set(linearDamping);
   }

   public void setAngularDamping(float angularDamping)
   {
      this.angularDamping.set(angularDamping);
   }  
   
   public void setJointFriction(float jointFriction)
   {
      this.jointFriction.set(jointFriction);
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

   public float getLinearDamping()
   {
      return (float)linearDamping.getValue();
   }
   
   public float getAngularDamping()
   {
      return (float)angularDamping.getValue();
   }   
   
   public float getJointFriction()
   {
      return (float)jointFriction.getValue();
   }   
}

package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoBulletMultiBodyJointParameters
{
   private YoDouble jointFriction;
   private YoDouble jointRestitution;
   private YoDouble jointHitFraction;
   private YoDouble jointRollingFriction;
   private YoDouble jointSpinningFriction;
   private YoDouble jointContactProcessingThreshold;

   public YoBulletMultiBodyJointParameters(String prefix, YoRegistry registry)
   {
      String bulletRobotJointFriction;
      String bulletRobotJointRestitution;
      String bulletRobotJointHitFraction;
      String bulletRobotJointRollingFriction;
      String bulletRobotJointSpinningFriction;
      String bulletRobotJointContactProcessingThreshold;

      if (prefix == null || prefix.isEmpty())
      {
         bulletRobotJointFriction = "Friction";
         bulletRobotJointRestitution = "Restitution";
         bulletRobotJointHitFraction = "HitFraction";
         bulletRobotJointRollingFriction = "RollingFriction";
         bulletRobotJointSpinningFriction = "SpinningFriction";
         bulletRobotJointContactProcessingThreshold = "ContactProcessingThreshold";
      }
      else
      {
         bulletRobotJointFriction = prefix + "Friction";
         bulletRobotJointRestitution = prefix + "Restitution";
         bulletRobotJointHitFraction = prefix + "HitFraction";
         bulletRobotJointRollingFriction = prefix + "RollingFriction";
         bulletRobotJointSpinningFriction = prefix + "SpinningFriction";
         bulletRobotJointContactProcessingThreshold = prefix + "ContactProcessingThreshold";
      }

      jointFriction = new YoDouble(bulletRobotJointFriction, registry);
      jointRestitution = new YoDouble(bulletRobotJointRestitution, registry);
      jointHitFraction = new YoDouble(bulletRobotJointHitFraction, registry);
      jointRollingFriction = new YoDouble(bulletRobotJointRollingFriction, registry);
      jointSpinningFriction = new YoDouble(bulletRobotJointSpinningFriction, registry);
      jointContactProcessingThreshold = new YoDouble(bulletRobotJointContactProcessingThreshold, registry);
   }

   public void set(BulletMultiBodyJointParameters parameters)
   {
      setJointFriction(parameters.getJointFriction());
      setJointRestitution(parameters.getJointRestitution());
      setJointHitFraction(parameters.getJointHitFraction());
      setJointRollingFriction(parameters.getJointRollingFriction());
      setJointSpinningFriction(parameters.getJointSpinningFriction());
      setJointContactProcessingThreshold(parameters.getJointContactProcessingThreshold());
   }

   public void setJointFriction(double jointFriction)
   {
      this.jointFriction.set(jointFriction);
   }

   public void setJointRestitution(double jointRestitution)
   {
      this.jointRestitution.set(jointRestitution);
   }

   public void setJointHitFraction(double jointHitFraction)
   {
      this.jointHitFraction.set(jointHitFraction);
   }

   public void setJointRollingFriction(double jointRollingFriction)
   {
      this.jointRollingFriction.set(jointRollingFriction);
   }

   public void setJointSpinningFriction(double jointSpinningFriction)
   {
      this.jointSpinningFriction.set(jointSpinningFriction);
   }

   public void setJointContactProcessingThreshold(double jointContactProcessingThreshold)
   {
      this.jointContactProcessingThreshold.set(jointContactProcessingThreshold);
   }

   public double getJointFriction()
   {
      return jointFriction.getValue();
   }

   public double getJointRestitution()
   {
      return jointRestitution.getValue();
   }

   public double getJointHitFraction()
   {
      return jointHitFraction.getValue();
   }

   public double getJointRollingFriction()
   {
      return jointRollingFriction.getValue();
   }

   public double getJointSpinningFriction()
   {
      return jointSpinningFriction.getValue();
   }

   public double getJointContactProcessingThreshold()
   {
      return jointContactProcessingThreshold.getValue();
   }
}

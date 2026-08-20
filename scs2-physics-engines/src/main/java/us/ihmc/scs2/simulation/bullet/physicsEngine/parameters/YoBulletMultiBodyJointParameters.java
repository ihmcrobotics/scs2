package us.ihmc.scs2.simulation.bullet.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoBulletMultiBodyJointParameters
{
   private boolean updateGlobalMultiBodyJointParameters;
   private final YoBoolean jointDisableParentCollision;
   private final YoDouble jointFriction;
   private final YoDouble jointRestitution;
   private final YoDouble jointHitFraction;
   private final YoDouble jointRollingFriction;
   private final YoDouble jointSpinningFriction;
   private final YoDouble jointContactProcessingThreshold;

   public YoBulletMultiBodyJointParameters(String prefix, YoRegistry registry)
   {
      String bulletRobotJointDisableParentCollision;
      String bulletRobotJointFriction;
      String bulletRobotJointRestitution;
      String bulletRobotJointHitFraction;
      String bulletRobotJointRollingFriction;
      String bulletRobotJointSpinningFriction;
      String bulletRobotJointContactProcessingThreshold;

      if (prefix == null || prefix.isEmpty())
      {
         bulletRobotJointDisableParentCollision = "DisableParentCollision";
         bulletRobotJointFriction = "Friction";
         bulletRobotJointRestitution = "Restitution";
         bulletRobotJointHitFraction = "HitFraction";
         bulletRobotJointRollingFriction = "RollingFriction";
         bulletRobotJointSpinningFriction = "SpinningFriction";
         bulletRobotJointContactProcessingThreshold = "ContactProcessingThreshold";
      }
      else
      {
         bulletRobotJointDisableParentCollision = prefix + "DisableParentCollision";
         bulletRobotJointFriction = prefix + "Friction";
         bulletRobotJointRestitution = prefix + "Restitution";
         bulletRobotJointHitFraction = prefix + "HitFraction";
         bulletRobotJointRollingFriction = prefix + "RollingFriction";
         bulletRobotJointSpinningFriction = prefix + "SpinningFriction";
         bulletRobotJointContactProcessingThreshold = prefix + "ContactProcessingThreshold";
      }

      jointDisableParentCollision = new YoBoolean(bulletRobotJointDisableParentCollision, registry);
      jointFriction = new YoDouble(bulletRobotJointFriction, registry);
      jointRestitution = new YoDouble(bulletRobotJointRestitution, registry);
      jointHitFraction = new YoDouble(bulletRobotJointHitFraction, registry);
      jointRollingFriction = new YoDouble(bulletRobotJointRollingFriction, registry);
      jointSpinningFriction = new YoDouble(bulletRobotJointSpinningFriction, registry);
      jointContactProcessingThreshold = new YoDouble(bulletRobotJointContactProcessingThreshold, registry);
      updateGlobalMultiBodyJointParameters = false;

      jointDisableParentCollision.addListener(v -> updateGlobalMultiBodyJointParameters = true);
      jointFriction.addListener(v -> updateGlobalMultiBodyJointParameters = true);
      jointRestitution.addListener(v -> updateGlobalMultiBodyJointParameters = true);
      jointHitFraction.addListener(v -> updateGlobalMultiBodyJointParameters = true);
      jointRollingFriction.addListener(v -> updateGlobalMultiBodyJointParameters = true);
      jointSpinningFriction.addListener(v -> updateGlobalMultiBodyJointParameters = true);
      jointContactProcessingThreshold.addListener(v -> updateGlobalMultiBodyJointParameters = true);
   }

   public void set(BulletMultiBodyJointParameters parameters)
   {
      setJointDisableParentCollision(parameters.getJointDisableParentCollision());
      setJointFriction(parameters.getJointFriction());
      setJointRestitution(parameters.getJointRestitution());
      setJointHitFraction(parameters.getJointHitFraction());
      setJointRollingFriction(parameters.getJointRollingFriction());
      setJointSpinningFriction(parameters.getJointSpinningFriction());
      setJointContactProcessingThreshold(parameters.getJointContactProcessingThreshold());
      setUpdateGlobalMultiBodyJointParameters(false);
   }

   public void setUpdateGlobalMultiBodyJointParameters(boolean updateGlobalMultiBodyJointParameters)
   {
      this.updateGlobalMultiBodyJointParameters = updateGlobalMultiBodyJointParameters;
   }

   public void setJointDisableParentCollision(boolean jointDisableParentCollision)
   {
      this.jointDisableParentCollision.set(jointDisableParentCollision);
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

   public boolean getUpdateGlobalMultiBodyJointParameters()
   {
      return updateGlobalMultiBodyJointParameters;
   }

   public boolean getJointDisableParentCollision()
   {
      return jointDisableParentCollision.getValue();
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

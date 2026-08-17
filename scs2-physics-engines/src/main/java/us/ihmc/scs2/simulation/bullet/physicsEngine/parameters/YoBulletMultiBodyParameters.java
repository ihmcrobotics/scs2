package us.ihmc.scs2.simulation.bullet.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoBulletMultiBodyParameters
{
   private boolean updateGlobalMultiBodyParameters;
   private final YoBoolean canSleep;
   private final YoBoolean hasSelfCollision;
   private final YoBoolean useGyroTerm;
   private final YoBoolean useGlobalVelocities;
   private final YoBoolean useRK4Intergration;
   private final YoDouble linearDamping;
   private final YoDouble angularDamping;
   private final YoDouble maxAppliedImpulse;
   private final YoDouble maxCoordinateVelocity;

   public YoBulletMultiBodyParameters(String prefix, YoRegistry registry)
   {
      String bulletRobotCanSleep;
      String bulletRobotHasSelfCollision;
      String bulletRobotUseGyroTerm;
      String bulletRobotUseGlobalVelocities;
      String bulletRobotAngularDamping;
      String bulletRobotLinearDamping;
      String bulletRobotMaxAppliedImpulse;
      String bulletRobotMaxCoordinateVelocity;
      String bulletRobotUseRK4Intergration;

      if (prefix == null || prefix.isEmpty())
      {
         bulletRobotCanSleep = "CanSleep";
         bulletRobotHasSelfCollision = "HasSelfCollision";
         bulletRobotUseGyroTerm = "UseGyroTerm";
         bulletRobotUseGlobalVelocities = "UseGlobalVelocities";
         bulletRobotUseRK4Intergration = "UseGlobalRK4Integation";
         bulletRobotAngularDamping = "AngularDamping";
         bulletRobotLinearDamping = "LinearDamping";
         bulletRobotMaxAppliedImpulse = "MaxAppliedImpulse";
         bulletRobotMaxCoordinateVelocity = "MaxCoordinateVelocity";
      }
      else
      {
         bulletRobotCanSleep = prefix + "CanSleep";
         bulletRobotHasSelfCollision = prefix + "HasSelfCollision";
         bulletRobotUseGyroTerm = prefix + "UseGyroTerm";
         bulletRobotUseGlobalVelocities = prefix + "UseGlobalVelocities";
         bulletRobotUseRK4Intergration = prefix + "UseGlobalRK4Integation";
         bulletRobotAngularDamping = prefix + "AngularDamping";
         bulletRobotLinearDamping = prefix + "LinearDamping";
         bulletRobotMaxAppliedImpulse = prefix + "MaxAppliedImpulse";
         bulletRobotMaxCoordinateVelocity = prefix + "MaxCoordinateVelocity";
      }

      canSleep = new YoBoolean(bulletRobotCanSleep, registry);
      hasSelfCollision = new YoBoolean(bulletRobotHasSelfCollision, registry);
      useGyroTerm = new YoBoolean(bulletRobotUseGyroTerm, registry);
      useGlobalVelocities = new YoBoolean(bulletRobotUseGlobalVelocities, registry);
      useRK4Intergration = new YoBoolean(bulletRobotUseRK4Intergration, registry);
      angularDamping = new YoDouble(bulletRobotAngularDamping, registry);
      linearDamping = new YoDouble(bulletRobotLinearDamping, registry);
      maxAppliedImpulse = new YoDouble(bulletRobotMaxAppliedImpulse, registry);
      maxCoordinateVelocity = new YoDouble(bulletRobotMaxCoordinateVelocity, registry);
      updateGlobalMultiBodyParameters = false;

      canSleep.addListener(v -> updateGlobalMultiBodyParameters = true);
      hasSelfCollision.addListener(v -> updateGlobalMultiBodyParameters = true);
      useGyroTerm.addListener(v -> updateGlobalMultiBodyParameters = true);
      useGlobalVelocities.addListener(v -> updateGlobalMultiBodyParameters = true);
      useRK4Intergration.addListener(v -> updateGlobalMultiBodyParameters = true);
      angularDamping.addListener(v -> updateGlobalMultiBodyParameters = true);
      linearDamping.addListener(v -> updateGlobalMultiBodyParameters = true);
      maxAppliedImpulse.addListener(v -> updateGlobalMultiBodyParameters = true);
      maxCoordinateVelocity.addListener(v -> updateGlobalMultiBodyParameters = true);
   }

   public void set(BulletMultiBodyParameters parameters)
   {
      setCanSleep(parameters.getCanSleep());
      setHasSelfCollision(parameters.getHasSelfCollision());
      setUseGyroTerm(parameters.getUseGyroTerm());
      setUseGlobalVelocities(parameters.getUseGlobalVelocities());
      setUseRK4Integration(parameters.getUseRK4Integration());
      setAngularDamping(parameters.getAngularDamping());
      setLinearDamping(parameters.getLinearDamping());
      setMaxAppliedImpulse(parameters.getMaxAppliedImpulse());
      setMaxCoordinateVelocity(parameters.getMaxCoordinateVelocity());
      setUpdateGlobalMultiBodyParameters(false);
   }

   public void setUpdateGlobalMultiBodyParameters(boolean updateGlobalMultiBodyParameters)
   {
      this.updateGlobalMultiBodyParameters = updateGlobalMultiBodyParameters;
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

   public void setMaxAppliedImpulse(double maxAppliedImpulse)
   {
      this.maxAppliedImpulse.set(maxAppliedImpulse);
   }

   public void setMaxCoordinateVelocity(double maxCoordinateVelocity)
   {
      this.maxCoordinateVelocity.set(maxCoordinateVelocity);
   }

   public boolean getUpdateGlobalMultiBodyParameters()
   {
      return updateGlobalMultiBodyParameters;
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

   public double getMaxAppliedImpulse()
   {
      return maxAppliedImpulse.getValue();
   }

   public double getMaxCoordinateVelocity()
   {
      return maxCoordinateVelocity.getValue();
   }
}

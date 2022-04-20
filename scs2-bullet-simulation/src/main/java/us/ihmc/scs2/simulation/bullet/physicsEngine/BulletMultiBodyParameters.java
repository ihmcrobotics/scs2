package us.ihmc.scs2.simulation.bullet.physicsEngine;

public class BulletMultiBodyParameters
{
   private boolean fixedBase;
   private boolean canSleep;
   private boolean hasSelfCollision;
   private boolean useGyroTerm;
   private boolean useGlobalVelocities;
   private boolean useRK4Intergration;
   private double linearDamping;
   private double angularDamping;
   private double jointFriction;
   private double maxAppliedImpulse;
   private double maxCoordinateVelocity;

   public static BulletMultiBodyParameters defaultBulletMultiBodyParameters()
   {
      BulletMultiBodyParameters bulletMultiBodyParameters = new BulletMultiBodyParameters();
      bulletMultiBodyParameters.setFixedBase(false);
      bulletMultiBodyParameters.setCanSleep(false);
      bulletMultiBodyParameters.setHasSelfCollision(true);
      bulletMultiBodyParameters.setUseGyroTerm(true);
      bulletMultiBodyParameters.setUseRK4Intergration(false);
      bulletMultiBodyParameters.setLinearDamping(0.1);
      bulletMultiBodyParameters.setAngularDamping(0.9);
      bulletMultiBodyParameters.setJointFriction(0.7);
      bulletMultiBodyParameters.setMaxAppliedImpulse(1000);
      bulletMultiBodyParameters.setMaxCoordinateVelocity(100);
      return bulletMultiBodyParameters;
   }

   public BulletMultiBodyParameters()
   {
   }

   public BulletMultiBodyParameters(boolean fixedBase,
                                    boolean canSleep,
                                    boolean hasSelfCollision,
                                    boolean useGyroTerm,
                                    boolean useGlobalVelocities,
                                    boolean useRK4Integration,
                                    double linearDamping,
                                    double angularDamping,
                                    double jointFriction,
                                    double maxAppliedImpulse,
                                    double maxCoordinatedVelocity)
   {
      this.fixedBase = fixedBase;
      this.canSleep = canSleep;
      this.hasSelfCollision = hasSelfCollision;
      this.useGyroTerm = useGyroTerm;
      this.useGlobalVelocities = useGlobalVelocities;
      this.useRK4Intergration = useRK4Integration;
      this.linearDamping = linearDamping;
      this.angularDamping = angularDamping;
      this.jointFriction = jointFriction;
      this.maxAppliedImpulse = maxAppliedImpulse;
      this.maxCoordinateVelocity = maxCoordinatedVelocity;
   }

   public void setFixedBase(boolean fixedBase)
   {
      this.fixedBase = fixedBase;
   }

   public void setCanSleep(boolean canSleep)
   {
      this.canSleep = canSleep;
   }

   public void setHasSelfCollision(boolean hasSelfCollision)
   {
      this.hasSelfCollision = hasSelfCollision;
   }

   public void setUseGyroTerm(boolean useGyroTerm)
   {
      this.useGyroTerm = useGyroTerm;
   }
   
   public void setUseGlobalVelocities(boolean useGlobalVelocities)
   {
      this.useGlobalVelocities = useGlobalVelocities;
   }
   
   public void setUseRK4Intergration(boolean useRK4Intergration)
   {
      this.useRK4Intergration = useRK4Intergration;
   }

   public void setLinearDamping(double linearDamping)
   {
      this.linearDamping = linearDamping;
   }

   public void setAngularDamping(double angularDamping)
   {
      this.angularDamping = angularDamping;
   }

   public void setMaxAppliedImpulse(double maxAppliedImpulse)
   {
      this.maxAppliedImpulse = maxAppliedImpulse;
   }

   public void setMaxCoordinateVelocity(double maxCoordinateVelocity)
   {
      this.maxCoordinateVelocity = maxCoordinateVelocity;
   }

   public void setJointFriction(double jointFriction)
   {
      this.jointFriction = jointFriction;
   }

   public boolean getFixedBase()
   {
      return fixedBase;
   }

   public boolean getCanSleep()
   {
      return canSleep;
   }

   public boolean getHasSelfCollision()
   {
      return hasSelfCollision;
   }

   public boolean getUseGyroTerm()
   {
      return useGyroTerm;
   }
   
   public boolean getUseGlobalVelocities()
   {
      return useGlobalVelocities;
   }
   
   public boolean getUseRK4Integration()
   {
      return useRK4Intergration;
   }

   public double getLinearDamping()
   {
      return linearDamping;
   }

   public double getAngularDamping()
   {
      return angularDamping;
   }

   public double getJointFriction()
   {
      return jointFriction;
   }

   public double getMaxAppliedImpulse()
   {
      return maxAppliedImpulse;
   }

   public double getMaxCoordinateVelocity()
   {
      return maxCoordinateVelocity;
   }
}

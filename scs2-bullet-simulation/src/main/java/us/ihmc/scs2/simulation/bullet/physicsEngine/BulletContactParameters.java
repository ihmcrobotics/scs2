package us.ihmc.scs2.simulation.bullet.physicsEngine;

public class BulletContactParameters
{
   private boolean fixedBase;
   private boolean canSleep;
   private boolean hasSelfCollision;
   private float linearDamping;
   private float angularDamping;
   private float jointFriction;

   public static BulletContactParameters defaultBulletContactParameters()
   {
      BulletContactParameters bulletContactParameters = new BulletContactParameters();
      bulletContactParameters.setFixedBase(false);
      bulletContactParameters.setCanSleep(false);
      bulletContactParameters.setHasSelfCollision(true);
      bulletContactParameters.setLinearDamping(0);
      bulletContactParameters.setAngularDamping(0);
      bulletContactParameters.setJointFriction(0.7f);
      return bulletContactParameters;
   }

   public BulletContactParameters()
   {
   }

   public BulletContactParameters(boolean fixedBase, boolean canSleep, boolean hasSelfCollision, float linearDamping, float angularDamping)
   {
      this.fixedBase = fixedBase;
      this.canSleep = canSleep;
      this.hasSelfCollision = hasSelfCollision;
      this.linearDamping = linearDamping;
      this.angularDamping = angularDamping;
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

   public void setLinearDamping(float linearDamping)
   {
      this.linearDamping = linearDamping;
   }

   public void setAngularDamping(float angularDamping)
   {
      this.angularDamping = angularDamping;
   }
   
   public void setJointFriction(float jointFriction)
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

   public float getLinearDamping()
   {
      return linearDamping;
   }

   public float getAngularDamping()
   {
      return angularDamping;
   }

   public float getJointFriction()
   {
      return jointFriction;
   }
}

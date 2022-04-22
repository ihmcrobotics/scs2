package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.yoVariables.variable.YoDouble;

public class BulletMultiBodyJointParameters
{
   private double jointFriction;
   private double jointRestitution;
   private double jointHitFraction;
   private double jointRollingFriction;
   private double jointSpinningFriction;
   private double jointContactProcessingThreshold;

   public static BulletMultiBodyJointParameters defaultBulletMultiBodyJointParameters()
   {
      BulletMultiBodyJointParameters bulletMultiBodyJointParameters = new BulletMultiBodyJointParameters();
      bulletMultiBodyJointParameters.setJointFriction(0.7);
      bulletMultiBodyJointParameters.setJointResitution(0);
      bulletMultiBodyJointParameters.setJointHitFraction(1);
      bulletMultiBodyJointParameters.setJointRollingFriction(0);
      bulletMultiBodyJointParameters.setJointSpinningFriction(0);
      
      //The constraint solver can discard solving contacts, if the distance is above this threshold. 0 by default.
      //Note: that using contacts with positive distance can improve stability. It increases, however, the chance of colliding with degerate contacts, such as 'interior' triangle edges
      //Note: when this parameter is set to zero, the simulation doesn't work well - need to investigate further what this should be defaulted as
      bulletMultiBodyJointParameters.setJointContactProcessingThreshold(0.00001);
      return bulletMultiBodyJointParameters;
   }

   public BulletMultiBodyJointParameters()
   {
   }

   public BulletMultiBodyJointParameters(double jointFriction, 
                                         double jointRestitution,
                                         double jointHitFration,
                                         double jointRollingFriction,
                                         double jointSpinningFriction,
                                         double jointContactProcessingThreshold)
   {
      this.jointFriction = jointFriction;
      this.jointRestitution = jointRestitution;
      this.jointHitFraction = jointHitFration;
      this.jointRollingFriction = jointRollingFriction;
      this.jointSpinningFriction = jointSpinningFriction;
      this.jointContactProcessingThreshold = jointContactProcessingThreshold;
   }

   public void setJointFriction(double jointFriction)
   {
      this.jointFriction = jointFriction;
   }
   
   public void setJointResitution(double jointRestitution)
   {
      this.jointRestitution = jointRestitution;
   }
   
   public void setJointHitFraction(double jointHitFraction)
   {
      this.jointHitFraction = jointHitFraction;
   }
   
   public void setJointRollingFriction(double jointRollingFriction)
   {
      this.jointRollingFriction = jointRollingFriction;
   }
   
   public void setJointSpinningFriction(double jointSpinningFriction)
   {
      this.jointSpinningFriction = jointSpinningFriction;
   }
   
   public void setJointContactProcessingThreshold(double jointContactProcessingThreshold)
   {
      this.jointContactProcessingThreshold = jointContactProcessingThreshold;
   }

   public double getJointFriction()
   {
      return jointFriction;
   }
   
   public double getJointRestitution()
   {
      return jointRestitution;
   }
   
   public double getJointHitFraction()
   {
      return jointHitFraction;
   }
   
   public double getJointRollingFriction()
   {
      return jointRollingFriction;
   }
   
   public double getJointSpinningFriction()
   {
      return jointSpinningFriction;
   }
   
   public double getJointContactProcessingThreshold()
   {
      return jointContactProcessingThreshold;
   }
}

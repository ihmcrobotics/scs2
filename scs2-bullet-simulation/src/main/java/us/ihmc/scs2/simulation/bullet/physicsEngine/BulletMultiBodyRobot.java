package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;

import org.bytedeco.bullet.BulletDynamics.btMultiBodyConstraint;
import org.bytedeco.bullet.LinearMath.btVector3;

import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyParameters;

public class BulletMultiBodyRobot
{
   private final org.bytedeco.bullet.BulletDynamics.btMultiBody btMultiBody;
   private HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
   private final ArrayList<BulletMultiBodyLinkCollider> allBulletMultiBodyLinkColliders = new ArrayList<>();
   private final ArrayList<org.bytedeco.bullet.BulletDynamics.btMultiBodyConstraint> allBtMultiBodyConstraints = new ArrayList<>();

   public BulletMultiBodyRobot(int numberOfLinks,
                               float rootBodyMass,
                               btVector3 rootBodyIntertia,
                               boolean fixedBase,
                               boolean canSleep,
                               HashMap<String, Integer> jointNameToBulletJointIndexMap)
   {
      btMultiBody = new org.bytedeco.bullet.BulletDynamics.btMultiBody(numberOfLinks, rootBodyMass, rootBodyIntertia, fixedBase, canSleep);

      this.jointNameToBulletJointIndexMap = jointNameToBulletJointIndexMap;
   }

   public org.bytedeco.bullet.BulletDynamics.btMultiBody getBtMultiBody()
   {
      return btMultiBody;
   }

   public ArrayList<BulletMultiBodyLinkCollider> getBulletMultiBodyLinkColliderArray()
   {
      return allBulletMultiBodyLinkColliders;
   }

   public BulletMultiBodyLinkCollider getBulletMultiBodyLinkCollider(int index)
   {
      return allBulletMultiBodyLinkColliders.get(index);
   }

   public ArrayList<org.bytedeco.bullet.BulletDynamics.btMultiBodyConstraint> getBtMultiBodyConstraintArray()
   {
      return allBtMultiBodyConstraints;
   }

   public HashMap<String, Integer> getJointNameToBulletJointIndexMap()
   {
      return jointNameToBulletJointIndexMap;
   }

   public void addBulletMuliBodyLinkCollider(BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider)
   {
      allBulletMultiBodyLinkColliders.add(bulletMultiBodyLinkCollider);
   }

   public void addBtMultiBodyConstraint(btMultiBodyConstraint btMultiBodyConstraint)
   {
      allBtMultiBodyConstraints.add(btMultiBodyConstraint);
   }

   public void finalizeMultiDof(YoBulletMultiBodyParameters bulletMultiBodyParameters, YoBulletMultiBodyJointParameters bulletMultiBodyJointParameters)
   {
      setMultiBodyParameters(bulletMultiBodyParameters);
      setMultiBodyJointParameters(bulletMultiBodyJointParameters);

      btMultiBody.finalizeMultiDof();
   }

   public void setMultiBodyParameters(YoBulletMultiBodyParameters bulletMultiBodyParameters)
   {
      btMultiBody.setHasSelfCollision(bulletMultiBodyParameters.getHasSelfCollision());
      btMultiBody.setCanSleep(bulletMultiBodyParameters.getCanSleep());
      btMultiBody.setUseGyroTerm(bulletMultiBodyParameters.getUseGyroTerm());
      btMultiBody.useGlobalVelocities(bulletMultiBodyParameters.getUseGlobalVelocities());
      btMultiBody.useRK4Integration(bulletMultiBodyParameters.getUseRK4Integration());
      btMultiBody.setLinearDamping((float) bulletMultiBodyParameters.getLinearDamping());
      btMultiBody.setAngularDamping((float) bulletMultiBodyParameters.getAngularDamping());
      btMultiBody.setMaxAppliedImpulse((float) bulletMultiBodyParameters.getMaxAppliedImpulse());
      btMultiBody.setMaxCoordinateVelocity((float) bulletMultiBodyParameters.getMaxCoordinateVelocity());
   }

   public void setMultiBodyJointParameters(YoBulletMultiBodyJointParameters bulletMultiBodyJointParameters)
   {
      for (BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider : allBulletMultiBodyLinkColliders)
      {
         bulletMultiBodyLinkCollider.setFriction(bulletMultiBodyJointParameters.getJointFriction());
         bulletMultiBodyLinkCollider.setRestitution(bulletMultiBodyJointParameters.getJointRestitution());
         bulletMultiBodyLinkCollider.setHitFraction(bulletMultiBodyJointParameters.getJointHitFraction());
         bulletMultiBodyLinkCollider.setRollingFriction(bulletMultiBodyJointParameters.getJointRollingFriction());
         bulletMultiBodyLinkCollider.setSpinningFriction(bulletMultiBodyJointParameters.getJointSpinningFriction());
         bulletMultiBodyLinkCollider.setContactProcessingThreshold(bulletMultiBodyJointParameters.getJointContactProcessingThreshold());
      }
   }

}

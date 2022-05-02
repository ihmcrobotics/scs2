package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraint;

public class BulletMultiBodyRobot
{
   private final btMultiBody btMultiBody;
   private HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
   private final ArrayList<BulletMultiBodyLinkCollider> allBulletMultiBodyLinkColliders = new ArrayList<>();
   private final ArrayList<btMultiBodyConstraint> allBtMultiBodyConstraints = new ArrayList<>();

   public BulletMultiBodyRobot(int numberOfLinks,
                               float rootBodyMass,
                               Vector3 rootBodyIntertia,
                               boolean fixedBase,
                               boolean canSleep,
                               HashMap<String, Integer> jointNameToBulletJointIndexMap)
   {
      btMultiBody = new btMultiBody(numberOfLinks, rootBodyMass, rootBodyIntertia, fixedBase, canSleep);

      this.jointNameToBulletJointIndexMap = jointNameToBulletJointIndexMap;
   }

   public btMultiBody getBtMultiBody()
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

   public ArrayList<btMultiBodyConstraint> getBtMultiBodyConstrantArray()
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

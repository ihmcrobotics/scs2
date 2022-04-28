package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraint;

public class BulletMultiBodyRobot
{
   private btMultiBody bulletMultiBody;
   private HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
   private final ArrayList<BulletMultiBodyLinkCollider> allBulletMultiBodyLinkColliders = new ArrayList<>();
   private final ArrayList<btMultiBodyConstraint> allBulletMultiBodyConstraints = new ArrayList<>();

   public BulletMultiBodyRobot(int numberOfLinks,
                               float rootBodyMass,
                               Vector3 rootBodyIntertia,
                               boolean fixedBase,
                               boolean canSleep,
                               HashMap<String, Integer> jointNameToBulletJointIndexMap)
   {
      bulletMultiBody = new btMultiBody(numberOfLinks,
                                             rootBodyMass,
                                             rootBodyIntertia,
                                             fixedBase,
                                             canSleep);
      
      this.jointNameToBulletJointIndexMap = jointNameToBulletJointIndexMap;
   }

   public btMultiBody getBulletMultiBody()
   {
      return bulletMultiBody;
   }

   public ArrayList<BulletMultiBodyLinkCollider> getBulletMultiBodyLinkColliderArray()
   {
      return allBulletMultiBodyLinkColliders;
   }

   public BulletMultiBodyLinkCollider getBulletMultiBodyLinkCollider(int index)
   {
      return allBulletMultiBodyLinkColliders.get(index);
   }

   public ArrayList<btMultiBodyConstraint> getBulletMultiBodyConstrantArray()
   {
      return allBulletMultiBodyConstraints;
   }

   public HashMap<String, Integer> getJointNameToBulletJointIndexMap()
   {
      return jointNameToBulletJointIndexMap;
   }

   public void addBulletMuliBodyLinkCollider(BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider)
   {
      allBulletMultiBodyLinkColliders.add(bulletMultiBodyLinkCollider);
   }

   public void addMultiBodyConstraint(btMultiBodyConstraint bulletMultiBodyConstraint)
   {
      allBulletMultiBodyConstraints.add(bulletMultiBodyConstraint);
   }

   public void finalizeMultiDof(YoBulletMultiBodyParameters bulletMultiBodyParameters, YoBulletMultiBodyJointParameters bulletMultiBodyJointParameters)
   {
      setMultiBodyParameters(bulletMultiBodyParameters);
      setMultiBodyJointParameters(bulletMultiBodyJointParameters);
      
      bulletMultiBody.finalizeMultiDof();
   }

   public void setMultiBodyParameters(YoBulletMultiBodyParameters bulletMultiBodyParameters)
   {
      bulletMultiBody.setHasSelfCollision(bulletMultiBodyParameters.getHasSelfCollision());
      bulletMultiBody.setCanSleep(bulletMultiBodyParameters.getCanSleep());
      bulletMultiBody.setUseGyroTerm(bulletMultiBodyParameters.getUseGyroTerm());
      bulletMultiBody.useGlobalVelocities(bulletMultiBodyParameters.getUseGlobalVelocities());
      bulletMultiBody.useRK4Integration(bulletMultiBodyParameters.getUseRK4Integration());
      bulletMultiBody.setLinearDamping((float)bulletMultiBodyParameters.getLinearDamping());
      bulletMultiBody.setAngularDamping((float)bulletMultiBodyParameters.getAngularDamping());
      bulletMultiBody.setMaxAppliedImpulse((float)bulletMultiBodyParameters.getMaxAppliedImpulse());
      bulletMultiBody.setMaxCoordinateVelocity((float)bulletMultiBodyParameters.getMaxCoordinateVelocity());
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

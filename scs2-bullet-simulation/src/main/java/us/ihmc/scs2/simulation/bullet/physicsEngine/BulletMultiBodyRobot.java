package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraint;

public class BulletMultiBodyRobot
{
   private btMultiBody bulletMultiBodyRobot;
   private HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
   private final ArrayList<BulletMultiBodyLinkCollider> allLinks = new ArrayList<>();
   private final ArrayList<btMultiBodyConstraint> allConstraints = new ArrayList<>();

   public BulletMultiBodyRobot(int numberOfLinks,
                               float rootBodyMass,
                               Vector3 rootBodyIntertia,
                               HashMap<String, Integer> jointNameToBulletJointIndexMap,
                               YoBulletMultiBodyParameters bulletMultiBodyParameters)
   {
      bulletMultiBodyRobot = new btMultiBody(numberOfLinks,
                                             rootBodyMass,
                                             rootBodyIntertia,
                                             bulletMultiBodyParameters.getFixedBase(),
                                             bulletMultiBodyParameters.getCanSleep());
      this.jointNameToBulletJointIndexMap = jointNameToBulletJointIndexMap;

      setMultiBodyParameters(bulletMultiBodyParameters);
   }

   public btMultiBody getBulletMultiBody()
   {
      return bulletMultiBodyRobot;
   }

   public ArrayList<BulletMultiBodyLinkCollider> getBulletMultiBodyLinkColliderArray()
   {
      return allLinks;
   }

   public BulletMultiBodyLinkCollider getBulletMultiBodyLinkCollider(int index)
   {
      return allLinks.get(index);
   }

   public ArrayList<btMultiBodyConstraint> getBulletMultiBodyConstrantArray()
   {
      return allConstraints;
   }

   public HashMap<String, Integer> getJointNameToBulletJointIndexMap()
   {
      return jointNameToBulletJointIndexMap;
   }

   public void addBulletMuliBodyLinkCollider(BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider)
   {
      allLinks.add(bulletMultiBodyLinkCollider);
   }

   public void addMultiBodyConstraint(btMultiBodyConstraint bulletMultiBodyConstraint)
   {
      allConstraints.add(bulletMultiBodyConstraint);
   }

   public void finalizeMultiDof()
   {
      bulletMultiBodyRobot.finalizeMultiDof();
   }

   public void setMultiBodyParameters(YoBulletMultiBodyParameters bulletMultiBodyParameters)
   {
      bulletMultiBodyRobot.setHasSelfCollision(bulletMultiBodyParameters.getHasSelfCollision());
      bulletMultiBodyRobot.setCanSleep(bulletMultiBodyParameters.getCanSleep());
      bulletMultiBodyRobot.setUseGyroTerm(bulletMultiBodyParameters.getUseGyroTerm());
      bulletMultiBodyRobot.useGlobalVelocities(bulletMultiBodyParameters.getUseGlobalVelocities());
      bulletMultiBodyRobot.useRK4Integration(bulletMultiBodyParameters.getUseRK4Integration());
      bulletMultiBodyRobot.setLinearDamping((float)bulletMultiBodyParameters.getLinearDamping());
      bulletMultiBodyRobot.setAngularDamping((float)bulletMultiBodyParameters.getAngularDamping());
      bulletMultiBodyRobot.setMaxAppliedImpulse((float)bulletMultiBodyParameters.getMaxAppliedImpulse());
      bulletMultiBodyRobot.setMaxCoordinateVelocity((float)bulletMultiBodyParameters.getMaxCoordinateVelocity());
      
      for (int i = 0; i < bulletMultiBodyRobot.getNumLinks(); i++)
      {
         bulletMultiBodyRobot.getLink(0).setJointFriction((float)bulletMultiBodyParameters.getJointFriction());
      }

   }

}

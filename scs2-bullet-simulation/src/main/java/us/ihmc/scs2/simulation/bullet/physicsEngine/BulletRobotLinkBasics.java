package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;

public abstract class BulletRobotLinkBasics
{
   private final SimRigidBodyBasics simRigidBody;
   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry;
   private final BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider;

   public BulletRobotLinkBasics(SimRigidBodyBasics simRigidBody,
                                RigidBodyWrenchRegistry rigidBodyWrenchRegistry,
                                BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider)
   {
      this.simRigidBody = simRigidBody;
      this.rigidBodyWrenchRegistry = rigidBodyWrenchRegistry;
      this.bulletMultiBodyLinkCollider = bulletMultiBodyLinkCollider;
   }

   public void updateBulletLinkColliderTransformFromMecanoRigidBody()
   {
      bulletMultiBodyLinkCollider.setWorldTransform(simRigidBody.getBodyFixedFrame().getTransformToRoot());
   }

   public abstract void pushStateToBullet();

   public abstract void pullStateFromBullet(double dt);

   public SimRigidBodyBasics getSimRigidBody()
   {
      return simRigidBody;
   }

   public BulletMultiBodyLinkCollider getBulletMultiBodyLinkCollider()
   {
      return bulletMultiBodyLinkCollider;
   }

   public RigidBodyWrenchRegistry getRigidBodyWrenchRegistry()
   {
      return rigidBodyWrenchRegistry;
   }
}

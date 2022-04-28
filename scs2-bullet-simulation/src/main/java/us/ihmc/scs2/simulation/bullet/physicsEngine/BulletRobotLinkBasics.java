package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;

public abstract class BulletRobotLinkBasics 
{
   private SimRigidBodyBasics simRigidBody;
   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry;
   private BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider;
   private final RigidBodyTransform bulletColliderCenterOfMassTransformToWorldEuclid = new RigidBodyTransform();

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
      simRigidBody.getBodyFixedFrame().getTransformToDesiredFrame(bulletColliderCenterOfMassTransformToWorldEuclid, SimulationSession.DEFAULT_INERTIAL_FRAME);
      bulletMultiBodyLinkCollider.setWorldTransform(bulletColliderCenterOfMassTransformToWorldEuclid);
   }

   public abstract void copyDataFromSCSToBullet();

   public abstract void copyBulletJointDataToSCS(double dt);

   public SimRigidBodyBasics getSimRigidBody()
   {
      return simRigidBody;
   }

   public BulletMultiBodyLinkCollider getBulletMultiBodyLinkCollider()
   {
      return bulletMultiBodyLinkCollider;
   }

   public RigidBodyTransform getbulletColliderCenterOfMassTransformToWorldEuclid()
   {
      return bulletColliderCenterOfMassTransformToWorldEuclid;
   }
   public RigidBodyWrenchRegistry getRigidBodyWrenchRegistry()
   {
      return rigidBodyWrenchRegistry;
   }
}

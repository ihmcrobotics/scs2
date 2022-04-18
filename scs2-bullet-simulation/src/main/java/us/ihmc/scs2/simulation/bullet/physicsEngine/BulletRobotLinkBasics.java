package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;

public abstract class BulletRobotLinkBasics 
{
   private SimRigidBodyBasics simRigidBody;
   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry;
   private BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider;
   private final Matrix4 bulletColliderCenterOfMassTransformToWorldBullet = new Matrix4();
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
      BulletTools.toBullet(bulletColliderCenterOfMassTransformToWorldEuclid, bulletColliderCenterOfMassTransformToWorldBullet);
      bulletMultiBodyLinkCollider.getMultiBodyLinkCollider().setWorldTransform(bulletColliderCenterOfMassTransformToWorldBullet);
   }

   public abstract void copyDataFromSCSToBullet();

   public abstract void copyBulletJointDataToSCS(double dt);

   public SimRigidBodyBasics getSimRigidBody()
   {
      return simRigidBody;
   }

   public int getBulletJointIndex()
   {
      return bulletMultiBodyLinkCollider.getLinkColliderIndex();
   }

   public btMultiBodyLinkCollider getBulletMultiBodyLinkCollider()
   {
      return bulletMultiBodyLinkCollider.getMultiBodyLinkCollider();
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

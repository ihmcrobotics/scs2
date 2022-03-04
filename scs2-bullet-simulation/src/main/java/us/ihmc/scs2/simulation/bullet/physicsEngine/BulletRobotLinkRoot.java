package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;

public class BulletRobotLinkRoot extends BulletRobotLinkBasics
{
   private int linkCountingIndex = 0;
   private final int numberOfLinks;
   private final SimFloatingRootJoint rootSimFloatingRootJoint;
   private final Matrix4 bulletPelvisColliderCenterOfMassTransformToWorldBullet = new Matrix4();
   private final RigidBodyTransform bulletPelvisColliderCenterOfMassTransformToWorldEuclid = new RigidBodyTransform();
   private final RigidBodyTransform bulletPelvisAfterJointTransformToWorldEuclid = new RigidBodyTransform();
   private final RigidBodyTransform bulletPelvisCenterOfMassTransformToAfterParentJointEuclid = new RigidBodyTransform();

   public BulletRobotLinkRoot(SixDoFJointDefinition rootSixDoFJointDefinition,
                              SimFloatingRootJoint rootSimFloatingRootJoint,
                              HashMap<String, Integer> jointNameToBulletJointIndexMap,
                              YoRegistry yoRegistry)
   {
      super(rootSixDoFJointDefinition.getSuccessor(), rootSimFloatingRootJoint.getSuccessor(), jointNameToBulletJointIndexMap);
      this.rootSimFloatingRootJoint = rootSimFloatingRootJoint;

      setBulletJointIndex(-1);
      numberOfLinks = countJoints(rootSimFloatingRootJoint) - 1; // which is also number of joints in this case

      bulletPelvisCenterOfMassTransformToAfterParentJointEuclid.set(getRigidBodyDefinition().getInertiaPose());

      addChildLinks(yoRegistry);
   }

   @Override
   public void setup()
   {
      boolean fixedBase = false;
      boolean canSleep = false;
      float rootBodyMass = (float) getRigidBodyDefinition().getMass();
      Vector3 rootBodyIntertia = new Vector3((float) getRigidBodyDefinition().getMomentOfInertia().getM00(),
                                             (float) getRigidBodyDefinition().getMomentOfInertia().getM11(),
                                             (float) getRigidBodyDefinition().getMomentOfInertia().getM22());
      btMultiBody bulletMultiBody = new btMultiBody(numberOfLinks, rootBodyMass, rootBodyIntertia, fixedBase, canSleep);
      bulletMultiBody.setHasSelfCollision(true);
      bulletMultiBody.setUseGyroTerm(true);
      bulletMultiBody.setLinearDamping(0.1f);
      bulletMultiBody.setAngularDamping(0.9f);
      setBulletMultiBody(bulletMultiBody);
   }

   @Override
   public void createBulletCollisionShape(BulletPhysicsEngine bulletPhysicsManager)
   {
      super.createBulletCollisionShape(bulletPhysicsManager);
      getBulletMultiBody().setBaseCollider(getBulletMultiBodyLinkCollider());
   }

   @Override
   public void copyDataFromSCSToBullet()
   {
      updateBulletLinkColliderTransformFromMecanoRigidBody();

      getBulletMultiBody().setBaseWorldTransform(getBulletColliderCenterOfMassTransformToWorldBullet());
   }

   @Override
   public void copyBulletJointDataToSCS()
   {
      getBulletMultiBodyLinkCollider().getWorldTransform(bulletPelvisColliderCenterOfMassTransformToWorldBullet);
      BulletTools.toEuclid(bulletPelvisColliderCenterOfMassTransformToWorldBullet, bulletPelvisColliderCenterOfMassTransformToWorldEuclid);
      bulletPelvisAfterJointTransformToWorldEuclid.set(bulletPelvisColliderCenterOfMassTransformToWorldEuclid);
      bulletPelvisCenterOfMassTransformToAfterParentJointEuclid.inverseTransform(bulletPelvisAfterJointTransformToWorldEuclid);
      rootSimFloatingRootJoint.setJointPosition(bulletPelvisAfterJointTransformToWorldEuclid.getTranslation());
      rootSimFloatingRootJoint.setJointOrientation(bulletPelvisAfterJointTransformToWorldEuclid.getRotation());
      // TODO: Calculate velocity & acceleration to pack Mecano stuff?
   }

   private int countJoints(JointBasics joint)
   {
      getJointNameToBulletJointIndexMap().put(joint.getName(), linkCountingIndex - 1);
      ++linkCountingIndex;
      int numberOfJoints = 1;
      for (JointBasics childrenJoint : joint.getSuccessor().getChildrenJoints())
      {
         numberOfJoints += countJoints(childrenJoint);
      }
      return numberOfJoints;
   }
}

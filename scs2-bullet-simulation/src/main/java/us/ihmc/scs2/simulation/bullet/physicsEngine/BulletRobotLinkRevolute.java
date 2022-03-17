package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.HashMap;

import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.dynamics.btMultibodyLink;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.sensors.SimWrenchSensor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BulletRobotLinkRevolute extends BulletRobotLinkBasics
{
   private final RevoluteJointDefinition revoluteJointDefinition;
   private final SimRevoluteJoint simRevoluteJoint;
   private final HashMap<btCollisionObject, BulletWrenchSensorCalculator> wrenchCalculatorMap;
   private int parentBulletJointIndex;
   private btMultibodyLink bulletLink;
   private YoDouble damping;
   private YoDouble bulletJointPosition;
   private YoDouble bulletJointVelocity;
   private YoDouble bulletJointVelocityTest;
   private YoDouble bulletUserAddedTorque;
   private YoDouble bulletJointTorque;
   private YoDouble bulletLinkAppliedForceX;
   private YoDouble bulletLinkAppliedForceY;
   private YoDouble bulletLinkAppliedForceZ;
   private YoDouble bulletLinkAppliedTorqueX;
   private YoDouble bulletLinkAppliedTorqueY;
   private YoDouble bulletLinkAppliedTorqueZ;

   public BulletRobotLinkRevolute(RevoluteJointDefinition revoluteJointDefinition,
                                  SimRevoluteJoint simRevoluteJoint,
                                  HashMap<String, Integer> jointNameToBulletJointIndexMap,
                                  HashMap<btCollisionObject, BulletWrenchSensorCalculator> wrenchCalculatorMap,
                                  YoRegistry yoRegistry)
   {
      super(revoluteJointDefinition.getSuccessor(), simRevoluteJoint.getSuccessor(), jointNameToBulletJointIndexMap, wrenchCalculatorMap);
      this.revoluteJointDefinition = revoluteJointDefinition;
      this.simRevoluteJoint = simRevoluteJoint;
      this.wrenchCalculatorMap = wrenchCalculatorMap;

      setBulletJointIndex(jointNameToBulletJointIndexMap.get(revoluteJointDefinition.getName()));
      parentBulletJointIndex = jointNameToBulletJointIndexMap.get(revoluteJointDefinition.getParentJoint().getName());

      addChildLinks(yoRegistry);

      damping = new YoDouble(simRevoluteJoint.getName() + "_damping", yoRegistry);
      bulletJointPosition = new YoDouble(simRevoluteJoint.getName() + "_q", yoRegistry);
      bulletJointVelocity = new YoDouble(simRevoluteJoint.getName() + "_qd", yoRegistry);
      bulletJointVelocityTest = new YoDouble(simRevoluteJoint.getName() + "_qdTest", yoRegistry);
      bulletUserAddedTorque = new YoDouble(simRevoluteJoint.getName() + "_btUserAddedTorque", yoRegistry);
      bulletJointTorque = new YoDouble(simRevoluteJoint.getName() + "_btJointTorque", yoRegistry);
      bulletLinkAppliedForceX = new YoDouble(simRevoluteJoint.getName() + "_btAppliedForceX", yoRegistry);
      bulletLinkAppliedForceY = new YoDouble(simRevoluteJoint.getName() + "_btAppliedForceY", yoRegistry);
      bulletLinkAppliedForceZ = new YoDouble(simRevoluteJoint.getName() + "_btAppliedForceZ", yoRegistry);
      bulletLinkAppliedTorqueX = new YoDouble(simRevoluteJoint.getName() + "_btAppliedTorqueX", yoRegistry);
      bulletLinkAppliedTorqueY = new YoDouble(simRevoluteJoint.getName() + "_btAppliedTorqueY", yoRegistry);
      bulletLinkAppliedTorqueZ = new YoDouble(simRevoluteJoint.getName() + "_btAppliedTorqueZ", yoRegistry);
   }

   @Override
   public void setup(BulletPhysicsEngine bulletPhysicsEngine)
   {
      Quaternion rotationFromParentGDX = new Quaternion();
      us.ihmc.euclid.tuple4D.Quaternion euclidRotationFromParent
            = new us.ihmc.euclid.tuple4D.Quaternion(revoluteJointDefinition.getTransformToParent().getRotation());
      euclidRotationFromParent.invert();
      BulletTools.toBullet(euclidRotationFromParent, rotationFromParentGDX);

      RigidBodyTransform parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid = new RigidBodyTransform();
      simRevoluteJoint.getPredecessor().getBodyFixedFrame().getTransformToDesiredFrame(parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid,
                                                                                       simRevoluteJoint.getFrameBeforeJoint());
      parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid.invert();
      Vector3 parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationGDX = new Vector3();
      BulletTools.toBullet(parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid.getTranslation(),
                           parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationGDX);

      RigidBodyTransform parentJointAfterFrameToLinkCenterOfMassTransformEuclid = new RigidBodyTransform();
      simRevoluteJoint.getFrameAfterJoint().getTransformToDesiredFrame(parentJointAfterFrameToLinkCenterOfMassTransformEuclid,
                                                                             getSimRigidBody().getBodyFixedFrame());
      parentJointAfterFrameToLinkCenterOfMassTransformEuclid.invert();
      Vector3 parentJointAfterFrameToLinkCenterOfMassTranslationGDX = new Vector3();
      BulletTools.toBullet(parentJointAfterFrameToLinkCenterOfMassTransformEuclid.getTranslation(), parentJointAfterFrameToLinkCenterOfMassTranslationGDX);

      float linkMass = (float) getRigidBodyDefinition().getMass();
      Vector3 baseInertiaDiagonal = new Vector3((float) getRigidBodyDefinition().getMomentOfInertia().getM00(),
                                                (float) getRigidBodyDefinition().getMomentOfInertia().getM11(),
                                                (float) getRigidBodyDefinition().getMomentOfInertia().getM22());
      BulletRobotLinkCollisionSet bulletCollisionSet = createBulletCollisionShape();
      // TODO: Should we let Bullet compute this?
      // bulletCollisionSet.getBulletCompoundShape().calculateLocalInertia(linkMass, baseInertiaDiagonal);

      Vector3 jointAxis = new Vector3();
      BulletTools.toBullet(revoluteJointDefinition.getAxis(), jointAxis);
      boolean disableParentCollision = true;
      getBulletMultiBody().setupRevolute(getBulletJointIndex(),
                                         linkMass,
                                         baseInertiaDiagonal,
                                         parentBulletJointIndex,
                                         rotationFromParentGDX,
                                         jointAxis,
                                         parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationGDX,
                                         parentJointAfterFrameToLinkCenterOfMassTranslationGDX,
                                         disableParentCollision);
      bulletLink = getBulletMultiBody().getLink(getBulletJointIndex());
      bulletLink.setJointDamping((float) revoluteJointDefinition.getDamping()); // Doesn't seem to do anything though
      bulletLink.setJointLowerLimit((float) revoluteJointDefinition.getPositionLowerLimit());
      bulletLink.setJointUpperLimit((float) revoluteJointDefinition.getPositionUpperLimit());
      bulletLink.setJointMaxForce((float) revoluteJointDefinition.getEffortUpperLimit());
      bulletLink.setJointMaxVelocity((float) revoluteJointDefinition.getVelocityUpperLimit());

      createBulletCollider(bulletPhysicsEngine);
      bulletLink.setCollider(getBulletMultiBodyLinkCollider());

      for (SimWrenchSensor wrenchSensor : simRevoluteJoint.getAuxialiryData().getWrenchSensors())
      {
         wrenchCalculatorMap.put(getBulletMultiBodyLinkCollider(), new BulletWrenchSensorCalculator(simRevoluteJoint, wrenchSensor));
      }
   }

   public void copyDataFromSCSToBullet()
   {
      updateBulletLinkColliderTransformFromMecanoRigidBody();

      getBulletMultiBody().setJointPos(getBulletJointIndex(), (float) simRevoluteJoint.getQ());
      getBulletMultiBody().setJointVel(getBulletJointIndex(), (float) simRevoluteJoint.getQd());
      // Don't call this here
      // getBulletMultiBody().clearForcesAndTorques();
      getBulletMultiBody().addJointTorque(getBulletJointIndex(), (float) simRevoluteJoint.getTau());
      
      if (simRevoluteJoint.getTau() != 0.0)
         System.out.println("sim tau " + simRevoluteJoint.getName() + " " + simRevoluteJoint.getTau() + " " + getBulletMultiBody().getJointTorque(getBulletJointIndex()));
   }

   public void copyBulletJointDataToSCS(double dt)
   {
      float jointPosition = getBulletMultiBody().getJointPos(getBulletJointIndex());
      bulletJointVelocityTest.set((jointPosition- simRevoluteJoint.getQ())/ dt);
      simRevoluteJoint.setQ(jointPosition);
      float jointPVel = getBulletMultiBody().getJointVel(getBulletJointIndex());
      simRevoluteJoint.setQdd((jointPVel - simRevoluteJoint.getQd())/dt);
      simRevoluteJoint.setQd(jointPVel);
      //simRevoluteJoint.setTau((double)getBulletMultiBody().getJointTorque(getBulletJointIndex()));

      // https://pybullet.org/Bullet/phpBB3/viewtopic.php?p=36667&hilit=btMultiBody+joint+torque#p36667
      // Assumes fixed time step. TODO: Get time of current step
      bulletJointPosition.set(jointPosition);
      bulletJointVelocity.set(jointPVel);
      bulletUserAddedTorque.set(damping.getValue() * bulletJointVelocity.getValue());
      bulletJointTorque.set(getBulletMultiBody().getJointTorque(getBulletJointIndex()));
      Vector3 linkForce = getBulletMultiBody().getLinkForce(getBulletJointIndex());
      bulletLinkAppliedForceX.set(linkForce.x);
      bulletLinkAppliedForceY.set(linkForce.y);
      bulletLinkAppliedForceZ.set(linkForce.z);
      Vector3 linkTorque = getBulletMultiBody().getLinkTorque(getBulletJointIndex());
      bulletLinkAppliedTorqueX.set(linkTorque.x);
      bulletLinkAppliedTorqueY.set(linkTorque.y);
      bulletLinkAppliedTorqueX.set(linkTorque.z);
   }

   public boolean isSameLink(RigidBodyDefinition rigidBodyDefinition)
   {
      return this.getRigidBodyDefinition().getName().equals(rigidBodyDefinition.getName());
   }

   public boolean isSameLink(RigidBodyBasics rigidBodyBasics)
   {
      return this.getRigidBodyDefinition().getName().equals(rigidBodyBasics.getName());
   }

   public YoDouble getDamping()
   {
      return damping;
   }

   public YoDouble getBulletJointPosition()
   {
      return bulletJointPosition;
   }

   public YoDouble getBulletJointVelocity()
   {
      return bulletJointVelocity;
   }

   public YoDouble getBulletUserAddedTorque()
   {
      return bulletUserAddedTorque;
   }

   public YoDouble getBulletJointTorque()
   {
      return bulletJointTorque;
   }

   public YoDouble getBulletLinkAppliedForceX()
   {
      return bulletLinkAppliedForceX;
   }

   public YoDouble getBulletLinkAppliedForceY()
   {
      return bulletLinkAppliedForceY;
   }

   public YoDouble getBulletLinkAppliedForceZ()
   {
      return bulletLinkAppliedForceZ;
   }

   public YoDouble getBulletLinkAppliedTorqueX()
   {
      return bulletLinkAppliedTorqueX;
   }

   public YoDouble getBulletLinkAppliedTorqueY()
   {
      return bulletLinkAppliedTorqueY;
   }

   public YoDouble getBulletLinkAppliedTorqueZ()
   {
      return bulletLinkAppliedTorqueZ;
   }
}

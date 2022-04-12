package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.HashMap;

import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraint;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyJointLimitConstraint;
import com.badlogic.gdx.physics.bullet.dynamics.btMultibodyLink;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AltBulletRobotLinkRevolute extends AltBulletRobotLinkBasics
{
   private final RevoluteJointDefinition revoluteJointDefinition;
   private final SimRevoluteJoint simRevoluteJoint;
   private int parentBulletJointIndex;
   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry;
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
   private btMultiBodyConstraint multiBodyJointLimitConstraint;

   public AltBulletRobotLinkRevolute(RevoluteJointDefinition revoluteJointDefinition,
                                  SimRevoluteJoint simRevoluteJoint,
                                  HashMap<String, Integer> jointNameToBulletJointIndexMap,
                                  RigidBodyWrenchRegistry rigidBodyWrenchRegistry,
                                  YoRegistry yoRegistry)
   {
      super(revoluteJointDefinition.getSuccessor(), simRevoluteJoint.getSuccessor(), jointNameToBulletJointIndexMap, rigidBodyWrenchRegistry);
      this.revoluteJointDefinition = revoluteJointDefinition;
      this.simRevoluteJoint = simRevoluteJoint;
      this.rigidBodyWrenchRegistry = rigidBodyWrenchRegistry;

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
   public void setup(AltBulletPhysicsEngine bulletPhysicsEngine)
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
      AltBulletRobotLinkCollisionSet bulletCollisionSet = createBulletCollisionShape();
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

      multiBodyJointLimitConstraint = new btMultiBodyJointLimitConstraint(getBulletMultiBody(),
                                                                          getBulletJointIndex(),
                                                                          (float) revoluteJointDefinition.getPositionLowerLimit(),
                                                                          (float) revoluteJointDefinition.getPositionUpperLimit());
      multiBodyJointLimitConstraint.setMaxAppliedImpulse((float) revoluteJointDefinition.getEffortUpperLimit());
      bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld().addMultiBodyConstraint(multiBodyJointLimitConstraint);
      bulletLink = getBulletMultiBody().getLink(getBulletJointIndex());
      
//      TODO: test if adding a multibodyJointMotor can be added to set max velocity and max force?      
//      m_motor = new btMultiBodyJointMotor(pMultiBody, link, targetVelocity, maxForce);
//      m_dynamicsWorld->addMultiBodyConstraint(m_motor);

//       btMultiBodyJointMotor::btMultiBodyJointMotor (  btMultiBody *  body,
//                                                       int   link,
//                                                       btScalar    desiredVelocity,
//                                                       btScalar    maxMotorImpulse 
//                                                    )  
      
      //The setJoint methods below do nothing -- see notes from documentation lines 147 - 152
//      147         btScalar m_jointDamping; //todo: implement this internally. It is unused for now, it is set by a URDF loader. User can apply manual damping.
//      148         btScalar m_jointFriction; //todo: implement this internally. It is unused for now, it is set by a URDF loader. User can apply manual friction using a velocity motor.
//      149         btScalar m_jointLowerLimit; //todo: implement this internally. It is unused for now, it is set by a URDF loader. 
//      150         btScalar m_jointUpperLimit; //todo: implement this internally. It is unused for now, it is set by a URDF loader.
//      151         btScalar m_jointMaxForce; //todo: implement this internally. It is unused for now, it is set by a URDF loader. 
//      152         btScalar m_jointMaxVelocity;//todo: implement this internally. It is unused for now, it is set by a URDF loader.
//      bulletLink.setJointDamping((float) revoluteJointDefinition.getDamping()); // Doesn't seem to do anything though
//      bulletLink.setJointLowerLimit((float) revoluteJointDefinition.getPositionLowerLimit());
//      bulletLink.setJointUpperLimit((float) revoluteJointDefinition.getPositionUpperLimit());
//      bulletLink.setJointMaxForce((float) revoluteJointDefinition.getEffortUpperLimit());
//      bulletLink.setJointMaxVelocity((float) revoluteJointDefinition.getVelocityUpperLimit());

      createBulletCollider(bulletPhysicsEngine);
      bulletLink.setCollider(getBulletMultiBodyLinkCollider());
   }

   public void copyDataFromSCSToBullet()
   {
      updateBulletLinkColliderTransformFromMecanoRigidBody();

      getBulletMultiBody().setJointPos(getBulletJointIndex(), (float) simRevoluteJoint.getQ());
      getBulletMultiBody().setJointVel(getBulletJointIndex(), (float) simRevoluteJoint.getQd());
      getBulletMultiBody().addJointTorque(getBulletJointIndex(), (float) simRevoluteJoint.getTau());
   }

   public void copyBulletJointDataToSCS(double dt)
   {
      float jointPosition = getBulletMultiBody().getJointPos(getBulletJointIndex());
      bulletJointVelocityTest.set((jointPosition- simRevoluteJoint.getQ())/ dt);
      simRevoluteJoint.setQ(jointPosition);
      float jointPVel = getBulletMultiBody().getJointVel(getBulletJointIndex());
      simRevoluteJoint.setQdd((jointPVel - simRevoluteJoint.getQd())/dt);
      simRevoluteJoint.setQd(jointPVel);
      
      // https://pybullet.org/Bullet/phpBB3/viewtopic.php?p=36667&hilit=btMultiBody+joint+torque#p36667
      // Assumes fixed time step. TODO: Get time of current step
      bulletJointPosition.set(jointPosition);
      bulletJointVelocity.set(jointPVel);
      bulletUserAddedTorque.set(damping.getValue() * bulletJointVelocity.getValue());
      bulletJointTorque.set(getBulletMultiBody().getJointTorque(getBulletJointIndex()));
      btVector3 linkForce = getBulletMultiBody().getLink(getBulletJointIndex()).getAppliedConstraintForce();
      bulletLinkAppliedForceX.set(linkForce.getX());
      bulletLinkAppliedForceY.set(linkForce.getY());
      bulletLinkAppliedForceZ.set(linkForce.getZ());
      btVector3 linkTorque = getBulletMultiBody().getLink(getBulletJointIndex()).getAppliedConstraintTorque();
      bulletLinkAppliedTorqueX.set(linkTorque.getX());
      bulletLinkAppliedTorqueY.set(linkTorque.getY());
      bulletLinkAppliedTorqueZ.set(linkTorque.getZ());
      
      ReferenceFrame bodyFrame = getSimRigidBody().getBodyFixedFrame();
      ReferenceFrame expressedInFrame = getSimRigidBody().getBodyFixedFrame(); //simRevoluteJoint.getFrameAfterJoint(); 
      Vector3DReadOnly torque = new Vector3D((double)linkTorque.getX(), (double)linkTorque.getY(), (double)linkTorque.getZ());
      Vector3DReadOnly force = new Vector3D((double)linkForce.getX(), (double)linkForce.getY(), (double)linkForce.getZ());
      rigidBodyWrenchRegistry.addWrench(getSimRigidBody(), new Wrench(bodyFrame, expressedInFrame, torque, force));
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

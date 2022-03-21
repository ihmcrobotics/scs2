package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.HashMap;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletRobotLinkRoot extends BulletRobotLinkBasics
{
   private int linkCountingIndex = 0;
   private final int numberOfLinks;
   private final SimFloatingRootJoint rootSimFloatingRootJoint;
   private final Matrix4 bulletSixDoFJointTransformToWorldBullet = new Matrix4();
   private final RigidBodyTransform bulletSixDoFJointTransformToWorldEuclid = new RigidBodyTransform();
   private final Vector3D bulletBaseLinearVelocityEuclid = new Vector3D();
   private final Vector3D bulletBaseAngularVelocityEuclid = new Vector3D();
   private final Pose3D previousBasePose = new Pose3D();
   private final Twist previousBaseTwist = new Twist();

   private final YoFixedFrameTwist twistFD;

   public BulletRobotLinkRoot(SixDoFJointDefinition rootSixDoFJointDefinition,
                              SimFloatingRootJoint rootSimFloatingRootJoint,
                              HashMap<String, Integer> jointNameToBulletJointIndexMap,
                              HashMap<btCollisionObject, BulletWrenchSensorCalculator> wrenchCalculatorMap,
                              YoRegistry yoRegistry)
   {
      super(rootSixDoFJointDefinition.getSuccessor(), rootSimFloatingRootJoint.getSuccessor(), jointNameToBulletJointIndexMap, wrenchCalculatorMap);
      this.rootSimFloatingRootJoint = rootSimFloatingRootJoint;

      setBulletJointIndex(-1);
      numberOfLinks = countJoints(rootSimFloatingRootJoint) - 1; // which is also number of joints in this case

      ReferenceFrame rootFrame = rootSimFloatingRootJoint.getFrameBeforeJoint().getRootFrame();
      twistFD = new YoFixedFrameTwist("testTwistFD",
                                      rootSimFloatingRootJoint.getFrameAfterJoint(),
                                      rootSimFloatingRootJoint.getFrameBeforeJoint(),
                                      rootSimFloatingRootJoint.getFrameAfterJoint(),
                                      yoRegistry);

      addChildLinks(yoRegistry);
   }

   @Override
   public void setup(BulletPhysicsEngine bulletPhysicsEngine)
   {
      boolean fixedBase = false;
      boolean canSleep = false;

      float rootBodyMass = (float) getRigidBodyDefinition().getMass();
      Vector3 rootBodyIntertia = new Vector3((float) getRigidBodyDefinition().getMomentOfInertia().getM00(),
                                             (float) getRigidBodyDefinition().getMomentOfInertia().getM11(),
                                             (float) getRigidBodyDefinition().getMomentOfInertia().getM22());
      BulletRobotLinkCollisionSet bulletCollisionSet = createBulletCollisionShape();
      // TODO: Should we let Bullet compute this?
      // bulletCollisionSet.getBulletCompoundShape().calculateLocalInertia(rootBodyMass, rootBodyIntertia);

      btMultiBody bulletMultiBody = new btMultiBody(numberOfLinks, rootBodyMass, rootBodyIntertia, fixedBase, canSleep);
      bulletMultiBody.setHasSelfCollision(false);
      bulletMultiBody.setLinearDamping(0.1f);
      bulletMultiBody.setAngularDamping(0.9f);
      setBulletMultiBody(bulletMultiBody);

      createBulletCollider(bulletPhysicsEngine);
      getBulletMultiBody().setBaseCollider(getBulletMultiBodyLinkCollider());
   }

   private final RigidBodyTransform rootJointSuccessorBodyFixedFrameToWorldEuclid = new RigidBodyTransform();
   private final Matrix4 rootJointSuccessorBodyFixedFrameToWorldBullet = new Matrix4();

   @Override
   public void copyDataFromSCSToBullet()
   {
      updateBulletLinkColliderTransformFromMecanoRigidBody();

      BulletTools.toBullet(getbulletColliderCenterOfMassTransformToWorldEuclid(), rootJointSuccessorBodyFixedFrameToWorldBullet);
      getBulletMultiBody().setBaseWorldTransform(rootJointSuccessorBodyFixedFrameToWorldBullet);
   }

   private final RigidBodyTransform bodyFixedFrameToFrameAfterJointTranform = new RigidBodyTransform();

   @Override
   public void copyBulletJointDataToSCS(double dt)
   {
      // T_BFF^W
      getBulletMultiBodyLinkCollider().getWorldTransform(rootJointSuccessorBodyFixedFrameToWorldBullet);
      BulletTools.toEuclid(rootJointSuccessorBodyFixedFrameToWorldBullet, rootJointSuccessorBodyFixedFrameToWorldEuclid);

      // T_FAJ^BFF
      rootSimFloatingRootJoint.getFrameAfterJoint().getTransformToDesiredFrame(bodyFixedFrameToFrameAfterJointTranform,
                                                                               rootSimFloatingRootJoint.getSuccessor().getBodyFixedFrame());

      // T_FAJ^W = T_BFF^W * T_FAJ^BFF
      bulletSixDoFJointTransformToWorldEuclid.set(rootJointSuccessorBodyFixedFrameToWorldEuclid);
      bulletSixDoFJointTransformToWorldEuclid.multiply(bodyFixedFrameToFrameAfterJointTranform);

      BulletTools.toEuclid(getBulletMultiBody().getBaseVel(), bulletBaseLinearVelocityEuclid); // bullet linear velocity is in world
      BulletTools.toEuclid(getBulletMultiBody().getBaseOmega(), bulletBaseAngularVelocityEuclid);

      bulletSixDoFJointTransformToWorldEuclid.inverseTransform(bulletBaseLinearVelocityEuclid);
      bulletSixDoFJointTransformToWorldEuclid.inverseTransform(bulletBaseAngularVelocityEuclid);
      
      previousBasePose.set(rootSimFloatingRootJoint.getJointPose());
      previousBaseTwist.setIncludingFrame(rootSimFloatingRootJoint.getJointTwist());

      rootSimFloatingRootJoint.setJointPosition(bulletSixDoFJointTransformToWorldEuclid.getTranslation());
      rootSimFloatingRootJoint.setJointOrientation(bulletSixDoFJointTransformToWorldEuclid.getRotation());
      rootSimFloatingRootJoint.setJointLinearVelocity(bulletBaseLinearVelocityEuclid);
      rootSimFloatingRootJoint.setJointAngularVelocity(bulletBaseAngularVelocityEuclid);

      computeSixDoFJointTwist(dt, previousBasePose, rootSimFloatingRootJoint.getJointPose(), twistFD);
      computeSixDoFJointAcceleration(dt,
                                     previousBasePose,
                                     rootSimFloatingRootJoint.getJointPose(),
                                     previousBaseTwist,
                                     rootSimFloatingRootJoint.getJointTwist(),
                                     rootSimFloatingRootJoint.getJointAcceleration());
      
      //computeSixDoFJointTwistAcceleration(dt, previousBasePose, rootSimFloatingRootJoint.getJointPose(), previousBaseTwist, rootSimFloatingRootJoint.getJointTwist(), rootSimFloatingRootJoint.getJointAcceleration());
      
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

   public static void computeSixDoFJointTwist(double dt, Pose3DReadOnly previousPose, Pose3DReadOnly currentPose, FixedFrameTwistBasics twistToPack)
   {
      twistToPack.getLinearPart().sub(currentPose.getPosition(), previousPose.getPosition());
      twistToPack.getLinearPart().scale(1.0 / dt);
      currentPose.getOrientation().inverseTransform(twistToPack.getLinearPart());
   }

   public static void computeSixDoFJointAcceleration(double dt,
                                                     Pose3DReadOnly previousPose,
                                                     Pose3DReadOnly currentPose,
                                                     TwistReadOnly previousTwist,
                                                     TwistReadOnly currentTwist,
                                                     FixedFrameSpatialAccelerationBasics accelerationToPack)
   {
      QuaternionReadOnly previousOrientation = previousPose.getOrientation();
      QuaternionReadOnly currentOrientation = currentPose.getOrientation();
      FixedFrameVector3DBasics angularAcceleration = accelerationToPack.getAngularPart();
      FixedFrameVector3DBasics linearAcceleration = accelerationToPack.getLinearPart();

      previousOrientation.transform(previousTwist.getAngularPart(), angularAcceleration);
      currentOrientation.inverseTransform(angularAcceleration); // Previous angular velocity in current pose
      angularAcceleration.sub(currentTwist.getAngularPart(), angularAcceleration); // w^curr - w^prev

      previousOrientation.transform(previousTwist.getLinearPart(), linearAcceleration);
      currentOrientation.inverseTransform(linearAcceleration); // Previous linear velocity in current pose
      linearAcceleration.sub(currentTwist.getLinearPart(), linearAcceleration); // v^curr - v^prev

      accelerationToPack.scale(1.0 / dt);
      accelerationToPack.addCrossToLinearPart(currentTwist.getLinearPart(), currentTwist.getAngularPart());
   }
   
   public static void computeSixDoFJointTwistAcceleration(double dt,
                                                          Pose3DReadOnly previousPose,
                                                          Pose3DReadOnly currentPose,
                                                          TwistReadOnly previousTwist,
                                                          FixedFrameTwistBasics twistToPack,
                                                          FixedFrameSpatialAccelerationBasics accelerationToPack)
   {
      QuaternionReadOnly previousOrientation = previousPose.getOrientation();
      QuaternionReadOnly currentOrientation = currentPose.getOrientation();
      FixedFrameVector3DBasics angularAcceleration = accelerationToPack.getAngularPart();
      FixedFrameVector3DBasics linearAcceleration = accelerationToPack.getLinearPart();
      FixedFrameVector3DBasics angularVelocity = twistToPack.getAngularPart();
      FixedFrameVector3DBasics linearVelocity = twistToPack.getLinearPart();
     
      Vector4D qDot = new Vector4D();
      // assume acceleration a is constant
      // v^next = a * t + v^init
      // x^next = 1/2 * a * t^2 + v^init * t + x^init
      // We deduce:
      // a = 2 * ( (x^next - x^init) / t^2 - v^init / t )
      // v^next = 2 * (x^next - x^init) / t - v^init
      // Let's compute v^next first
      linearVelocity.sub(currentPose.getPosition(), previousPose.getPosition());
      linearVelocity.scale(2.0 / dt);
      currentOrientation.inverseTransform(linearVelocity);
      linearVelocity.sub(previousTwist.getLinearPart());
      qDot.sub(currentOrientation, previousOrientation);
      qDot.scale(2.0 / dt);
      computeAngularVelocityInBodyFixedFrame(currentOrientation, qDot, angularVelocity);
      angularVelocity.sub(previousTwist.getAngularPart());
      // Now compute the acceleration with the relation a = (v^next - v^init) / t
      previousOrientation.transform(previousTwist.getAngularPart(), angularAcceleration);
      currentOrientation.inverseTransform(angularAcceleration); // Previous angular velocity in current pose
      angularAcceleration.sub(angularVelocity, angularAcceleration); // w^curr - w^prev
      previousOrientation.transform(previousTwist.getLinearPart(), linearAcceleration);
      currentOrientation.inverseTransform(linearAcceleration); // Previous linear velocity in current pose
      linearAcceleration.sub(linearVelocity, linearAcceleration); // v^curr - v^prev
      accelerationToPack.scale(1.0 / dt);
   }
   public static void computeAngularVelocityInBodyFixedFrame(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DBasics angularVelocityToPack)
   {
      Vector4D pureQuatForMultiply = new Vector4D();
      QuaternionTools.multiplyConjugateLeft(q, qDot, pureQuatForMultiply );
      angularVelocityToPack.set(pureQuatForMultiply.getX(), pureQuatForMultiply.getY(), pureQuatForMultiply.getZ());
      angularVelocityToPack.scale(2.0);
   }
}

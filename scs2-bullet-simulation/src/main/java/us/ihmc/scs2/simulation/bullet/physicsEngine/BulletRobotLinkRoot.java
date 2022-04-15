package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.HashMap;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletRobotLinkRoot extends BulletRobotLinkBasics
{
   private final SimFloatingRootJoint rootSimFloatingRootJoint;
   private final RigidBodyTransform bulletSixDoFJointTransformToWorldEuclid = new RigidBodyTransform();
   private final Vector3D bulletBaseLinearVelocityEuclid = new Vector3D();
   private final Vector3D bulletBaseAngularVelocityEuclid = new Vector3D();
   private final Pose3D previousBasePose = new Pose3D();
   private final Twist previousBaseTwist = new Twist();

   private final YoFixedFrameTwist twistFD;

   public BulletRobotLinkRoot(SixDoFJointDefinition rootSixDoFJointDefinition,
                              SimFloatingRootJoint rootSimFloatingRootJoint,
                              HashMap<String, Integer> jointNameToBulletJointIndexMap,
                              RigidBodyWrenchRegistry rigidBodyWrenchRegistry,
                              YoRegistry yoRegistry,
                              BulletMultiBodyRobot bulletMultiBodyRobot)
   {
      super(rootSixDoFJointDefinition.getSuccessor(), rootSimFloatingRootJoint.getSuccessor(), jointNameToBulletJointIndexMap, rigidBodyWrenchRegistry, bulletMultiBodyRobot.getBulletMultiBody().getBaseCollider());
      this.rootSimFloatingRootJoint = rootSimFloatingRootJoint;

      setBulletJointIndex(-1);

      twistFD = new YoFixedFrameTwist("testTwistFD",
                                      rootSimFloatingRootJoint.getFrameAfterJoint(),
                                      rootSimFloatingRootJoint.getFrameBeforeJoint(),
                                      rootSimFloatingRootJoint.getFrameAfterJoint(),
                                      yoRegistry);

      addChildLinks(yoRegistry, bulletMultiBodyRobot);
   }

   private final RigidBodyTransform rootJointSuccessorBodyFixedFrameToWorldEuclid = new RigidBodyTransform();
   private final Matrix4 rootJointSuccessorBodyFixedFrameToWorldBullet = new Matrix4();

   @Override
   public void copyDataFromSCSToBullet()
   {
      updateBulletLinkColliderTransformFromMecanoRigidBody();
      
      BulletTools.toBullet(getbulletColliderCenterOfMassTransformToWorldEuclid(), rootJointSuccessorBodyFixedFrameToWorldBullet);
      getBulletMultiBody().setBaseWorldTransform(rootJointSuccessorBodyFixedFrameToWorldBullet);

      MovingReferenceFrame bodyFixedFrame = rootSimFloatingRootJoint.getSuccessor().getBodyFixedFrame();
      
      Vector3D linearVelocity = new Vector3D(bodyFixedFrame.getTwistOfFrame().getLinearPart());
      bodyFixedFrame.transformFromThisToDesiredFrame(bodyFixedFrame.getRootFrame(), linearVelocity);
      Vector3 linearVelocityBullet = new Vector3();
      BulletTools.toBullet(linearVelocity, linearVelocityBullet);
      getBulletMultiBody().setBaseVel(linearVelocityBullet);
      
      Vector3D angularVelocity = new Vector3D(bodyFixedFrame.getTwistOfFrame().getAngularPart());
      bodyFixedFrame.transformFromThisToDesiredFrame(bodyFixedFrame.getRootFrame(), angularVelocity);
      Vector3 angularVelocityBullet = new Vector3();
      BulletTools.toBullet(angularVelocity, angularVelocityBullet);
      getBulletMultiBody().setBaseOmega(angularVelocityBullet);
   }

   private final RigidBodyTransform frameAfterJointToBodyFixedFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform frameBodyFixedFrameTransformToFrameAfterTransfer = new RigidBodyTransform();
   private final Point3D point = new Point3D();

   @Override
   public void copyBulletJointDataToSCS(double dt)
   {
      // T_BFF^W
      getBulletMultiBodyLinkCollider().getWorldTransform(rootJointSuccessorBodyFixedFrameToWorldBullet);
      BulletTools.toEuclid(rootJointSuccessorBodyFixedFrameToWorldBullet, rootJointSuccessorBodyFixedFrameToWorldEuclid);

      // T_FAJ^BFF
      rootSimFloatingRootJoint.getFrameAfterJoint().getTransformToDesiredFrame(frameAfterJointToBodyFixedFrameTransform,
                                                                               rootSimFloatingRootJoint.getSuccessor().getBodyFixedFrame());

      // T_FAJ^W = T_BFF^W * T_FAJ^BFF
      bulletSixDoFJointTransformToWorldEuclid.set(rootJointSuccessorBodyFixedFrameToWorldEuclid);
      bulletSixDoFJointTransformToWorldEuclid.multiply(frameAfterJointToBodyFixedFrameTransform);

      BulletTools.toEuclid(getBulletMultiBody().getBaseVel(), bulletBaseLinearVelocityEuclid); // bullet linear velocity of BFF is in world
      BulletTools.toEuclid(getBulletMultiBody().getBaseOmega(), bulletBaseAngularVelocityEuclid); // bullet angular velocity of BFF is in world

      bulletSixDoFJointTransformToWorldEuclid.inverseTransform(bulletBaseLinearVelocityEuclid);
      bulletSixDoFJointTransformToWorldEuclid.inverseTransform(bulletBaseAngularVelocityEuclid);
      
      previousBasePose.set(rootSimFloatingRootJoint.getJointPose());
      previousBaseTwist.setIncludingFrame(rootSimFloatingRootJoint.getJointTwist());
      
      rootSimFloatingRootJoint.setJointPosition(bulletSixDoFJointTransformToWorldEuclid.getTranslation());
      rootSimFloatingRootJoint.setJointOrientation(bulletSixDoFJointTransformToWorldEuclid.getRotation());

      // SCS2 Linear velocity - of FAJ - expressed in FAJ
      frameBodyFixedFrameTransformToFrameAfterTransfer.set(frameAfterJointToBodyFixedFrameTransform.getRotation(), frameAfterJointToBodyFixedFrameTransform.getTranslation());
      rootSimFloatingRootJoint.getSuccessor().getBodyFixedFrame().getTransformToDesiredFrame(frameBodyFixedFrameTransformToFrameAfterTransfer
            , rootSimFloatingRootJoint.getFrameAfterJoint());

      point.set(frameBodyFixedFrameTransformToFrameAfterTransfer.getTranslation().getX(), frameBodyFixedFrameTransformToFrameAfterTransfer.getTranslation().getY(), frameBodyFixedFrameTransformToFrameAfterTransfer.getTranslation().getZ());
      rootSimFloatingRootJoint.getJointTwist().set(bulletBaseAngularVelocityEuclid, bulletBaseLinearVelocityEuclid, point);

      computeSixDoFJointTwist(dt, previousBasePose, rootSimFloatingRootJoint.getJointPose(), twistFD);
      computeSixDoFJointAcceleration(dt,
                                     previousBasePose,
                                     rootSimFloatingRootJoint.getJointPose(),
                                     previousBaseTwist,
                                     rootSimFloatingRootJoint.getJointTwist(),
                                     rootSimFloatingRootJoint.getJointAcceleration());
   }

   public static void computeSixDoFJointTwist(double dt, Pose3DReadOnly previousPose, Pose3DReadOnly currentPose, FixedFrameTwistBasics twistToPack)
   {
      twistToPack.getLinearPart().sub(currentPose.getPosition(), previousPose.getPosition());
      twistToPack.getLinearPart().scale(1.0 / dt);
      currentPose.getOrientation().inverseTransform(twistToPack.getLinearPart());
      
      Vector4D qDot = new Vector4D();
      qDot.sub(currentPose.getOrientation(), previousPose.getOrientation());
      qDot.scale(1.0 / dt);
      computeAngularVelocityInBodyFixedFrame(currentPose.getOrientation(), qDot, twistToPack.getAngularPart());
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

   public static void computeAngularVelocityInBodyFixedFrame(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DBasics angularVelocityToPack)
   {
      Vector4D pureQuatForMultiply = new Vector4D();
      QuaternionTools.multiplyConjugateLeft(q, qDot, pureQuatForMultiply );
      angularVelocityToPack.set(pureQuatForMultiply.getX(), pureQuatForMultiply.getY(), pureQuatForMultiply.getZ());
      angularVelocityToPack.scale(2.0);
   }
}

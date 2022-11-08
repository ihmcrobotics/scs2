package us.ihmc.scs2.simulation.bullet.physicsEngine;

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
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletRobotLinkRoot extends BulletRobotLinkBasics
{
   private final SimFloatingRootJoint rootSimFloatingRootJoint;
   private final RigidBodyTransform bulletJointTransformToWorldEuclid = new RigidBodyTransform();
   private final YoFixedFrameTwist twistFD;

   public BulletRobotLinkRoot(SimFloatingRootJoint rootSimFloatingRootJoint,
                              RigidBodyWrenchRegistry rigidBodyWrenchRegistry,
                              YoRegistry yoRegistry,
                              BulletMultiBodyLinkCollider multiBodyLinkCollider)
   {
      super(rootSimFloatingRootJoint.getSuccessor(), rigidBodyWrenchRegistry, multiBodyLinkCollider);
      this.rootSimFloatingRootJoint = rootSimFloatingRootJoint;

      twistFD = new YoFixedFrameTwist("testTwistFD",
                                      rootSimFloatingRootJoint.getFrameAfterJoint(),
                                      rootSimFloatingRootJoint.getFrameBeforeJoint(),
                                      rootSimFloatingRootJoint.getFrameAfterJoint(),
                                      yoRegistry);

   }

   private final Vector3D linearVelocity = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();

   @Override
   public void pushStateToBullet()
   {
      updateBulletLinkColliderTransformFromMecanoRigidBody();

      MovingReferenceFrame bodyFixedFrame = rootSimFloatingRootJoint.getSuccessor().getBodyFixedFrame();

      linearVelocity.set(bodyFixedFrame.getTwistOfFrame().getLinearPart());
      bodyFixedFrame.transformFromThisToDesiredFrame(bodyFixedFrame.getRootFrame(), linearVelocity);
      getBulletMultiBodyLinkCollider().setBaseVel(linearVelocity);

      angularVelocity.set(bodyFixedFrame.getTwistOfFrame().getAngularPart());
      bodyFixedFrame.transformFromThisToDesiredFrame(bodyFixedFrame.getRootFrame(), angularVelocity);
      getBulletMultiBodyLinkCollider().setBaseOmega(angularVelocity);
   }

   private final RigidBodyTransform rootJointSuccessorBodyFixedFrameToWorldEuclid = new RigidBodyTransform();
   private final RigidBodyTransform frameAfterJointToBodyFixedFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform frameBodyFixedFrameTransformToFrameAfterTransfer = new RigidBodyTransform();
   private final Point3D point = new Point3D();
   private final Vector3D bulletBaseLinearVelocityEuclid = new Vector3D();
   private final Vector3D bulletBaseAngularVelocityEuclid = new Vector3D();
   private final Pose3D previousBasePose = new Pose3D();
   private final Twist previousBaseTwist = new Twist();

   @Override
   public void pullStateFromBullet(double dt)
   {
      // T_BFF^W
      getBulletMultiBodyLinkCollider().getWorldTransform(rootJointSuccessorBodyFixedFrameToWorldEuclid);

      // T_FAJ^BFF
      rootSimFloatingRootJoint.getFrameAfterJoint().getTransformToDesiredFrame(frameAfterJointToBodyFixedFrameTransform,
                                                                               rootSimFloatingRootJoint.getSuccessor().getBodyFixedFrame());

      // T_FAJ^W = T_BFF^W * T_FAJ^BFF
      bulletJointTransformToWorldEuclid.set(rootJointSuccessorBodyFixedFrameToWorldEuclid);
      bulletJointTransformToWorldEuclid.multiply(frameAfterJointToBodyFixedFrameTransform);

      getBulletMultiBodyLinkCollider().getBaseVel(bulletBaseLinearVelocityEuclid); // bullet linear velocity of BFF is in world
      getBulletMultiBodyLinkCollider().getBaseOmega(bulletBaseAngularVelocityEuclid); // bullet angular velocity of BFF is in world

      bulletJointTransformToWorldEuclid.inverseTransform(bulletBaseLinearVelocityEuclid);
      bulletJointTransformToWorldEuclid.inverseTransform(bulletBaseAngularVelocityEuclid);

      previousBasePose.set(rootSimFloatingRootJoint.getJointPose());
      previousBaseTwist.setIncludingFrame(rootSimFloatingRootJoint.getJointTwist());

      rootSimFloatingRootJoint.setJointPosition(bulletJointTransformToWorldEuclid.getTranslation());
      rootSimFloatingRootJoint.setJointOrientation(bulletJointTransformToWorldEuclid.getRotation());

      // SCS2 Linear velocity - of FAJ - expressed in FAJ
      frameBodyFixedFrameTransformToFrameAfterTransfer.set(frameAfterJointToBodyFixedFrameTransform.getRotation(),
                                                           frameAfterJointToBodyFixedFrameTransform.getTranslation());
      rootSimFloatingRootJoint.getSuccessor().getBodyFixedFrame().getTransformToDesiredFrame(frameBodyFixedFrameTransformToFrameAfterTransfer,
                                                                                             rootSimFloatingRootJoint.getFrameAfterJoint());

      point.set(frameBodyFixedFrameTransformToFrameAfterTransfer.getTranslation().getX(),
                frameBodyFixedFrameTransformToFrameAfterTransfer.getTranslation().getY(),
                frameBodyFixedFrameTransformToFrameAfterTransfer.getTranslation().getZ());
      rootSimFloatingRootJoint.getJointTwist().set(bulletBaseAngularVelocityEuclid, bulletBaseLinearVelocityEuclid, point);

      computeJointTwist(dt, previousBasePose, rootSimFloatingRootJoint.getJointPose(), twistFD);
      computeJointAcceleration(dt,
                               previousBasePose,
                               rootSimFloatingRootJoint.getJointPose(),
                               previousBaseTwist,
                               rootSimFloatingRootJoint.getJointTwist(),
                               rootSimFloatingRootJoint.getJointAcceleration());
   }

   private final Vector4D qDot = new Vector4D();

   public void computeJointTwist(double dt, Pose3DReadOnly previousPose, Pose3DReadOnly currentPose, FixedFrameTwistBasics twistToPack)
   {
      twistToPack.getLinearPart().sub(currentPose.getPosition(), previousPose.getPosition());
      twistToPack.getLinearPart().scale(1.0 / dt);
      currentPose.getOrientation().inverseTransform(twistToPack.getLinearPart());

      qDot.sub(currentPose.getOrientation(), previousPose.getOrientation());
      qDot.scale(1.0 / dt);
      computeAngularVelocityInBodyFixedFrame(currentPose.getOrientation(), qDot, twistToPack.getAngularPart());
   }

   public static void computeJointAcceleration(double dt,
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

   private final Vector4D pureQuatForMultiply = new Vector4D();

   public void computeAngularVelocityInBodyFixedFrame(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DBasics angularVelocityToPack)
   {
      QuaternionTools.multiplyConjugateLeft(q, qDot, pureQuatForMultiply);
      angularVelocityToPack.set(pureQuatForMultiply.getX(), pureQuatForMultiply.getY(), pureQuatForMultiply.getZ());
      angularVelocityToPack.scale(2.0);
   }
}

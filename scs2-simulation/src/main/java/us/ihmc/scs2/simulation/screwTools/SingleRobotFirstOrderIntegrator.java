package us.ihmc.scs2.simulation.screwTools;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FixedJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFixedJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimSphericalJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimMultiBodySystemBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;

public class SingleRobotFirstOrderIntegrator
{
   /** Intermediate variable used to perform garbage-free operations. */
   private final Vector3D deltaPosition = new Vector3D();
   /** Intermediate variable used to perform garbage-free operations. */
   private final Vector3D rotationVector = new Vector3D();
   /** Intermediate variable used to perform garbage-free operations. */
   private final Quaternion orientationChange = new Quaternion();
   /** Intermediate variable used to perform garbage-free operations. */
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   /** Intermediate variable used to perform garbage-free operations. */
   private final SpatialVector spatialVelocityChange = new SpatialVector();
   /** Intermediate variable used to perform garbage-free operations. */
   private final FrameVector3D angularVelocityChange = new FrameVector3D();

   public SingleRobotFirstOrderIntegrator()
   {
   }

   public void integrate(double dt, SimMultiBodySystemBasics input)
   {
      List<? extends SimJointBasics> jointsToConsider = input.getJointsToConsider();

      for (SimJointBasics joint : jointsToConsider)
      {
         if (joint instanceof SimOneDoFJointBasics)
         {
            integrateOneDoFJoint(dt, (SimOneDoFJointBasics) joint);
         }
         else if (joint instanceof SimFloatingJointBasics)
         {
            integrateFloatingJoint(dt, (SimFloatingJointBasics) joint);
         }
         else if (joint instanceof SimSphericalJoint)
         {
            integrateSphericalJoint(dt, (SimSphericalJoint) joint);
         }
         else if (joint instanceof SimFixedJoint)
         {
            // Do nothing
         }
         else
         {
            throw new UnsupportedOperationException("Unsupported joint " + joint);
         }
      }
   }

   public void integrate(double dt, DMatrixRMaj velocityChangeMatrix, MultiBodySystemBasics input)
   {
      List<? extends JointBasics> jointsToConsider = input.getJointsToConsider();
      int startIndex = 0;

      for (JointBasics joint : jointsToConsider)
      {
         // TODO Implements for other joints
         if (joint instanceof OneDoFJointBasics)
         {
            double velocityChange;
            if (velocityChangeMatrix == null)
               velocityChange = 0.0;
            else
               velocityChange = velocityChangeMatrix.get(startIndex);
            integrateOneDoFJoint(dt, (OneDoFJointBasics) joint, velocityChange);
         }
         else if (joint instanceof FloatingJointBasics)
         {
            if (velocityChangeMatrix == null)
               spatialVelocityChange.setToZero(joint.getFrameAfterJoint());
            else
               spatialVelocityChange.setIncludingFrame(joint.getFrameAfterJoint(), startIndex, velocityChangeMatrix);
            integrateFloatingJoint(dt, (FloatingJointBasics) joint, spatialVelocityChange);
         }
         else if (joint instanceof SphericalJointBasics)
         {
            if (velocityChangeMatrix == null)
               angularVelocityChange.setToZero(joint.getFrameAfterJoint());
            else
               angularVelocityChange.setIncludingFrame(joint.getFrameAfterJoint(), startIndex, velocityChangeMatrix);
            integrateSphericalJoint(dt, (SphericalJointBasics) joint, angularVelocityChange);
         }
         else if (joint instanceof FixedJointBasics)
         {
            // Do nothing
         }
         else
         {
            throw new UnsupportedOperationException("Unsupported joint " + joint);
         }
         startIndex += joint.getDegreesOfFreedom();
      }
   }

   public void integrateOneDoFJoint(double dt, SimOneDoFJointBasics joint)
   {
      if (!joint.isPinned())
         integrateOneDoFJoint(dt, joint, joint.getDeltaQd());
   }

   public void integrateOneDoFJoint(double dt, OneDoFJointBasics joint, double velocityChange)
   {
      double qdd = joint.getQdd() /* + velocityChange / dt */;
      double qd = joint.getQd() + qdd * dt + velocityChange;
      double q = joint.getQ() + (joint.getQd() + 0.5 * velocityChange) * dt + 0.5 * joint.getQdd() * dt * dt;
      joint.setQ(q);
      joint.setQd(qd);
      joint.setQdd(qdd);
   }

   public void integrateSphericalJoint(double dt, SimSphericalJoint joint)
   {
      if (!joint.isPinned())
         integrateSphericalJoint(dt, joint, joint.getJointAngularDeltaVelocity());
   }

   public void integrateSphericalJoint(double dt, SphericalJointBasics joint, FrameVector3DReadOnly angularVelocityChange)
   {
      QuaternionBasics orientation = joint.getJointOrientation();
      FixedFrameVector3DBasics angularVelocity = joint.getJointAngularVelocity();
      FixedFrameVector3DBasics angularAcceleration = joint.getJointAngularAcceleration();

      rotationVector.setAndScale(dt, angularVelocity);
      rotationVector.scaleAdd(0.5 * dt, angularVelocityChange, rotationVector);
      rotationVector.scaleAdd(0.5 * dt * dt, angularAcceleration, rotationVector);
      orientationChange.setRotationVector(rotationVector);

      angularVelocity.scaleAdd(dt, angularAcceleration, angularVelocity);
      angularVelocity.add(angularVelocityChange);

      orientation.append(orientationChange);
   }

   public void integrateFloatingJoint(double dt, SimFloatingJointBasics joint)
   {
      if (!joint.isPinned())
         integrateFloatingJoint(dt, joint, joint.getJointDeltaTwist());
   }

   public void integrateFloatingJoint(double dt, FloatingJointBasics joint, SpatialVectorReadOnly spatialVelocityChange)
   {
      FrameVector3DReadOnly angularVelocityChange = spatialVelocityChange.getAngularPart();
      FrameVector3DReadOnly linearVelocityChange = spatialVelocityChange.getLinearPart();

      Pose3DBasics pose = joint.getJointPose();
      QuaternionBasics orientation = pose.getOrientation();
      Point3DBasics position = pose.getPosition();

      FixedFrameTwistBasics twist = joint.getJointTwist();
      FixedFrameVector3DBasics angularVelocity = twist.getAngularPart();
      FixedFrameVector3DBasics linearVelocity = twist.getLinearPart();

      FixedFrameSpatialAccelerationBasics spatialAcceleration = joint.getJointAcceleration();
      FixedFrameVector3DBasics angularAcceleration = spatialAcceleration.getAngularPart();

      spatialAcceleration.getLinearAccelerationAtBodyOrigin(twist, linearAcceleration);

      rotationVector.setAndScale(dt, angularVelocity);
      rotationVector.scaleAdd(0.5 * dt, angularVelocityChange, rotationVector);
      rotationVector.scaleAdd(0.5 * dt * dt, angularAcceleration, rotationVector);
      orientationChange.setRotationVector(rotationVector);

      angularVelocity.scaleAdd(dt, angularAcceleration, angularVelocity);
      angularVelocity.add(angularVelocityChange);

      deltaPosition.setAndScale(dt, linearVelocity);
      deltaPosition.scaleAdd(0.5 * dt, linearVelocityChange, deltaPosition);
      deltaPosition.scaleAdd(0.5 * dt * dt, linearAcceleration, deltaPosition);
      orientation.transform(deltaPosition);
      position.add(deltaPosition);

      linearVelocity.scaleAdd(dt, linearAcceleration, linearVelocity);
      linearVelocity.add(linearVelocityChange);
      orientationChange.inverseTransform(linearVelocity);

      orientationChange.inverseTransform(linearAcceleration);
      spatialAcceleration.setBasedOnOriginAcceleration(angularAcceleration, linearAcceleration, twist);

      orientation.append(orientationChange);
   }
}

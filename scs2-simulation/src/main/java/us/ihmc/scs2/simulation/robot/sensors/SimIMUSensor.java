package us.ihmc.scs2.simulation.robot.sensors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimIMUSensor extends SimSensor
{
   private final YoFrameQuaternion orientation;
   private final YoFrameVector3D angularVelocity;
   private final YoFrameVector3D linearAcceleration;

   public SimIMUSensor(IMUSensorDefinition definition, SimJointBasics parentJoint)
   {
      this(definition.getName(), parentJoint, definition.getTransformToJoint());
   }

   public SimIMUSensor(String name, SimJointBasics parentJoint, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, parentJoint, transformToParent);
      ReferenceFrame rootFrame = parentJoint.getFrameAfterJoint().getRootFrame();
      YoRegistry registry = parentJoint.getRegistry();
      orientation = new YoFrameQuaternion(parentJoint.getName() + name + "Orientation", rootFrame, registry);
      angularVelocity = new YoFrameVector3D(parentJoint.getName() + name + "AngularVelocity", getFrame(), registry);
      linearAcceleration = new YoFrameVector3D(parentJoint.getName() + name + "LinearAcceleration", getFrame(), registry);
   }

   private final FramePoint3D bodyFixedPoint = new FramePoint3D();
   private final FrameVector3D intermediateAcceleration = new FrameVector3D();

   @Override
   public void update(RobotPhysicsOutput robotPhysicsOutput)
   {
      super.update(robotPhysicsOutput);
      orientation.setFromReferenceFrame(getFrame());
      angularVelocity.set(getFrame().getTwistOfFrame().getAngularPart());

      double dt = robotPhysicsOutput.getDT();
      RigidBodyTwistProvider deltaTwistProvider = robotPhysicsOutput.getDeltaTwistProvider();
      RigidBodyAccelerationProvider accelerationProvider = robotPhysicsOutput.getAccelerationProvider();
      SimRigidBodyBasics body = getParentJoint().getSuccessor();
      MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();

      bodyFixedPoint.setIncludingFrame(getOffset().getPosition());
      bodyFixedPoint.changeFrame(bodyFrame);
      accelerationProvider.getAccelerationOfBody(body).getLinearAccelerationAt(bodyFrame.getTwistOfFrame(), bodyFixedPoint, intermediateAcceleration);
      if (dt != 0.0 && deltaTwistProvider != null) // This can happen at initialization
         intermediateAcceleration.scaleAdd(1.0 / dt, deltaTwistProvider.getLinearVelocityOfBodyFixedPoint(body, bodyFixedPoint), intermediateAcceleration);
      linearAcceleration.setMatchingFrame(intermediateAcceleration);
   }

   public YoFrameQuaternion getOrientation()
   {
      return orientation;
   }

   public YoFrameVector3D getAngularVelocity()
   {
      return angularVelocity;
   }

   public YoFrameVector3D getLinearAcceleration()
   {
      return linearAcceleration;
   }
}

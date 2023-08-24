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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimIMUSensor extends SimSensor
{
   private final YoFrameQuaternion orientation;
   private final YoFrameVector3D angularVelocity;
   private final YoFrameVector3D linearAcceleration;

   private final YoDouble filterBreakFrequency;
   private final YoBoolean filterInitialized;
   private final YoFrameQuaternion orientationFiltered;
   private final YoFrameVector3D angularVelocityFiltered;
   private final YoFrameVector3D linearAccelerationFiltered;

   public SimIMUSensor(IMUSensorDefinition definition, SimJointBasics parentJoint)
   {
      this(definition.getName(), parentJoint, definition.getTransformToJoint());
      setSamplingRate(toSamplingRate(definition.getUpdatePeriod()));
   }

   public SimIMUSensor(String name, SimJointBasics parentJoint, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, parentJoint, transformToParent);
      ReferenceFrame rootFrame = parentJoint.getFrameAfterJoint().getRootFrame();
      YoRegistry registry = parentJoint.getRegistry();
      orientation = new YoFrameQuaternion(name + "Orientation", rootFrame, registry);
      angularVelocity = new YoFrameVector3D(name + "AngularVelocity", getFrame(), registry);
      linearAcceleration = new YoFrameVector3D(name + "LinearAcceleration", getFrame(), registry);

      filterBreakFrequency = new YoDouble(name + "FilterBreakFrequency", registry);
      filterBreakFrequency.set(Double.POSITIVE_INFINITY);
      getSamplingRate().addListener(v -> filterBreakFrequency.set(getSamplingRate().getValue() * ANTIALIASING_FILTER_RATIO));
      filterInitialized = new YoBoolean(name + "FilterInitialized", registry);
      orientationFiltered = new YoFrameQuaternion(name + "OrientationFiltered", rootFrame, registry);
      angularVelocityFiltered = new YoFrameVector3D(name + "AngularVelocityFiltered", getFrame(), registry);
      linearAccelerationFiltered = new YoFrameVector3D(name + "LinearAccelerationFiltered", getFrame(), registry);
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
      // TODO This operation add "v x w" to the acceleration which doesn't seem right as the acceleration is measured locally and not in the world frame.
      accelerationProvider.getAccelerationOfBody(body).getLinearAccelerationAt(bodyFrame.getTwistOfFrame(), bodyFixedPoint, intermediateAcceleration);
      if (dt != 0.0 && deltaTwistProvider != null) // This can happen at initialization
         intermediateAcceleration.scaleAdd(1.0 / dt, deltaTwistProvider.getLinearVelocityOfBodyFixedPoint(body, bodyFixedPoint), intermediateAcceleration);
      linearAcceleration.setMatchingFrame(intermediateAcceleration);

      if (dt <= 0.0)
      { // Initialize
         filterInitialized.set(false);
      }

      if (!filterInitialized.getValue())
      {
         orientationFiltered.set(orientation);
         angularVelocityFiltered.set(angularVelocity);
         linearAccelerationFiltered.set(linearAcceleration);
         filterInitialized.set(true);
      }
      else
      {
         if (Double.isInfinite(filterBreakFrequency.getValue()))
            filterBreakFrequency.set(ANTIALIASING_FILTER_RATIO / dt);
         double alpha = 1.0 - computeLowPassFilterAlpha(filterBreakFrequency.getValue(), dt);
         orientationFiltered.interpolate(orientation, alpha);
         angularVelocityFiltered.interpolate(angularVelocity, alpha);
         linearAccelerationFiltered.interpolate(linearAcceleration, alpha);
      }
   }

   /**
    * Overrides the default break frequency used to compute the filtered measurements.
    * 
    * @param breakFrequency the new break frequency.
    */
   public void setFilterBreakFrequency(double breakFrequency)
   {
      filterBreakFrequency.set(breakFrequency);
   }

   /**
    * Gets the raw measured orientation.
    * <p>
    * It is recommended to use the filtered measurement which provides anti-aliasing.
    * </p>
    * 
    * @return the raw orientation measurement.
    * @see #getOrientationFiltered()
    */
   public YoFrameQuaternion getOrientation()
   {
      return orientation;
   }

   /**
    * Gets the raw measured angular velocity.
    * <p>
    * It is recommended to use the filtered measurement which provides anti-aliasing.
    * </p>
    * 
    * @return the raw angular velocity measurement.
    * @see #getAngularVelocityFiltered()
    */
   public YoFrameVector3D getAngularVelocity()
   {
      return angularVelocity;
   }

   /**
    * Gets the raw measured linear acceleration measurement.
    * <p>
    * It is recommended to use the filtered measurement which provides anti-aliasing.
    * </p>
    * 
    * @return the raw linear acceleration measurement.
    * @see #getLinearAccelerationFiltered()
    */
   public YoFrameVector3D getLinearAcceleration()
   {
      return linearAcceleration;
   }

   /**
    * Gets the break frequency variable used to compute the filtered measurements.
    * 
    * @return the filter break frequency.
    */
   public YoDouble getFilterBreakFrequency()
   {
      return filterBreakFrequency;
   }

   /**
    * Gets the variable indicating whether the filtered measurements have been initialized or not.
    * 
    * @return the filter initialized variable.
    */
   public YoBoolean getFilterInitialized()
   {
      return filterInitialized;
   }

   /**
    * Gets the filtered orientation measurement.
    * 
    * @return the filtered orientation measurement.
    */
   public YoFrameQuaternion getOrientationFiltered()
   {
      return orientationFiltered;
   }

   /**
    * Gets the filtered angular velocity measurement.
    * 
    * @return the filtered angular velocity measurement.
    */
   public YoFrameVector3D getAngularVelocityFiltered()
   {
      return angularVelocityFiltered;
   }

   /**
    * Gets the filtered linear acceleration measurement.
    * 
    * @return the filtered linear acceleration measurement.
    */
   public YoFrameVector3D getLinearAccelerationFiltered()
   {
      return linearAccelerationFiltered;
   }
}

package us.ihmc.scs2.simulation.robot.sensors;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.session.YoFixedMovingReferenceFrameUsingYawPitchRoll;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class SimSensor
{
   private final String name;
   private final SimJointBasics parentJoint;
   private final YoFixedMovingReferenceFrameUsingYawPitchRoll frame;

   private final YoDouble samplingRate;
   /**
    * Ratio used to compute a default filter break frequency given the sensor sampling rate of
    * simulation rate.
    * <p>
    * This low-pass filter serves as an anti-aliasing filter particularly useful when the simulation
    * rate is greater than the controller rate.
    * </p>
    */
   public static final double ANTIALIASING_FILTER_RATIO = 0.25;

   /**
    * Computes the alpha factor that is used to update a discrete low-pass filter.
    * 
    * @param breakFrequency the desired cut-off frequency of the low-pass filter.
    * @param dt             the update period of the filter.
    * @return the alpha factor &in; [0, 1].
    */
   public static double computeLowPassFilterAlpha(double breakFrequency, double dt)
   {
      if (Double.isInfinite(breakFrequency))
         return 0.0;

      double halfOmegaDT = Math.PI * breakFrequency * dt;
      return MathTools.clamp((1.0 - halfOmegaDT) / (1.0 + halfOmegaDT), 0.0, 1.0);
   }

   public SimSensor(SensorDefinition definition, SimJointBasics parentJoint)
   {
      this(definition.getName(), parentJoint, definition.getTransformToJoint());
      setSamplingRate(toSamplingRate(definition.getUpdatePeriod()));
   }

   public SimSensor(String name, SimJointBasics parentJoint, RigidBodyTransformReadOnly transformToParent)
   {
      this.name = name;
      this.parentJoint = parentJoint;

      frame = new YoFixedMovingReferenceFrameUsingYawPitchRoll(name + "Frame", name + "Offset", parentJoint.getFrameAfterJoint(), parentJoint.getRegistry());
      frame.getOffset().set(transformToParent);
      this.samplingRate = new YoDouble(name + "SamplingRate", parentJoint.getRegistry());
      this.samplingRate.set(Double.POSITIVE_INFINITY);
   }

   /**
    * Transform the update period in millisecond to a sampling rate in Hz.
    * 
    * @param updatePeriod the sensor's update period in milliseconds.
    * @return the sensor's update rate in Hz, {@link Double#POSITIVE_INFINITY} if the given period is
    *         less-or-equal to 0.
    */
   public static double toSamplingRate(int updatePeriod)
   {
      if (updatePeriod <= 0)
         return Double.POSITIVE_INFINITY;
      else
         return 1000.0 / updatePeriod;
   }

   /**
    * Called at the end of the physics engine's update to indicate this sensor should update its
    * readings.
    * 
    * @param robotPhysicsOutput the output from the physics engine.
    */
   public void update(RobotPhysicsOutput robotPhysicsOutput)
   {
      frame.update();
   }

   /**
    * Sets this sensor's update rate in Hz.
    * 
    * @param samplingRate the new sampling rate in Hz.
    */
   public void setSamplingRate(double samplingRate)
   {
      this.samplingRate.set(samplingRate);
   }

   /**
    * The name of this sensor.
    * 
    * @return this sensor's name.
    */
   public String getName()
   {
      return name;
   }

   /**
    * The joint to which this sensor is attached.
    * 
    * @return this sensor's parent joint.
    */
   public SimJointBasics getParentJoint()
   {
      return parentJoint;
   }

   /**
    * The reference frame in which this sensor is collecting data.
    * 
    * @return the sensor frame.
    */
   public MovingReferenceFrame getFrame()
   {
      return frame;
   }

   /**
    * The sensor frame offset from the parent joint.
    * 
    * @return the sensor frame offset.
    */
   public YoFramePoseUsingYawPitchRoll getOffset()
   {
      return frame.getOffset();
   }

   /**
    * The update rate for this sensor.
    * 
    * @return this sensor sampling rate.
    */
   public YoDouble getSamplingRate()
   {
      return samplingRate;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + " - " + getName();
   }
}

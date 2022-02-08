package us.ihmc.scs2.simulation.robot.sensors;

import java.util.function.Function;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimWrenchSensor extends SimSensor
{
   private final YoFixedFrameWrench wrench;

   private final YoDouble filterBreakFrequency;
   private final YoBoolean filterInitialized;
   private final YoFixedFrameWrench wrenchFiltered;

   public SimWrenchSensor(WrenchSensorDefinition definition, SimJointBasics parentJoint)
   {
      this(definition.getName(), parentJoint, definition.getTransformToJoint());
      setSamplingRate(toSamplingRate(definition.getUpdatePeriod()));
   }

   public SimWrenchSensor(String name, SimJointBasics parentJoint, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, parentJoint, transformToParent);

      YoRegistry registry = parentJoint.getRegistry();
      wrench = new YoFixedFrameWrench(parentJoint.getSuccessor().getBodyFixedFrame(),
                                      new YoFrameVector3D(name + "Moment", getFrame(), registry),
                                      new YoFrameVector3D(name + "Force", getFrame(), registry));

      filterBreakFrequency = new YoDouble(parentJoint.getName() + name + "FilterBreakFrequency", registry);
      filterBreakFrequency.set(Double.POSITIVE_INFINITY);
      getSamplingRate().addListener(v -> filterBreakFrequency.set(getSamplingRate().getValue() * ANTIALIASING_FILTER_RATIO));
      filterInitialized = new YoBoolean(parentJoint.getName() + name + "FilterInitialized", registry);
      wrenchFiltered = new YoFixedFrameWrench(parentJoint.getSuccessor().getBodyFixedFrame(),
                                              new YoFrameVector3D(name + "MomentFiltered", getFrame(), registry),
                                              new YoFrameVector3D(name + "ForceFiltered", getFrame(), registry));
   }

   private final Wrench intermediateWrench = new Wrench();

   @Override
   public void update(RobotPhysicsOutput robotPhysicsOutput)
   {
      super.update(robotPhysicsOutput);

      SimRigidBodyBasics body = getParentJoint().getSuccessor();
      Function<RigidBodyReadOnly, WrenchReadOnly> externalWrenchProvider = robotPhysicsOutput.getExternalWrenchProvider();
      Function<RigidBodyReadOnly, SpatialImpulseReadOnly> externalImpulseProvider = robotPhysicsOutput.getExternalImpulseProvider();

      double dt = robotPhysicsOutput.getDT();
      WrenchReadOnly externalWrench = externalWrenchProvider == null ? null : externalWrenchProvider.apply(body);
      SpatialImpulseReadOnly externalImpulse = externalImpulseProvider == null ? null : externalImpulseProvider.apply(body);

      if (dt != 0.0 && externalImpulse != null)
      {
         wrench.setMatchingFrame(externalImpulse);
         wrench.scale(1.0 / dt);

         if (externalWrench != null)
         {
            intermediateWrench.setIncludingFrame(externalWrench);
            intermediateWrench.changeFrame(getFrame());
            wrench.add(intermediateWrench);
         }
      }
      else if (externalWrench != null)
      {
         wrench.setMatchingFrame(externalWrench);
      }
      else
      {
         wrench.setToZero();
      }

      if (dt <= 0.0)
      { // Initialize
         filterInitialized.set(false);
      }

      if (!filterInitialized.getValue())
      {
         wrenchFiltered.set(wrench);
      }
      else
      {
         if (Double.isInfinite(filterBreakFrequency.getValue()))
            filterBreakFrequency.set(ANTIALIASING_FILTER_RATIO / dt);
         double alpha = 1.0 - computeLowPassFilterAlpha(filterBreakFrequency.getValue(), dt);
         wrenchFiltered.interpolate(wrench, alpha);
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
    * Gets the raw measured wrench.
    * <p>
    * It is recommended to use the filtered measurement which provides anti-aliasing.
    * </p>
    * 
    * @return the raw wrench measurement.
    * @see #getWrenchFiltered()
    */
   public YoFixedFrameWrench getWrench()
   {
      return wrench;
   }

   /**
    * Gets the filtered wrench measurement.
    * 
    * @return the filtered wrench measurement.
    */
   public YoFixedFrameWrench getWrenchFiltered()
   {
      return wrenchFiltered;
   }
}

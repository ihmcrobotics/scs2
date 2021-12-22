package us.ihmc.scs2.simulation.robot.sensors;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.session.YoFixedMovingReferenceFrameUsingYawPitchRoll;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;

public abstract class SimSensor
{
   private final String name;
   private final SimJointBasics parentJoint;
   private final YoFixedMovingReferenceFrameUsingYawPitchRoll frame;

   public SimSensor(SensorDefinition definition, SimJointBasics parentJoint)
   {
      this(definition.getName(), parentJoint, definition.getTransformToJoint());
   }

   public SimSensor(String name, SimJointBasics parentJoint, RigidBodyTransformReadOnly transformToParent)
   {
      this.name = name;
      this.parentJoint = parentJoint;

      frame = new YoFixedMovingReferenceFrameUsingYawPitchRoll(name + "Frame", name + "Offset", parentJoint.getFrameAfterJoint(), parentJoint.getRegistry());
      frame.getOffset().set(transformToParent);
   }

   public void update(RobotPhysicsOutput robotPhysicsOutput)
   {
      frame.update();
   }

   public String getName()
   {
      return name;
   }

   public SimJointBasics getParentJoint()
   {
      return parentJoint;
   }

   public MovingReferenceFrame getFrame()
   {
      return frame;
   }

   public YoFramePoseUsingYawPitchRoll getOffset()
   {
      return frame.getOffset();
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + " - " + getName();
   }
}

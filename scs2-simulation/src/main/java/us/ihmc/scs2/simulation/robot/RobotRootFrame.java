package us.ihmc.scs2.simulation.robot;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;

public class RobotRootFrame extends FixedMovingReferenceFrame
{
   private final Robot robot;

   public RobotRootFrame(Robot robot)
   {
      super(robot.getName() + "RootFrame", robot.getInertialFrame(), new RigidBodyTransform());
      this.robot = robot;
   }

   public Robot getRobot()
   {
      return robot;
   }
}

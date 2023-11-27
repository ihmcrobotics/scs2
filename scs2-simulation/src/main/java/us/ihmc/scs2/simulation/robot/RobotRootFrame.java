package us.ihmc.scs2.simulation.robot;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;

public class RobotRootFrame extends FixedMovingReferenceFrame
{
   public RobotRootFrame(String robotName, ReferenceFrame inertialFrame)
   {
      super(robotName + "RootFrame", inertialFrame, new RigidBodyTransform());
   }
}

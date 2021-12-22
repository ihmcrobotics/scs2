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

public class SimWrenchSensor extends SimSensor
{
   private final YoFixedFrameWrench wrench;

   public SimWrenchSensor(WrenchSensorDefinition definition, SimJointBasics parentJoint)
   {
      this(definition.getName(), parentJoint, definition.getTransformToJoint());
   }

   public SimWrenchSensor(String name, SimJointBasics parentJoint, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, parentJoint, transformToParent);

      YoRegistry registry = parentJoint.getRegistry();
      wrench = new YoFixedFrameWrench(parentJoint.getSuccessor().getBodyFixedFrame(),
                                      new YoFrameVector3D(name + "Moment", getFrame(), registry),
                                      new YoFrameVector3D(name + "Force", getFrame(), registry));
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

      if (externalImpulse != null)
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
   }

   public YoFixedFrameWrench getWrench()
   {
      return wrench;
   }
}

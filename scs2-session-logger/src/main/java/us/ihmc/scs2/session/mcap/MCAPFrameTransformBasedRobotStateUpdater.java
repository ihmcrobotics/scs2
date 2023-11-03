package us.ihmc.scs2.session.mcap;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.scs2.simulation.robot.Robot;

public class MCAPFrameTransformBasedRobotStateUpdater
{

   private final MCAPFrameTransformManager frameTransformManager;

   private final List<Runnable> jointStateUpdaters = new ArrayList<>();

   public MCAPFrameTransformBasedRobotStateUpdater(Robot robot, MCAPFrameTransformManager frameTransformManager)
   {

      this.frameTransformManager = frameTransformManager;

      for (JointBasics joint : robot.getAllJoints())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;
            jointStateUpdaters.add(createOneDoFJointUpdater(oneDoFJoint));
         }
         else if (joint instanceof SixDoFJointBasics)
         {
            SixDoFJointBasics sixDoFJoint = (SixDoFJointBasics) joint;
            jointStateUpdaters.add(createSixDoFJointUpdater(sixDoFJoint));
         }
      }
   }

   private Runnable createOneDoFJointUpdater(OneDoFJointBasics oneDoFJoint)
   {
      RigidBodyTransform jointConfiguration = new RigidBodyTransform();
      RigidBodyTransformReadOnly beforeJointTransform = oneDoFJoint.getFrameBeforeJoint().getTransformToParent();
      MCAPFrameTransformManager.YoFoxGloveFrameTransform transform = frameTransformManager.getTransformFromSanitizedName(oneDoFJoint.getSuccessor().getName());
      return () ->
      {
         RigidBodyTransformReadOnly transformToParentJoint = transform.getTransformToParent();

         jointConfiguration.setAndInvert(beforeJointTransform);
         jointConfiguration.multiply(transformToParentJoint);
         oneDoFJoint.setJointConfiguration(jointConfiguration);
      };
   }

   private Runnable createSixDoFJointUpdater(SixDoFJointBasics sixDoFJoint)
   {
      MCAPFrameTransformManager.YoFoxGloveFrameTransform transform = frameTransformManager.getTransformFromSanitizedName(sixDoFJoint.getSuccessor().getName());
      return () ->
      {
         RigidBodyTransformReadOnly transformToParentJoint = transform.getTransformToParent();
         sixDoFJoint.setJointConfiguration(transformToParentJoint);
      };
   }

   public void updateRobotState()
   {
      for (Runnable jointStateUpdater : jointStateUpdaters)
      {
         jointStateUpdater.run();
      }
   }
}

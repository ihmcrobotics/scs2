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
   private final List<Runnable> jointStateUpdaters = new ArrayList<>();

   public MCAPFrameTransformBasedRobotStateUpdater(Robot robot, MCAPFrameTransformManager frameTransformManager)
   {
      for (JointBasics joint : robot.getAllJoints())
      {
         MCAPFrameTransformManager.YoFoxGloveFrameTransform transform = frameTransformManager.getTransformFromSanitizedName(joint.getSuccessor().getName());
         MCAPFrameTransformManager.YoFoxGloveFrameTransform parentJointTransform = frameTransformManager.getTransformFromSanitizedName(joint.getPredecessor()
                                                                                                                                            .getName());

         if (joint instanceof OneDoFJointBasics oneDoFJoint)
         {
            jointStateUpdaters.add(new OneDoFJointStateUpdater(oneDoFJoint, transform, parentJointTransform));
         }
         else if (joint instanceof SixDoFJointBasics sixDoFJoint)
         {
            jointStateUpdaters.add(new SixDoFJointStateUpdater(sixDoFJoint, transform));
         }
      }
   }

   public void updateRobotState()
   {
      for (Runnable jointStateUpdater : jointStateUpdaters)
      {
         jointStateUpdater.run();
      }
   }

   public static class SixDoFJointStateUpdater implements Runnable
   {
      private final SixDoFJointBasics joint;
      private final MCAPFrameTransformManager.YoFoxGloveFrameTransform transform;

      public SixDoFJointStateUpdater(SixDoFJointBasics joint, MCAPFrameTransformManager.YoFoxGloveFrameTransform transform)
      {
         this.joint = joint;
         this.transform = transform;
      }

      @Override
      public void run()
      {
         RigidBodyTransformReadOnly transformToParentJoint = transform.getTransformToParent();
         joint.setJointConfiguration(transformToParentJoint);
      }
   }

   public static class OneDoFJointStateUpdater implements Runnable
   {
      private final OneDoFJointBasics joint;
      private final MCAPFrameTransformManager.YoFoxGloveFrameTransform transform;
      private final MCAPFrameTransformManager.YoFoxGloveFrameTransform parentJointTransform;
      private final RigidBodyTransform jointConfiguration = new RigidBodyTransform();

      public OneDoFJointStateUpdater(OneDoFJointBasics joint,
                                     MCAPFrameTransformManager.YoFoxGloveFrameTransform transform,
                                     MCAPFrameTransformManager.YoFoxGloveFrameTransform parentJointTransform)
      {
         this.joint = joint;
         this.transform = transform;
         this.parentJointTransform = parentJointTransform;
      }

      @Override
      public void run()
      {
         RigidBodyTransformReadOnly beforeJointTransform = joint.getFrameBeforeJoint().getTransformToParent();
         RigidBodyTransformReadOnly transformParentJointToRoot = parentJointTransform.getTransformToRoot();
         RigidBodyTransformReadOnly transformToRoot = transform.getTransformToRoot();

         jointConfiguration.setIdentity();
         jointConfiguration.setAndInvert(beforeJointTransform);
         jointConfiguration.multiplyInvertOther(transformParentJointToRoot);
         jointConfiguration.multiply(transformToRoot);
         joint.setJointConfiguration(jointConfiguration);
      }
   }
}

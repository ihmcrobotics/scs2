package us.ihmc.scs2.session.mcap;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.scs2.session.mcap.MCAPFrameTransformManager.YoFoxGloveFrameTransform;
import us.ihmc.scs2.simulation.robot.Robot;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is used to update the robot state based on the frame transforms.
 */
public class MCAPFrameTransformBasedRobotStateUpdater implements RobotStateUpdater
{
   private final List<Runnable> jointStateUpdaters = new ArrayList<>();

   public MCAPFrameTransformBasedRobotStateUpdater(Robot robot, MCAPFrameTransformManager frameTransformManager)
   {
      for (JointBasics joint : robot.getAllJoints())
      {
         String successorName = joint.getSuccessor().getName();
         String predecessorName = joint.getPredecessor().getName();
         YoFoxGloveFrameTransform transform = frameTransformManager.getTransformFromSanitizedName(successorName);
         YoFoxGloveFrameTransform parentJointTransform = frameTransformManager.getTransformFromSanitizedName(predecessorName);

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

   @Override
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

package us.ihmc.scs2.session.mcap;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointReadOnly;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

public class MCAPMujocoBasedRobotStateUpdater implements RobotStateUpdater
{
   private static final String MUJOCO_ROOT_JOINT = "root_joint";

   public static boolean isRobotMujocoStateMessage(Robot robot, YoMCAPMessage message)
   {
      if (!message.getSchema().getName().contains("mujoco"))
         return false;

      SimFloatingJointReadOnly rootJoint = robot.getFloatingRootJoint();

      if (rootJoint != null)
      {
         if (rootJoint instanceof SixDoFJointReadOnly)
         {
            if (findSixDoFJointConfigurationVariable(message, MUJOCO_ROOT_JOINT) == null)
               return false;
         }
         else
         {
            throw new UnsupportedOperationException("Cannot handle root joint type: " + rootJoint.getClass().getSimpleName());
         }
      }

      for (JointReadOnly joint : robot.getAllJoints())
      {
         if (rootJoint == joint)
            continue;

         if (joint instanceof OneDoFJointReadOnly oneDoFJoint)
         {
            if (findOneDoFJointConfigurationVariable(message, oneDoFJoint) == null)
               return false;
         }
         else if (joint instanceof SixDoFJointReadOnly sixDoFJoint)
         {
            if (findSixDoFJointConfigurationVariable(message, sixDoFJoint) == null)
               return false;
         }
      }
      return true;
   }

   private final List<Runnable> jointStateUpdaters = new ArrayList<>();

   public MCAPMujocoBasedRobotStateUpdater(Robot robot, YoMCAPMessage message)
   {
      SimFloatingJointBasics rootJoint = robot.getFloatingRootJoint();

      if (rootJoint != null)
      {
         if (rootJoint instanceof SixDoFJointBasics sixDoFJoint)
         {
            YoPose3D pose = findSixDoFJointConfigurationVariable(message, MUJOCO_ROOT_JOINT);
            YoFrameVectorPair velocity = findSixDoFJointVelocityVariable(message,
                                                                         MUJOCO_ROOT_JOINT,
                                                                         sixDoFJoint.getFrameBeforeJoint(),
                                                                         sixDoFJoint.getFrameAfterJoint());
            YoFrameVectorPair acceleration = findSixDoFJointAccelerationVariable(message,
                                                                                 MUJOCO_ROOT_JOINT,
                                                                                 sixDoFJoint.getFrameBeforeJoint(),
                                                                                 sixDoFJoint.getFrameAfterJoint());

            if (pose != null)
               jointStateUpdaters.add(new SixDoFJointStateUpdater(sixDoFJoint, pose, velocity, acceleration));
         }
         else
         {
            throw new UnsupportedOperationException("Cannot handle root joint type: " + rootJoint.getClass().getSimpleName());
         }
      }

      for (JointReadOnly joint : robot.getAllJoints())
      {
         if (joint == rootJoint)
            continue;

         if (joint instanceof OneDoFJointBasics oneDoFJoint)
         {
            YoDouble q = findOneDoFJointConfigurationVariable(message, oneDoFJoint);
            YoDouble qd = findOneDoFJointVelocityVariable(message, oneDoFJoint);
            YoDouble qdd = findOneDoFJointAccelerationVariable(message, oneDoFJoint);
            YoDouble tau = findOneDoFJointTorqueVariable(message, oneDoFJoint);

            if (q != null)
               jointStateUpdaters.add(new OneDoFJointStateUpdater(oneDoFJoint, q, qd, qdd, tau));
         }
         else if (joint instanceof SixDoFJointBasics sixDoFJoint)
         {
            YoPose3D pose = findSixDoFJointConfigurationVariable(message, sixDoFJoint);
            YoFrameVectorPair velocity = findSixDoFJointVelocityVariable(message, sixDoFJoint);
            YoFrameVectorPair acceleration = findSixDoFJointAccelerationVariable(message, sixDoFJoint);

            if (pose != null)
               jointStateUpdaters.add(new SixDoFJointStateUpdater(sixDoFJoint, pose, velocity, acceleration));
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

   private static class SixDoFJointStateUpdater implements Runnable
   {
      private final SixDoFJointBasics joint;
      private final YoPose3D pose;
      private final YoFrameVectorPair velocity;
      private final YoFrameVectorPair acceleration;

      public SixDoFJointStateUpdater(SixDoFJointBasics joint, YoPose3D pose, YoFrameVectorPair velocity, YoFrameVectorPair acceleration)
      {
         this.joint = joint;
         this.pose = pose;
         this.velocity = velocity;
         this.acceleration = acceleration;
      }

      private final Vector3D tempAcceleration = new Vector3D();

      @Override
      public void run()
      {
         joint.getJointPose().set(pose);

         Orientation3DReadOnly orientation = pose.getOrientation();

         if (velocity != null)
         {
            if (velocity.angularPart.getReferenceFrame() == joint.getFrameAfterJoint())
               joint.getJointTwist().getAngularPart().set(velocity.angularPart);
            else if (velocity.angularPart.getReferenceFrame() == joint.getFrameBeforeJoint())
               orientation.inverseTransform(velocity.angularPart, joint.getJointTwist().getAngularPart());
            else
               throw new UnsupportedOperationException("Cannot handle angular velocity expressed in frame: " + velocity.angularPart.getReferenceFrame());

            if (velocity.linearPart.getReferenceFrame() == joint.getFrameAfterJoint())
               joint.getJointTwist().getLinearPart().set(velocity.linearPart);
            else if (velocity.linearPart.getReferenceFrame() == joint.getFrameBeforeJoint())
               orientation.inverseTransform(velocity.linearPart, joint.getJointTwist().getLinearPart());
            else
               throw new UnsupportedOperationException("Cannot handle linear velocity expressed in frame: " + velocity.linearPart.getReferenceFrame());
         }

         if (acceleration != null)
         {
            if (acceleration.angularPart.getReferenceFrame() == joint.getFrameAfterJoint())
               joint.getJointAcceleration().getAngularPart().set(acceleration.angularPart);
            else if (acceleration.angularPart.getReferenceFrame() == joint.getFrameBeforeJoint())
               orientation.inverseTransform(acceleration.angularPart, joint.getJointAcceleration().getAngularPart());
            else
               throw new UnsupportedOperationException(
                     "Cannot handle angular acceleration expressed in frame: " + acceleration.angularPart.getReferenceFrame());

            if (acceleration.linearPart.getReferenceFrame() == joint.getFrameAfterJoint())
            {
               joint.getJointAcceleration().getLinearPart().set(acceleration.linearPart);
            }
            else if (acceleration.linearPart.getReferenceFrame() == joint.getFrameBeforeJoint())
            {
               orientation.inverseTransform(acceleration.linearPart, tempAcceleration);
               MecanoTools.addCrossToVector(joint.getJointTwist().getLinearPart(), joint.getJointTwist().getAngularPart(), tempAcceleration);
               joint.getJointAcceleration().getLinearPart().set(tempAcceleration);
            }
            else
            {
               throw new UnsupportedOperationException("Cannot handle linear acceleration expressed in frame: " + acceleration.linearPart.getReferenceFrame());
            }
         }
      }
   }

   public static class OneDoFJointStateUpdater implements Runnable
   {
      private final OneDoFJointBasics joint;
      private final YoDouble q;
      private final YoDouble qd;
      private final YoDouble qdd;
      private final YoDouble tau;

      public OneDoFJointStateUpdater(OneDoFJointBasics joint, YoDouble q, YoDouble qd, YoDouble qdd, YoDouble tau)
      {
         this.joint = joint;
         this.q = q;
         this.qd = qd;
         this.qdd = qdd;
         this.tau = tau;
      }

      @Override
      public void run()
      {
         joint.setQ(q.getValue());
         if (qd != null)
            joint.setQd(qd.getValue());
         if (qdd != null)
            joint.setQdd(qdd.getValue());
         if (tau != null)
            joint.setTau(tau.getValue());
      }
   }

   private static YoDouble findOneDoFJointConfigurationVariable(YoMCAPMessage message, OneDoFJointReadOnly joint)
   {
      return findOneDoFJointConfigurationVariable(message, joint.getName());
   }

   private static YoDouble findOneDoFJointConfigurationVariable(YoMCAPMessage message, String jointName)
   {
      return (YoDouble) YoSearchTools.findVariable(variable ->
                                                   {
                                                      if (!(variable instanceof YoDouble))
                                                         return false;
                                                      String varName = variable.getName();
                                                      if (!varName.contains(jointName))
                                                         return false;
                                                      varName = varName.replace(jointName, "").replace("_", "");
                                                      return varName.equals("q");
                                                   }, message.getRegistry());
   }

   private static YoDouble findOneDoFJointVelocityVariable(YoMCAPMessage message, OneDoFJointReadOnly joint)
   {
      return findOneDoFJointVelocityVariable(message, joint.getName());
   }

   private static YoDouble findOneDoFJointVelocityVariable(YoMCAPMessage message, String jointName)
   {
      return (YoDouble) YoSearchTools.findVariable(variable ->
                                                   {
                                                      if (!(variable instanceof YoDouble))
                                                         return false;
                                                      String varName = variable.getName();
                                                      if (!varName.contains(jointName))
                                                         return false;
                                                      varName = varName.replace(jointName, "").replace("_", "");
                                                      return varName.equals("qd");
                                                   }, message.getRegistry());
   }

   private static YoDouble findOneDoFJointAccelerationVariable(YoMCAPMessage message, OneDoFJointReadOnly joint)
   {
      return findOneDoFJointAccelerationVariable(message, joint.getName());
   }

   private static YoDouble findOneDoFJointAccelerationVariable(YoMCAPMessage message, String jointName)
   {
      return (YoDouble) YoSearchTools.findVariable(variable ->
                                                   {
                                                      if (!(variable instanceof YoDouble))
                                                         return false;
                                                      String varName = variable.getName();
                                                      if (!varName.contains(jointName))
                                                         return false;
                                                      varName = varName.replace(jointName, "").replace("_", "");
                                                      return varName.equals("qdd");
                                                   }, message.getRegistry());
   }

   private static YoDouble findOneDoFJointTorqueVariable(YoMCAPMessage message, OneDoFJointReadOnly joint)
   {
      return findOneDoFJointTorqueVariable(message, joint.getName());
   }

   private static YoDouble findOneDoFJointTorqueVariable(YoMCAPMessage message, String jointName)
   {
      return (YoDouble) YoSearchTools.findVariable(variable ->
                                                   {
                                                      if (!(variable instanceof YoDouble))
                                                         return false;
                                                      String varName = variable.getName();
                                                      if (!varName.contains(jointName))
                                                         return false;
                                                      varName = varName.replace(jointName, "").replace("_", "");
                                                      return varName.equals("tau");
                                                   }, message.getRegistry());
   }

   private static YoPose3D findSixDoFJointConfigurationVariable(YoMCAPMessage message, SixDoFJointReadOnly joint)
   {
      return findSixDoFJointConfigurationVariable(message, joint.getName());
   }

   private static YoPose3D findSixDoFJointConfigurationVariable(YoMCAPMessage message, String jointName)
   {
      YoPoint3D position = findPoint3D(vector ->
                                       {
                                          String varName = vector.getNamePrefix() + vector.getNameSuffix();
                                          if (!varName.contains(jointName))
                                             return false;
                                          varName = varName.replace(jointName, "").replace("_", "");
                                          return varName.isBlank();
                                       }, message.getRegistry());
      YoQuaternion orientation = findQuaternion(quaternion ->
                                                {
                                                   String varName = quaternion.getNamePrefix() + quaternion.getNameSuffix();
                                                   if (!varName.contains(jointName))
                                                      return false;
                                                   varName = varName.replace(jointName, "").replace("_", "");
                                                   return varName.isBlank() || varName.equals("q");
                                                }, message.getRegistry());

      if (position == null || orientation == null)
         return null;
      else
         return new YoPose3D(position, orientation);
   }

   private static YoFrameVectorPair findSixDoFJointVelocityVariable(YoMCAPMessage message, SixDoFJointReadOnly joint)
   {
      return findSixDoFJointVelocityVariable(message, joint.getName(), joint.getFrameBeforeJoint(), joint.getFrameAfterJoint());
   }

   private static YoFrameVectorPair findSixDoFJointVelocityVariable(YoMCAPMessage message,
                                                                    String jointName,
                                                                    ReferenceFrame frameBeforeJoint,
                                                                    ReferenceFrame frameAfterJoint)
   {
      YoVector3D linearVelocity = findVector3D(vector ->
                                               {
                                                  String varName = vector.getNamePrefix() + vector.getNameSuffix();
                                                  if (!varName.contains(jointName))
                                                     return false;
                                                  varName = varName.replace(jointName, "").replace("_", "");
                                                  return varName.equals("d");
                                               }, message.getRegistry());
      YoVector3D angularVelocity = findVector3D(vector ->
                                                {
                                                   String varName = vector.getNamePrefix() + vector.getNameSuffix();
                                                   if (!varName.contains(jointName))
                                                      return false;
                                                   varName = varName.replace(jointName, "").replace("_", "");
                                                   return varName.equals("w");
                                                }, message.getRegistry());
      if (linearVelocity == null || angularVelocity == null)
         return null;
      else
         return new YoFrameVectorPair(new YoFrameVector3D(angularVelocity, frameAfterJoint), new YoFrameVector3D(linearVelocity, frameBeforeJoint));
   }

   private static YoFrameVectorPair findSixDoFJointAccelerationVariable(YoMCAPMessage message, SixDoFJointReadOnly joint)
   {
      return findSixDoFJointAccelerationVariable(message, joint.getName(), joint.getFrameBeforeJoint(), joint.getFrameAfterJoint());
   }

   private static YoFrameVectorPair findSixDoFJointAccelerationVariable(YoMCAPMessage message,
                                                                        String jointName,
                                                                        ReferenceFrame frameBeforeJoint,
                                                                        ReferenceFrame frameAfterJoint)
   {
      YoVector3D linearAcceleration = findVector3D(vector ->
                                                   {
                                                      String varName = vector.getNamePrefix() + vector.getNameSuffix();
                                                      if (!varName.contains(jointName))
                                                         return false;
                                                      varName = varName.replace(jointName, "").replace("_", "");
                                                      return varName.equals("dd");
                                                   }, message.getRegistry());
      YoVector3D angularAcceleration = findVector3D(vector ->
                                                    {
                                                       String varName = vector.getNamePrefix() + vector.getNameSuffix();
                                                       if (!varName.contains(jointName))
                                                          return false;
                                                       varName = varName.replace(jointName, "").replace("_", "");
                                                       return varName.equals("wd");
                                                    }, message.getRegistry());
      if (linearAcceleration == null || angularAcceleration == null)
         return null;
      else
         return new YoFrameVectorPair(new YoFrameVector3D(angularAcceleration, frameAfterJoint), new YoFrameVector3D(linearAcceleration, frameBeforeJoint));
   }

   // TODO Move to YoSearchTools
   public static YoPoint3D findPoint3D(Predicate<YoPoint3D> filter, YoVariableHolder yoVariableHolder)
   {
      List<YoPoint3D> yoPoint3Ds = filterPoint3Ds(filter, yoVariableHolder);

      if (yoPoint3Ds.isEmpty())
         return null;
      else
         return yoPoint3Ds.get(0);
   }

   public static YoVector3D findVector3D(Predicate<YoVector3D> filter, YoVariableHolder yoVariableHolder)
   {
      List<YoVector3D> yoVector3Ds = filterVector3Ds(filter, yoVariableHolder);

      if (yoVector3Ds.isEmpty())
         return null;
      else
         return yoVector3Ds.get(0);
   }

   public static List<YoPoint3D> filterPoint3Ds(Predicate<YoPoint3D> filter, YoVariableHolder yoVariableHolder)
   {
      return filterTuple3Ds(filter, yoVariableHolder, YoPoint3D::new);
   }

   public static List<YoVector3D> filterVector3Ds(Predicate<YoVector3D> filter, YoVariableHolder yoVariableHolder)
   {
      return filterTuple3Ds(filter, yoVariableHolder, YoVector3D::new);
   }

   public static YoQuaternion findQuaternion(Predicate<YoQuaternion> filter, YoVariableHolder yoVariableHolder)
   {
      List<YoQuaternion> yoQuaternions = filterQuaternions(filter, yoVariableHolder);

      if (yoQuaternions.isEmpty())
         return null;
      else
         return yoQuaternions.get(0);
   }

   private static <T extends YoTuple3D> List<T> filterTuple3Ds(Predicate<T> filter, YoVariableHolder yoVariableHolder, YoTuple3DBuilder<T> builder)
   {
      List<T> yoTuple3Ds = new ArrayList<>();

      List<YoVariable> xComponents = YoSearchTools.filterVariables(variable -> variable instanceof YoDouble && variable.getName().toLowerCase().contains("x"),
                                                                   yoVariableHolder);

      for (YoVariable xComponent : xComponents)
      {
         String xComponentName = xComponent.getName();
         String xComponentNameLC = xComponentName.toLowerCase();

         int xIndex = -1;
         while ((xIndex = xComponentNameLC.indexOf('x', xIndex + 1)) > -1)
         {
            boolean isIdentifierUpperCase = Character.isUpperCase(xComponentName.charAt(xIndex));

            String prefix = xIndex == 0 ? "" : xComponentName.substring(0, xIndex);
            String suffix = xIndex == xComponentName.length() - 1 ? "" : xComponentName.substring(xIndex + 1);

            String yComponentName = prefix + (isIdentifierUpperCase ? "Y" : "y") + suffix;
            YoVariable yComponent = yoVariableHolder.findVariable(yComponentName);
            if (!(yComponent instanceof YoDouble))
               continue;

            String zComponentName = prefix + (isIdentifierUpperCase ? "Z" : "z") + suffix;
            YoVariable zComponent = yoVariableHolder.findVariable(zComponentName);
            if (!(zComponent instanceof YoDouble))
               continue;

            T yoTuple3D = builder.build((YoDouble) xComponent, (YoDouble) yComponent, (YoDouble) zComponent);
            if (filter.test(yoTuple3D))
               yoTuple3Ds.add(yoTuple3D);
         }
      }

      return yoTuple3Ds;
   }

   public static List<YoQuaternion> filterQuaternions(Predicate<YoQuaternion> filter, YoVariableHolder yoVariableHolder)
   {
      List<YoQuaternion> yoQuaternions = new ArrayList<>();

      List<YoVariable> sComponents = YoSearchTools.filterVariables(variable -> variable instanceof YoDouble && variable.getName().toLowerCase().contains("x"),
                                                                   yoVariableHolder);

      for (YoVariable xComponent : sComponents)
      {
         String xComponentName = xComponent.getName();
         String xComponentNameLC = xComponentName.toLowerCase();

         int wIndex = -1;
         while ((wIndex = xComponentNameLC.indexOf('x', wIndex + 1)) > -1)
         {
            boolean isIdentifierUpperCase = Character.isUpperCase(xComponentName.charAt(wIndex));

            String prefix = wIndex == 0 ? "" : xComponentName.substring(0, wIndex);
            String suffix = wIndex == xComponentName.length() - 1 ? "" : xComponentName.substring(wIndex + 1);

            String yComponentName = prefix + (isIdentifierUpperCase ? "Y" : "y") + suffix;
            YoVariable yComponent = yoVariableHolder.findVariable(yComponentName);
            if (!(yComponent instanceof YoDouble))
               continue;

            String zComponentName = prefix + (isIdentifierUpperCase ? "Z" : "z") + suffix;
            YoVariable zComponent = yoVariableHolder.findVariable(zComponentName);
            if (!(zComponent instanceof YoDouble))
               continue;

            String sComponentName = prefix + (isIdentifierUpperCase ? "S" : "s") + suffix;
            YoVariable sComponent = yoVariableHolder.findVariable(sComponentName);
            if (!(sComponent instanceof YoDouble))
            {
               sComponentName = prefix + (isIdentifierUpperCase ? "W" : "w") + suffix;
               sComponent = yoVariableHolder.findVariable(sComponentName);
               if (!(sComponent instanceof YoDouble))
                  continue;
            }

            YoQuaternion yoQuaternion = new YoQuaternion((YoDouble) xComponent, (YoDouble) yComponent, (YoDouble) zComponent, (YoDouble) sComponent);
            if (filter.test(yoQuaternion))
               yoQuaternions.add(yoQuaternion);
         }
      }

      return yoQuaternions;
   }

   private interface YoTuple3DBuilder<T extends YoTuple3D>
   {
      T build(YoDouble x, YoDouble y, YoDouble z);
   }

   private record YoFrameVectorPair(YoFrameVector3D angularPart, YoFrameVector3D linearPart)
   {
   }
}

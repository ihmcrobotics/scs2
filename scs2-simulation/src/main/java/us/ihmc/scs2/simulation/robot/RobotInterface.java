package us.ihmc.scs2.simulation.robot;

import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.robot.controller.RobotControllerManager;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimMultiBodySystemBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;

public interface RobotInterface extends SimMultiBodySystemBasics
{
   String getName();

   RobotDefinition getRobotDefinition();

   RobotControllerManager getControllerManager();

   void initializeState();

   default void updateFrames()
   {
      getRootBody().updateFramesRecursively();
   }

   SimRigidBodyBasics getRigidBody(String name);

   SimJointBasics getJoint(String name);

   default SimOneDoFJointBasics getOneDoFJoint(String name)
   {
      SimJointBasics joint = getJoint(name);
      if (joint instanceof SimOneDoFJointBasics)
         return (SimOneDoFJointBasics) joint;
      else
         return null;
   }
}

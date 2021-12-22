package us.ihmc.scs2.simulation.robot;

import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.robot.controller.RobotControllerManager;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class RobotExtension implements RobotInterface
{
   private final Robot robot;

   public RobotExtension(Robot robot)
   {
      this.robot = robot;
   }

   public RobotExtension(RobotDefinition robotDefinition, ReferenceFrame inertialFrame)
   {
      this(new Robot(robotDefinition, inertialFrame));
   }

   public Robot getRobot()
   {
      return robot;
   }

   @Override
   public String getName()
   {
      return robot.getName();
   }

   @Override
   public RobotDefinition getRobotDefinition()
   {
      return robot.getRobotDefinition();
   }

   @Override
   public RobotControllerManager getControllerManager()
   {
      return robot.getControllerManager();
   }

   @Override
   public void initializeState()
   {
      robot.initializeState();
   }

   @Override
   public SimRigidBodyBasics getRootBody()
   {
      return robot.getRootBody();
   }

   @Override
   public SimRigidBodyBasics getRigidBody(String name)
   {
      return robot.getRigidBody(name);
   }

   @Override
   public SimJointBasics getJoint(String name)
   {
      return robot.getJoint(name);
   }

   @Override
   public List<? extends SimJointBasics> getAllJoints()
   {
      return robot.getAllJoints();
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return robot.getInertialFrame();
   }

   @Override
   public JointMatrixIndexProvider getJointMatrixIndexProvider()
   {
      return robot.getJointMatrixIndexProvider();
   }

   @Override
   public List<? extends SimJointBasics> getJointsToConsider()
   {
      return robot.getJointsToConsider();
   }

   @Override
   public List<? extends SimJointBasics> getJointsToIgnore()
   {
      return robot.getJointsToIgnore();
   }

   @Override
   public YoRegistry getRegistry()
   {
      return robot.getRegistry();
   }
}

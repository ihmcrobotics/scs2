package us.ihmc.scs2.simulation.robot;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition.JointStateEntry;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.simulation.robot.controller.RobotControllerManager;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.robot.state.YoJointState;
import us.ihmc.scs2.simulation.robot.state.YoOneDoFJointState;
import us.ihmc.scs2.simulation.robot.state.YoSixDoFJointState;
import us.ihmc.scs2.simulation.robot.state.YoSphericalJointState;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class RobotExtension implements RobotInterface
{
   private final Robot robot;
   private final YoRegistry robotPhysicsRegistry;
   private final RobotState robotBeforePhysicsState;

   public RobotExtension(Robot robot, YoRegistry physicsRegistry)
   {
      this.robot = robot;
      robotPhysicsRegistry = new YoRegistry(robot.getName() + "Physics");
      physicsRegistry.addChild(robotPhysicsRegistry);
      robotBeforePhysicsState = new RobotState("beforePhysics", getRootBody(), robotPhysicsRegistry);
   }

   public RobotExtension(RobotDefinition robotDefinition, ReferenceFrame inertialFrame, YoRegistry physicsRegistry)
   {
      this(new Robot(robotDefinition, inertialFrame), physicsRegistry);
   }

   public Robot getRobot()
   {
      return robot;
   }

   public YoRegistry getRobotPhysicsRegistry()
   {
      return robotPhysicsRegistry;
   }

   public void saveRobotBeforePhysicsState()
   {
      robotBeforePhysicsState.readState();
   }

   public RobotStateDefinition getRobotBeforePhysicsStateDefinition()
   {
      return robotBeforePhysicsState.toRobotStateDefinition(robot.getName());
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
   public void resetState()
   {
      robot.resetState();
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

   public YoRegistry getSecondaryRegistry()
   {
      return robot.getSecondaryRegistry();
   }

   protected static class RobotState
   {
      private final List<JointReadOnly> joints = new ArrayList<>();
      private final List<JointStateBasics> jointStates = new ArrayList<>();

      public RobotState(String namePrefix, RigidBodyReadOnly rootBody, YoRegistry registry)
      {
         for (JointReadOnly joint : rootBody.childrenSubtreeIterable())
         {
            joints.add(joint);

            if (joint instanceof OneDoFJointReadOnly)
               jointStates.add(new YoOneDoFJointState(namePrefix, joint.getName(), registry));
            else if (joint instanceof SixDoFJointReadOnly)
               jointStates.add(new YoSixDoFJointState(namePrefix, joint.getName(), registry));
            else if (joint instanceof SphericalJointReadOnly)
               jointStates.add(new YoSphericalJointState(namePrefix, joint.getName(), registry));
            else
               jointStates.add(new YoJointState(namePrefix, joint.getName(), joint.getConfigurationMatrixSize(), joint.getDegreesOfFreedom(), registry));
         }
      }

      public void readState()
      {
         for (int i = 0; i < joints.size(); i++)
         {
            jointStates.get(i).set(joints.get(i));
         }
      }

      public RobotStateDefinition toRobotStateDefinition(String robotName)
      {
         RobotStateDefinition definition = new RobotStateDefinition();
         definition.setRobotName(robotName);
         List<JointStateEntry> entries = new ArrayList<>();

         for (int i = 0; i < joints.size(); i++)
         {
            entries.add(new JointStateEntry(joints.get(i).getName(), jointStates.get(i).copy()));
         }

         definition.setJointStateEntries(entries);
         return definition;
      }
   }
}

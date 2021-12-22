package us.ihmc.scs2.simulation.robot.controller;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimMultiBodySystemBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RobotControllerManager
{
   private final YoRegistry registry;
   private final SimControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final List<Controller> controllers = new ArrayList<>();
   private final SimMultiBodySystemBasics input;

   public RobotControllerManager(SimMultiBodySystemBasics input, YoRegistry registry)
   {
      this.input = input;
      this.registry = registry;
      controllerInput = new SimControllerInput(input);
      controllerOutput = new ControllerOutput(input);
   }

   public SimControllerInput getControllerInput()
   {
      return controllerInput;
   }

   public ControllerOutput getControllerOutput()
   {
      return controllerOutput;
   }

   public void addController(Controller controller)
   {
      if (controllers.add(controller))
      {
         registry.addChild(controller.getYoRegistry());
      }
   }

   public void addController(ControllerDefinition controllerDefinition)
   {
      addController(controllerDefinition.newController(controllerInput, controllerOutput));
   }

   public void initializeControllers()
   {
      controllerInput.setTime(0.0);

      for (Controller controller : controllers)
      {
         controller.initialize();
      }
   }

   public void updateControllers(double time)
   {
      controllerInput.setTime(time);

      for (Controller controller : controllers)
      {
         controller.doControl();
      }
   }

   public void pauseControllers()
   {
      for (Controller controller : controllers)
      {
         controller.pause();
      }
   }

   public void writeControllerOutput(JointStateType... statesToWrite)
   {
      for (JointStateType stateToWrite : statesToWrite)
      {
         writeControllerOutput(stateToWrite);
      }
   }

   public void writeControllerOutput(JointStateType stateToWrite)
   {
      for (JointBasics joint : input.getJointsToConsider())
      {
         JointStateBasics jointOutput = controllerOutput.getJointOutput(joint);
         if (jointOutput.hasOutputFor(stateToWrite))
         {
            if (stateToWrite == JointStateType.CONFIGURATION)
               jointOutput.getConfiguration(joint);
            else if (stateToWrite == JointStateType.VELOCITY)
               jointOutput.getVelocity(joint);
            else if (stateToWrite == JointStateType.ACCELERATION)
               jointOutput.getAcceleration(joint);
            else if (stateToWrite == JointStateType.EFFORT)
               jointOutput.getEffort(joint);
         }
      }
   }

   public void writeControllerOutputForJointsToIgnore(JointStateType... statesToWrite)
   {
      for (JointBasics joint : input.getJointsToIgnore())
      {
         JointStateBasics jointOutput = controllerOutput.getJointOutput(joint);

         for (JointStateType stateToWrite : statesToWrite)
         {
            if (!jointOutput.hasOutputFor(stateToWrite))
               continue;
            if (stateToWrite == JointStateType.CONFIGURATION)
               jointOutput.getConfiguration(joint);
            else if (stateToWrite == JointStateType.VELOCITY)
               jointOutput.getVelocity(joint);
            else if (stateToWrite == JointStateType.ACCELERATION)
               jointOutput.getAcceleration(joint);
            else if (stateToWrite == JointStateType.EFFORT)
               jointOutput.getEffort(joint);
         }
      }
   }
}

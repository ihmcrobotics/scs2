package us.ihmc.scs2.simulation.robot;

import java.util.concurrent.TimeUnit;

import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
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

   /**
    * Adds a controller to be run with the robot owning this manager.
    * <p>
    * The controller will be updated every session run tick, e.g. simulation tick.
    * </p>
    * 
    * @param controller the new controller.
    */
   default void addController(Controller controller)
   {
      getControllerManager().addController(controller);
   }

   /**
    * Adds a controller to be run with this robot and specifies the desired period at which it should
    * be updated.
    * 
    * @param controller      the new controller.
    * @param periodInSeconds the update period in seconds. Be mindful that this period should be a
    *                        multiple of the session period (e.g. simulation DT) or the update rate
    *                        will be inaccurate.
    */
   default void addThrottledController(Controller controller, double periodInSeconds)
   {
      getControllerManager().addThrottledController(controller, periodInSeconds);
   }

   /**
    * Adds a controller to be run with the robot owning this manager and specifies the desired period
    * at which it should be updated.
    * 
    * @param controller the new controller.
    * @param period     the update period. Be mindful that this period should be a multiple of the
    *                   session period (e.g. simulation DT) or the update rate will be inaccurate.
    * @param timeUnit   the unit of time for the period.
    */
   default void addThrottledController(Controller controller, long period, TimeUnit timeUnit)
   {
      getControllerManager().addThrottledController(controller, period, timeUnit);
   }

   /**
    * Adds a controller to be run with this robot.
    * <p>
    * The controller will be updated every session run tick, e.g. simulation tick.
    * </p>
    * 
    * @param controller the new controller.
    */
   default void addController(ControllerDefinition controllerDefinition)
   {
      getControllerManager().addController(controllerDefinition);
   }

   /**
    * Adds a controller to be run with this robot and specifies the desired period at which it should
    * be updated.
    * 
    * @param controller      the new controller.
    * @param periodInSeconds the update period in seconds. Be mindful that this period should be a
    *                        multiple of the session period (e.g. simulation DT) or the update rate
    *                        will be inaccurate.
    */
   default void addThrottledController(ControllerDefinition controllerDefinition, double periodInSeconds)
   {
      getControllerManager().addThrottledController(controllerDefinition, periodInSeconds);
   }

   /**
    * Adds a controller to be run with this robot and specifies the desired period at which it should
    * be updated.
    * 
    * @param controller the new controller.
    * @param period     the update period. Be mindful that this period should be a multiple of the
    *                   session period (e.g. simulation DT) or the update rate will be inaccurate.
    * @param timeUnit   the unit of time for the period.
    */
   default void addThrottledController(ControllerDefinition controllerDefinition, long period, TimeUnit timeUnit)
   {
      getControllerManager().addThrottledController(controllerDefinition, period, timeUnit);
   }

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

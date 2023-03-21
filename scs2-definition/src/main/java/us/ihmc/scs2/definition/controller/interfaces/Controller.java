package us.ihmc.scs2.definition.controller.interfaces;

import us.ihmc.yoVariables.registry.YoRegistry;

public interface Controller
{
   /**
    * Called before calling {@link #doControl()} for the first time.
    */
   default void initialize()
   {
   }

   /**
    * Called at regular time interval.
    */
   void doControl();

   /**
    * Called when pausing the simulation and the controller needs to park some jobs.
    */
   default void pause()
   {
   }

   /**
    * The registry used to store all the {@code YoVariable}s for the controller.
    * <p>
    * In simulation, the registry is typically attached to the robot's registry when adding it
    * {@code this} to the robot controllers.
    * </p>
    * 
    * @return the controller's registry or {@code null} if the controller doesn't use
    *         {@code YoVariable}s
    */
   default YoRegistry getYoRegistry()
   {
      return null;
   }

   /**
    * The name of this controller.
    * <p>
    * The default implementation returns the simple name of the controller class.
    * </p>
    * 
    * @return the controller name.
    */
   default String getName()
   {
      return getClass().getSimpleName();
   }

   /**
    * Creates a controller that does nothing.
    * 
    * @return an empty controller.
    */
   public static Controller emptyController()
   {
      return () ->
      {
         /* Do nothing */
      };
   }
}

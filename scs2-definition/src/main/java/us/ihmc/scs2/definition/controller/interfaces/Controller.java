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

   default YoRegistry getYoRegistry()
   {
      return null;
   }

   default String getName()
   {
      return getClass().getSimpleName();
   }

   public static Controller emptyController()
   {
      return () ->
      {
         /* Do nothing */ };
   }
}

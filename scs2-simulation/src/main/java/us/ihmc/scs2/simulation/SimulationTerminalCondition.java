package us.ihmc.scs2.simulation;

import java.util.function.BooleanSupplier;

/**
 * This interface can be used to implement a custom terminal condition for stopping a simulation.
 * 
 * @see SimulationSessionControls
 */
public interface SimulationTerminalCondition
{
   /** Determines whether the simulation was successful or not. */
   public enum TerminalState
   {
      /** Indicates that the simulation was successful, good job robot! */
      SUCCESS,
      /** Indicates that the simulation was not successful, boooooo. */
      FAILURE;
   }

   /**
    * Called every simulation tick until the returned value is not {@code null}.
    * 
    * @return {@code null} is the simulation should keep going, {@link TerminalState#SUCCESS} to
    *         indicate that the simulation should end and that it was successful, or
    *         {@link TerminalState#FAILURE} to indicate that the simulation should end and that it was
    *         not successful.
    */
   TerminalState testCondition();

   /**
    * Convenience factory for creating terminal condition that can only terminate the simulation in
    * case of successful completion.
    * 
    * @param condition when returning {@code true}, the simulation will end and be considered a
    *                  success.
    * @return the condition to use with {@link SimulationSessionControls}
    */
   public static SimulationTerminalCondition newSuccessfulTerminalCondition(BooleanSupplier condition)
   {
      return () -> condition.getAsBoolean() ? TerminalState.SUCCESS : null;
   }

   /**
    * Convenience factory for creating terminal condition that can only terminate the simulation in
    * case of a failure.
    * 
    * @param condition when returning {@code true}, the simulation will end and be considered a
    *                  failure.
    * @return the condition to use with {@link SimulationSessionControls}
    */
   public static SimulationTerminalCondition newUnsuccessfulTerminalCondition(BooleanSupplier condition)
   {
      return () -> condition.getAsBoolean() ? TerminalState.FAILURE : null;
   }
}

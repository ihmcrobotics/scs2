package us.ihmc.scs2.simulation;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Interface that represents the Java API for controlling an instance of a
 * {@link SimulationSession}.
 */
public interface SimulationSessionControls
{
   /**
    * Requests to stops any ongoing simulation and playback. (asynchronous)
    * <p>
    * This is a non-blocking request, the session will handle the request as soon as possible.
    * </p>
    */
   void pause();

   /**
    * Requests to simulate indefinitely. (asynchronous)
    * <p>
    * This is a non-blocking request, the session will handle the request as soon as possible.
    * </p>
    */
   void simulate();

   /**
    * Requests to simulate for a given {@code duration} in seconds. (asynchronous)
    * <p>
    * This is a non-blocking request, the session will handle the request as soon as possible.
    * </p>
    * 
    * @param duration the simulation duration in seconds.
    */
   void simulate(double duration);

   /**
    * Requests to simulate for a given number of simulation ticks. (asynchronous)
    * <p>
    * This is a non-blocking request, the session will handle the request as soon as possible.
    * </p>
    * 
    * @param numberOfTicks the number of simulation ticks to perform.
    */
   void simulate(int numberOfTicks);

   /**
    * Requests to simulate for a given {@code duration} in seconds. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * 
    * @param duration the simulation duration in seconds.
    * @return whether the simulation was considered successful ({@code true}) or not ({@code false}).
    *         The simulation is successful if the desired {@code duration} has been reached or if any
    *         of the addition external terminal conditions was fulfilled without any interruption. An
    *         interruption can be for instance an exception thrown by the simulation or a controller.
    */
   boolean simulateNow(double duration);

   /**
    * Requests to simulate for a given number of simulation ticks. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * 
    * @param duration the simulation duration in seconds.
    * @return whether the simulation was considered successful ({@code true}) or not ({@code false}).
    *         The simulation is successful if the desired {@code numberOfTicks} were all executed or if
    *         any of the addition external terminal conditions was fulfilled without any interruption.
    *         An interruption can be for instance an exception thrown by the simulation or a
    *         controller.
    */
   boolean simulateNow(long numberOfTicks);

   /**
    * Requests to simulate for an infinite period of time. (synchronous)
    * <p>
    * WARNING: This will block the calling thread indefinitely. The caller needs to add an additional
    * terminal condition beforehand to eventually stop the simulation.
    * </p>
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * 
    * @return whether the simulation was considered successful ({@code true}) or not ({@code false}).
    *         The simulation is successful if the desired {@code numberOfTicks} were all executed or if
    *         any of the addition external terminal conditions was fulfilled without any interruption.
    *         An interruption can be for instance an exception thrown by the simulation or a
    *         controller.
    * @see #addExternalTerminalCondition(BooleanSupplier...)
    */
   boolean simulateNow();

   /**
    * Adds a listener to be notified of any exception thrown during a simulation tick.
    * 
    * @param listener the consumer to use as a exception listener.
    */
   void addSimulationThrowableListener(Consumer<Throwable> listener);

   // Buffer controls:
   /**
    * Requests setting the in-point of the buffer to the current index. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    */
   void setBufferInPointIndexToCurrent();

   /**
    * Requests setting the out-point of the buffer to the current index. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    */
   void setBufferOutPointIndexToCurrent();

   /**
    * Requests moving the current index to go to the in-point. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    */
   void setBufferCurrentIndexToInPoint();

   /**
    * Requests moving the current index to go to the out-point. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    */
   void setBufferCurrentIndexToOutPoint();

   /**
    * Requests stepping the current index backward by one simulation tick. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    */
   default void stepBufferIndexBackward()
   {
      stepBufferIndexBackward(1);
   }

   /**
    * Requests stepping the current index backward by {@code stepSize} simulation tick(s).
    * (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    */
   void stepBufferIndexBackward(int stepSize);

   /**
    * Requests stepping the current index forward by one simulation tick. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    */
   default void stepBufferIndexForward()
   {
      stepBufferIndexForward(1);
   }

   /**
    * Requests stepping the current index forward by {@code stepSize} simulation tick(s). (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    */
   void stepBufferIndexForward(int stepSize);

   /**
    * Adds custom terminal condition(s) that will be used to identify when to terminate the next call
    * to the next simulation. The simulation is considered successful when it terminates by triggering
    * of the terminal conditions.
    * 
    * @param externalTerminalConditions the custom conditions to be considered for the next
    *                                   simulations.
    */
   void addExternalTerminalCondition(BooleanSupplier... externalTerminalConditions);

   /**
    * Removes a custom condition previously registered.
    * 
    * @param externalTerminalCondition the condition to be removed.
    * @return {@code true} if the condition was successfully removed, {@code false} if it could not be
    *         found.
    */
   boolean removeExternalTerminalCondition(BooleanSupplier externalTerminalCondition);

   /**
    * Removes all custom conditions previously added.
    */
   void clearExternalTerminalConditions();
}

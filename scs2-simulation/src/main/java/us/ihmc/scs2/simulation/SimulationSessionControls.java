package us.ihmc.scs2.simulation;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferProcessor;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Interface that represents the Java API for controlling an instance of a
 * {@link SimulationSession}.
 */
public interface SimulationSessionControls
{
   // ------------------------------------------------------------------------------- //
   // ------------------------ Simulation Properties -------------------------------- //
   // ------------------------------------------------------------------------------- //

   /**
    * Gets the simulation step size in seconds.
    * 
    * @return the simulation step size in seconds.
    */
   double getDT();

   /**
    * Sets the simulation step size in seconds.
    * 
    * @param dt the new simulation step size in seconds.
    */
   void setDT(double dt);

   /**
    * Returns whether this simulation's thread has been started.
    * 
    * @return {@code true} if the simulation thread has been started.
    */
   boolean isSimulationThreadRunning();

   /**
    * Whether the simulation is capped to run no faster that real-time.
    * 
    * @return {@code true} if the simulation is capped to run no faster than real-time.
    */
   boolean isRealTimeRateSimulation();

   /**
    * Whether this session is currently simulating.
    * 
    * @return {@code true} if this session is simulating, {@code false} if it is paused or playing
    *         back.
    */
   boolean isSimulating();

   /**
    * Whether this session is currently playing back the data previously recorded.
    * 
    * @return {@code true} if this session is playing back, {@code false} if it is paused or
    *         simulating.
    */
   boolean isPlaying();

   /**
    * Whether this session is currently pause.
    * 
    * @return {@code true} if this session is paused, {@code false} if it is simulating or playing
    *         back.
    */
   boolean isPaused();

   /**
    * Returns whether this simulation session has been shutdown and is thus unusable.
    * 
    * @return {@code true} if this session has been shutdown.
    */
   boolean isSessionShutdown();

   // ------------------------------------------------------------------------------- //
   // ------------------------- Simulation Controls --------------------------------- //
   // ------------------------------------------------------------------------------- //

   /**
    * Starts the internal thread of this simulation running the current session mode.
    * 
    * @return {@code true} if the thread has started, {@code false} if it could not be started, e.g. it
    *         was already started or the simulation was shutdown.
    */
   boolean startSimulationThread();

   /**
    * Stops the internal thread of this simulation without notifying any of the listeners.
    * <p>
    * After calling this method, this simulation stops operating but it can resume from it by calling
    * {@link #startSimulationThread()}.
    * </p>
    * <p>
    * This is a blocking operation and will return only when done.
    * </p>
    * 
    * @return {@code true} if the thread has stopped, {@code false} if it could not be stopped, e.g. it
    *         was already stopped or the simulation was shutdown.
    */
   boolean stopSimulationThread();

   /**
    * Shuts down this simulation session permanently, it becomes unusable.
    * <p>
    * This method notifies the shutdown listeners and performs a memory cleanup.
    * </p>
    */
   void shutdownSession();

   /**
    * Adds a listener to be notified whenever this session is about to shutdown.
    * 
    * @param listener the listener to add.
    */
   void addSessionShutdownListener(Runnable listener);

   /**
    * Sets whether or not the simulation should be capped to be running no faster than real-time.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param runAtRealTimeRate {@code true} to cap the running mode at real-time rate, {@code false} to
    *                          let the running mode run as fast as possible. Default value
    *                          {@value #DEFAULT_RUN_AT_REALTIME_RATE}.
    */
   void setRealTimeRateSimulation(boolean enableRealTimeRate);

   /**
    * Requests to simulate indefinitely. (asynchronous)
    * <p>
    * This is a non-blocking request, the session will handle the request as soon as possible.
    * </p>
    */
   default void simulate()
   {
      simulate(Double.POSITIVE_INFINITY);
   }

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
    * <p>
    * In the case the user request to pause the simulation from the visualizer, the simulation will be
    * paused and the thread will be blocked until the user request to run the simulation from the
    * visualizer. When running the simulation is requested, the current buffer index is first moved to
    * the out-point where the {@code YoVariable}s are reloaded before resuming the simulation. This
    * allows to resume the simulation as if the user had never paused.
    * </p>
    * 
    * @param duration the simulation duration in seconds.
    * @return whether the simulation was considered successful ({@code true}) or not ({@code false}).
    *         The simulation is successful if the desired {@code duration} has been reached or if any
    *         of the addition external terminal conditions was fulfilled without any interruption. An
    *         interruption can be for instance an exception thrown by the simulation or a controller.
    */
   default boolean simulateNow(double duration)
   {
      return simulateNow((long) (duration / getDT()));
   }

   /**
    * Requests to simulate for a given number of simulation ticks. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * In the case the user request to pause the simulation from the visualizer, the simulation will be
    * paused and the thread will be blocked until the user request to run the simulation from the
    * visualizer. When running the simulation is requested, the current buffer index is first moved to
    * the out-point where the {@code YoVariable}s are reloaded before resuming the simulation. This
    * allows to resume the simulation as if the user had never paused.
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
    * <p>
    * In the case the user request to pause the simulation from the visualizer, the simulation will be
    * paused and the thread will be blocked until the user request to run the simulation from the
    * visualizer. When running the simulation is requested, the current buffer index is first moved to
    * the out-point where the {@code YoVariable}s are reloaded before resuming the simulation. This
    * allows to resume the simulation as if the user had never paused.
    * </p>
    * 
    * @return whether the simulation was considered successful ({@code true}) or not ({@code false}).
    *         The simulation is successful if the desired {@code numberOfTicks} were all executed or if
    *         any of the addition external terminal conditions was fulfilled without any interruption.
    *         An interruption can be for instance an exception thrown by the simulation or a
    *         controller.
    * @see #addExternalTerminalCondition(BooleanSupplier...)
    */
   default boolean simulateNow()
   {
      return simulateNow(-1L);
   }

   /**
    * Adds a listener to be notified of any exception thrown during a simulation tick.
    * 
    * @param listener the consumer to use as a exception listener.
    */
   void addSimulationThrowableListener(Consumer<Throwable> listener);

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

   /**
    * Requests to start playing back previously recorded data. (asynchronous)
    * <p>
    * This is a non-blocking request, the session will handle the request as soon as possible.
    * </p>
    */
   void play();

   /**
    * Requests to stops any ongoing simulation and playback. (asynchronous)
    * <p>
    * This is a non-blocking request, the session will handle the request as soon as possible.
    * </p>
    */
   void pause();

   // ------------------------------------------------------------------------------- //
   // -------------------------- Buffer Properties ---------------------------------- //
   // ------------------------------------------------------------------------------- //

   /**
    * Gets the current buffer properties.
    * 
    * @return the read-only properties of the buffer.
    */
   YoBufferPropertiesReadOnly getBufferProperties();

   /**
    * Gets the buffer capacity, i.e. the total the number of values that the buffer can store for each
    * {@link YoVariable}.
    * 
    * @return the buffer size.
    */
   default int getBufferSize()
   {
      return getBufferProperties().getSize();
   }

   /**
    * Gets the current reading/writing position in the buffer.
    * 
    * @return the current buffer index.
    */
   default int getBufferCurrentIndex()
   {
      return getBufferProperties().getCurrentIndex();
   }

   /**
    * Gets the first index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * 
    * @return the in-point index.
    */
   default int getBufferInPoint()
   {
      return getBufferProperties().getInPoint();
   }

   /**
    * Gets the last index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * 
    * @return the out-point index.
    */
   default int getBufferOutPoint()
   {
      return getBufferProperties().getOutPoint();
   }

   /**
    * The period in number of ticks at which data from the {@link YoVariable}s is written into the
    * buffer.
    * 
    * @return the period, in number of run ticks, at which the {@link YoVariable}s are saved into the
    *         buffer.
    */
   int getBufferRecordTickPeriod();

   /**
    * The period in seconds at which data from the {@link YoVariable}s is written into the buffer.
    * 
    * @return the period, in seconds, at which the {@link YoVariable}s are saved into the buffer.
    */
   default double getBufferRecordTimePeriod()
   {
      return getBufferRecordTickPeriod() * getDT();
   }

   /**
    * Gets the instance of this session's buffer.
    * <p>
    * It is not recommended to access and operate directly on the buffer, while the simulation thread
    * is running.
    * </p>
    * 
    * @return the internal buffer.
    */
   YoSharedBuffer getBuffer();

   // ------------------------------------------------------------------------------- //
   // --------------------------- Buffer Controls ----------------------------------- //
   // ------------------------------------------------------------------------------- //

   /**
    * Sets the initial record period in number of ticks for this session.
    * <p>
    * Unlike {@link #setBufferRecordTickPeriod(int)}, this method will change the property only the
    * first time it is invoked. The subsequent calls will be ignored.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param bufferRecordTickPeriod the period in number of ticks that data should be stored in the
    *                               buffer.
    * @return {@code true} if the request is going through, {@code false} if it is being ignored.
    */
   boolean initializeBufferRecordTickPeriod(int bufferRecordTickPeriod);

   /**
    * Sets the period, in number of ticks, at which simulation data should be recorded into the buffer.
    * <p>
    * A larger value allows to store data over longer period of time.
    * </p>
    * 
    * @param bufferRecordTickPeriod the period in number of ticks that data should be stored in the
    *                               buffer.
    */
   void setBufferRecordTickPeriod(int bufferRecordTickPeriod);

   /**
    * Sets the period, in seconds, at which simulation data should be recorded into the buffer.
    * <p>
    * Note that this method relies on the simulation DT to have been set previously.
    * </p>
    * <p>
    * A larger value allows to store data over longer period of time.
    * </p>
    * 
    * @param bufferRecordTickPeriod the period in number of ticks that data should be stored in the
    *                               buffer.
    */
   default void setBufferRecordTimePeriod(double bufferRecordTimePeriod)
   {
      setBufferRecordTickPeriod((int) (bufferRecordTimePeriod / getDT()));
   }

   /**
    * Increments the buffer current index by {@code 1}.
    * 
    * @see #stepBufferIndexForward()
    */
   default void tick()
   {
      stepBufferIndexForward();
   }

   /**
    * Increments the buffer current index by {@code 1} and then writes the current {@code YoVariable}
    * values into the buffer.
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    */
   default void tickAndWrite()
   {
      if (!isPaused())
         return;

      if (isSimulationThreadRunning())
      {
         stopSimulationThread();
         tick();
         getBuffer().writeBuffer();
         startSimulationThread();
      }
      else
      {
         tick();
         getBuffer().writeBuffer();
      }
   }

   /**
    * Writes the current {@code YoVariable} values into the buffer and then increments the buffer
    * current index by {@code 1}.
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    */
   default void writeAndTick()
   {
      if (!isPaused())
         return;

      if (isSimulationThreadRunning())
      {
         stopSimulationThread();
         getBuffer().writeBuffer();
         tick();
         startSimulationThread();
      }
      else
      {
         getBuffer().writeBuffer();
         tick();
      }
   }

   /**
    * Requests to move the current buffer index, i.e. reading/writing position.
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    * 
    * @param bufferIndexRequest the current index to go to.
    */
   void gotoBufferIndex(int bufferIndexRequest);

   /**
    * Requests moving the current index to go to the in-point. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    */
   default void gotoBufferInPoint()
   {
      gotoBufferIndex(getBufferInPoint());
   }

   /**
    * Requests moving the current index to go to the out-point. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    */
   default void gotoBufferOutPoint()
   {
      gotoBufferIndex(getBufferOutPoint());
   }

   /**
    * Requests setting the in-point of the buffer to the current index. (synchronous)
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    */
   default void setBufferInPoint()
   {
      setBufferInPoint(getBufferCurrentIndex());
   }

   /**
    * Requests setting the in-point of the buffer to the given index. (synchronous)
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    * 
    * @param index the new index for the in-point.
    */
   void setBufferInPoint(int index);

   /**
    * Requests setting the out-point of the buffer to the current index. (synchronous)
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    */
   default void setBufferOutPoint()
   {
      setBufferOutPoint(getBufferCurrentIndex());
   }

   /**
    * Requests setting the out-point of the buffer to the given index. (synchronous)
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    * 
    * @param index the new index for the out-point.
    */
   void setBufferOutPoint(int index);

   /**
    * Requests stepping the current index backward by one simulation tick. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
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
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    */
   void stepBufferIndexBackward(int stepSize);

   /**
    * Requests stepping the current index forward by one simulation tick. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
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
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    */
   void stepBufferIndexForward(int stepSize);

   /**
    * Requests cropping the buffer to only keep the active part of the buffer, i.e. between the
    * in-point and out-point indices. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise. This
    * request will cancel any other requests submitted at the same time.
    * </p>
    */
   default void cropBuffer()
   {
      cropBuffer(new CropBufferRequest(getBufferInPoint(), getBufferOutPoint()));
   }

   /**
    * Requests the buffer to be cropped. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise. This
    * request will cancel any other requests submitted at the same time.
    * </p>
    * 
    * @param cropBufferRequest the request.
    * @see CropBufferRequest
    */
   void cropBuffer(CropBufferRequest request);

   /**
    * Sets the initial size of this session's buffer.
    * <p>
    * Unlike {@link #changeBufferSize(int)}, this method will change the buffer size only the first
    * time it is invoked. The subsequent calls will be ignored.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param bufferSize the initial size of the buffer.
    * @return {@code true} if the request is going through, {@code false} if it is being ignored.
    */
   boolean initializeBufferSize(int bufferSize);

   /**
    * Requests to change the size the of the buffer. (synchronous)
    * <p>
    * This is typically used to increased the buffer size. To decrease the buffer size, it is
    * recommended to use a crop request that provides better control on the data being preserved.
    * </p>
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    * 
    * @param bufferSize the new buffer size.
    * @see #submitCropBufferRequest(CropBufferRequest)
    */
   void changeBufferSize(int bufferSize);

   /**
    * Applies a function to the buffer from the in-point to the out-point. (synchronous)
    * <p>
    * This is a blocking request which will return only when the operation has completed.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    * 
    * @param processor the function to apply to the buffer.
    */
   void applyBufferProcessor(YoBufferProcessor processor);

   // ------------------------------------------------------------------------------- //
   // ---------------------------- Misc Controls ------------------------------------ //
   // ------------------------------------------------------------------------------- //

   /**
    * Gets the name of this simulation session.
    * 
    * @return this session name.
    */
   String getSimulationName();

   /**
    * Requests to export this simulation's data to file.
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the simulation is paused, it will be ignored otherwise.
    * </p>
    * 
    * @param sessionDataExportRequest the request.
    * @see SessionDataExportRequest
    */
   void exportData(SessionDataExportRequest request);

   /**
    * Adds a time consumer to be invoked every simulation tick right before the physics engine is
    * invoked.
    * 
    * @param beforePhysicsCallback the time consumer to be invoked every tick.
    */
   void addBeforePhysicsCallback(TimeConsumer beforePhysicsCallback);

   /**
    * Removes a time consumer previously added.
    * 
    * @param beforePhysicsCallback the time consumer to be removed.
    * @return {@code true} if the time consumer was successfully found and removed.
    */
   boolean removeBeforePhysicsCallback(TimeConsumer beforePhysicsCallback);

   /**
    * Adds a time consumer to be invoked every simulation tick right after the physics engine is
    * invoked.
    * 
    * @param afterPhysicsCallback the time consumer to be invoked every tick.
    */
   void addAfterPhysicsCallback(TimeConsumer afterPhysicsCallback);

   /**
    * Removes a time consumer previously added.
    * 
    * @param afterPhysicsCallback the time consumer to be removed.
    * @return {@code true} if the time consumer was successfully found and removed.
    */
   boolean removeAfterPhysicsCallback(TimeConsumer afterPhysicsCallback);
}

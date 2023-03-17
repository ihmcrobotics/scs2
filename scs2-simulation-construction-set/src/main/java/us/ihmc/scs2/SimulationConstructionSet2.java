package us.ihmc.scs2;

import java.net.URL;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.layout.Pane;
import javafx.stage.Window;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.session.SessionDataFilterParameters;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.SceneVideoRecordingRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionChangeListener;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoBooleanProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoEnumAsStringProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoIntegerProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoLongProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.SimulationTerminalCondition;
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.parameters.ContactParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferProcessor;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Convenience class for creating a simulation environment with a JavaFX GUI.
 * <p>
 * Example for using {@code SimulationConstructionSet2} for setting up a simulation that will be
 * controlled via the visualizer:<br>
 * 
 * <pre>
 * SimulationConstructionSet2 scs = new SimulationConstructionSet2();
 * scs.addRobot(robot);
 * robot.addThrottledController(smartRobotController, controllerPeriodInSeconds);
 * scs.start(true, false, false);
 * // Set nice camera angle, there are more methods for setting up the camera.
 * scs.setCameraFocusPosition(0.0, 0.0, 1.0);
 * scs.setCameraPosition(8.0, 0.0, 3.0);
 * // If you want to start simulating right away
 * scs.simulate();
 * // Now the visualizer is up, the user can admire the robot doing smart things, or falling over in a remarkable way.
 * </pre>
 * </p>
 * <p>
 * Example for using {@code SimulationConstructionSet2} for doing an end-to-end simulation test
 * environment:<br>
 * 
 * <pre>
 * SimulationConstructionSet2 scs = new SimulationConstructionSet2();
 * scs.addRobot(robot);
 * robot.addThrottledController(controllerToTestBehavior, controllerPeriodInSeconds);
 * scs.start(true, true, true);
 * // Set nice camera angle, there are more methods for setting up the camera.
 * scs.setCameraFocusPosition(0.0, 0.0, 1.0);
 * scs.setCameraPosition(8.0, 0.0, 3.0);
 * 
 * for (int i = 0; i < numberOfChecks; i++)
 * {
 *    // Provide some input to the controller to test
 *    boolean successfullySimulated = scs.simulateNow(duration); // Simulate for the given duration and return when done
 *    assertTrue(successfullySimulated); // We check that the simulation completed normally without any exceptions.
 *    assert (robotIsWhereItShouldBe); // Perform some assertions
 * 
 *    // Provide some more input to the controller to test
 *    MutableObject<Throwable> robotDidNotDoSoGood = new MutableObject<Throwable>(null);
 *    scs.addSimulationThrowableListener(thrown -> robotDidNotDoSoGood.setValue(thrown)); // Allows to save any exception being thrown by a controller for instance.
 *    scs.addExternalTerminalCondition(() -> robotMadeItBeforeSimulationDuration); // This allows to terminate the next simulation early if the robot made it in advance.
 * 
 *    successfullySimulated = scs.simulateNow(forSomeMoreTime);
 * 
 *    assertTrue(successfullySimulated);
 *    assertNull(robotDidNotDoSoGood.getValue); // We can check that no exception has been thrown for instance (it is redundant with successfullySimulated though).
 * }
 * 
 * // Make sure we pause before resuming the thread.
 * scs.pause();
 * // Re-start the simulation thread so the visualizer is functional.
 * scs.startSimulationThread();
 * 
 * if (keepVisualizerUp)
 * { // This will cause this thread to pause and resume only once the user closes the visualizer.
 *    scs.waitUntilVisualizerDown();
 * }
 * 
 * // On shutdown, the visualizer already does that, so just being extra cautious here.
 * scs.shutdownSession();
 * // Free up that memory
 * scs = null;
 * </pre>
 * </p>
 * <p>
 * Example for using {@code SimulationConstructionSet2} for visualization:<br>
 * 
 * <pre>
 * // Use the do-nothing physics engine, that way you have 100% control on what the robot is doing.
 * SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.doNothingPhysicsEngine());
 * scs.addRobot(theRobotToVisualize);
 * scs.start(true, true, true);
 * // Set nice camera angle, there are more methods for setting up the camera.
 * scs.setCameraFocusPosition(0.0, 0.0, 1.0);
 * scs.setCameraPosition(8.0, 0.0, 3.0);
 * 
 * for (int i = 0; i < numberOfThingsToDo; i++)
 * {
 *    // Do smart calculations
 *    theRobotToVisualize.getOneDoFJoint("aJoint").setQ(theAngleComputed);
 *    // This will record the data in the buffer and also let the visualizer know to update its graphics.
 *    scs.simulateNow(1);
 * }
 * 
 * // Make sure we pause before resuming the thread.
 * scs.pause();
 * // Re-start the simulation thread so the visualizer is functional.
 * scs.startSimulationThread();
 * 
 * if (keepVisualizerUp)
 * { // This will cause this thread to pause and resume only once the user closes the visualizer.
 *    scs.waitUntilVisualizerDown();
 * }
 * 
 * // On shutdown, the visualizer already does that, so just being extra cautious here.
 * scs.shutdownSession();
 * // Free up that memory
 * scs = null;
 * </pre>
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class SimulationConstructionSet2 implements YoVariableHolder, SimulationSessionControls, SessionVisualizerControls
{
   public static final ReferenceFrame inertialFrame = SimulationSession.DEFAULT_INERTIAL_FRAME;

   /**
    * Default value for {@link #visualizerEnabled}. If the system property is not set, it is
    * {@code true} by default.
    */
   public static final boolean DEFAULT_VISUALIZER_ENABLED = SessionPropertiesHelper.loadBooleanProperty("create.scs.gui", true)
                                                            && SessionPropertiesHelper.loadBooleanProperty("scs2.disablegui", true, false);

   private final SimulationSession simulationSession;

   private final SimulationSessionControls simulationSessionControls;
   private SessionVisualizerControls visualizerControls;
   private ConcurrentLinkedQueue<Runnable> pendingVisualizerTasks = null;

   private boolean visualizerEnabled = DEFAULT_VISUALIZER_ENABLED;
   private boolean shutdownSessionOnVisualizerClose = true;
   /**
    * This is initialized to {@code null} such that the JavaFX flag is not set by default, allowing the
    * user to set it from outside.
    */
   private Boolean javaFXThreadImplicitExit = null;

   /**
    * Factory for setting up a contact point based physics engine. It is the default physics engine and
    * is an adaptation of SCS1's physics engine.
    * 
    * @return the physics engine factory.
    */
   public static PhysicsEngineFactory contactPointBasedPhysicsEngineFactory()
   {
      return PhysicsEngineFactory.newContactPointBasedPhysicsEngineFactory();
   }

   /**
    * Factory for setting up a contact point based physics engine. It is the default physics engine and
    * is an adaptation of SCS1's physics engine.
    * 
    * @param contactParameters the parameters to use for resolving contacts.
    * @return the physics engine factory.
    */
   public static PhysicsEngineFactory contactPointBasedPhysicsEngineFactory(ContactPointBasedContactParameters contactParameters)
   {
      return PhysicsEngineFactory.newContactPointBasedPhysicsEngineFactory(contactParameters);
   }

   /**
    * Factory for setting up an impulse based physics engine. It is still at the experimental phase but
    * can handle complex contact interactions.
    * 
    * @return the physics engine factory.
    */
   public static PhysicsEngineFactory impulseBasedPhysicsEngineFactory()
   {
      return impulseBasedPhysicsEngineFactory(null);
   }

   /**
    * Factory for setting up an impulse based physics engine. It is still at the experimental phase but
    * can handle complex contact interactions.
    * 
    * @param contactParameters the parameters to use for resolving contacts.
    * @return the physics engine factory.
    */
   public static PhysicsEngineFactory impulseBasedPhysicsEngineFactory(ContactParametersReadOnly contactParameters)
   {
      return PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters);
   }

   /**
    * Factory for setting up a physics engine that does close to nothing.
    * <p>
    * This is convenient for using the simulation environment for visualization. The physics engine
    * will still run the controllers attached to the robot and apply their output, but that's it. The
    * robot sensors are not updated.
    * </p>
    * 
    * @return the physics engine factory.
    */
   public static PhysicsEngineFactory doNothingPhysicsEngine()
   {
      return PhysicsEngineFactory.newDoNothingPhysicsEngineFactory();
   }

   /**
    * Creates a new simulation environment.
    * <ul>
    * <li>See {@link #addRobot(RobotDefinition)} for adding robots to the simulation.
    * <li>See {@link #addTerrainObject(TerrainObjectDefinition)} for adding objects to the environment.
    * <li>Call {@link #startSimulationThread()} to fire up the environment before simulating.
    * </ul>
    */
   public SimulationConstructionSet2()
   {
      this(Session.retrieveCallerName(), contactPointBasedPhysicsEngineFactory());
   }

   /**
    * Creates a new simulation environment.
    * <ul>
    * <li>See {@link #addRobot(RobotDefinition)} for adding robots to the simulation.
    * <li>See {@link #addTerrainObject(TerrainObjectDefinition)} for adding objects to the environment.
    * <li>Call {@link #startSimulationThread()} to fire up the environment before simulating.
    * </ul>
    * 
    * @param simulationName the name of the simulation.
    */
   public SimulationConstructionSet2(String simulationName)
   {
      this(simulationName, contactPointBasedPhysicsEngineFactory());
   }

   /**
    * Creates a new simulation environment.
    * <ul>
    * <li>See {@link #addRobot(RobotDefinition)} for adding robots to the simulation.
    * <li>See {@link #addTerrainObject(TerrainObjectDefinition)} for adding objects to the environment.
    * <li>Call {@link #startSimulationThread()} to fire up the environment before simulating.
    * </ul>
    * 
    * @param physicsEngineFactory the factory to use for setting the physics engine.
    */
   public SimulationConstructionSet2(PhysicsEngineFactory physicsEngineFactory)
   {
      this(Session.retrieveCallerName(), physicsEngineFactory);
   }

   /**
    * Creates a new simulation environment.
    * <ul>
    * <li>See {@link #addRobot(RobotDefinition)} for adding robots to the simulation.
    * <li>See {@link #addTerrainObject(TerrainObjectDefinition)} for adding objects to the environment.
    * <li>Call {@link #startSimulationThread()} to fire up the environment before simulating.
    * </ul>
    * 
    * @param simulationName       the name of the simulation.
    * @param physicsEngineFactory the factory to use for setting the physics engine.
    */
   public SimulationConstructionSet2(String simulationName, PhysicsEngineFactory physicsEngineFactory)
   {
      simulationSession = new SimulationSession(inertialFrame, simulationName, physicsEngineFactory);
      simulationSessionControls = simulationSession.getSimulationSessionControls();
   }

   /**
    * Gets the internal session.
    * 
    * @return the simulation session.
    */
   public SimulationSession getSimulationSession()
   {
      return simulationSession;
   }

   /**
    * Gets the instance of the physics engine used in this simulation.
    * 
    * @return the physics engine.
    */
   public PhysicsEngine getPhysicsEngine()
   {
      return simulationSession.getPhysicsEngine();
   }

   /**
    * Sets whether the visualizer should be created.
    *
    * @param visualizerEnabled whether to create the visualizer or not.
    */
   public void setVisualizerEnabled(boolean visualizerEnabled)
   {
      this.visualizerEnabled = visualizerEnabled;
   }

   /**
    * Gets whether the visualizer is to be created or not.
    *
    * @return {@code true} if the visualizer is to be created.
    */
   public boolean isVisualizerEnabled()
   {
      return visualizerEnabled;
   }

   /**
    * Specifies whether the closing visualizer should result in shutting down the simulation as well.
    * <p>
    * When {@code false}, the visualizer can be closed and then restarted via
    * {@link #startSimulationThread()}.
    * </p>
    * 
    * @param shutdownSessionOnVisualizerClose whether the visualizer should shut down the simulation
    *                                         when being closed. Default value {@code true}.
    */
   public void setShutdownSessionOnVisualizerClose(boolean shutdownSessionOnVisualizerClose)
   {
      this.shutdownSessionOnVisualizerClose = shutdownSessionOnVisualizerClose;
   }

   public boolean isShutdownSessionOnVisualizerClose()
   {
      return shutdownSessionOnVisualizerClose;
   }

   /**
    * Sets the JavaFX implicit exit flag.
    * <p>
    * If {@code true}, the main JavaFX thread will terminate when closing the last window. Once the
    * JavaFX thread is terminated, it is not possible to restart it. Sets this to false to make
    * possible to start a new session with the same JVM.
    * </p>
    * <p>
    * If {@code false}, the main JavaFX thread will only terminate when explicitly requested via
    * {@link Platform#exit()} for instance. This will cause the JVM to not terminate even after
    * shutting down this simulation.
    * </p>
    * <p>
    * If {@code null}, the JavaFX implicit exit flag is not modified.
    * </p>
    *
    * @param javaFXThreadImplicitExit a flag indicating whether or not to implicitly exit when the last
    *                                 window is closed.
    * @see Platform#setImplicitExit(boolean)
    */
   public void setJavaFXThreadImplicitExit(Boolean javaFXThreadImplicitExit)
   {
      this.javaFXThreadImplicitExit = javaFXThreadImplicitExit;
   }

   /**
    * Gets the JavaFX implicit exit flag value.
    * <p>
    * If {@code true}, the main JavaFX thread will terminate when closing the last window. Once the
    * JavaFX thread is terminated, it is not possible to restart it. Sets this to false to make
    * possible to start a new session with the same JVM.
    * </p>
    * <p>
    * If {@code false}, the main JavaFX thread will only terminate when explicitly requested via
    * {@link Platform#exit()} for instance. This will cause the JVM to not terminate even after
    * shutting down this simulation.
    * </p>
    * <p>
    * If {@code null}, the JavaFX implicit exit flag is not modified.
    * </p>
    *
    * @return the flag current value.
    * @see Platform#setImplicitExit(boolean)
    */
   public Boolean isJavaFXThreadImplicitExit()
   {
      return javaFXThreadImplicitExit;
   }

   /**
    * Gets the list of robots being simulated.
    * 
    * @return the simulated robots.
    */
   public List<? extends Robot> getRobots()
   {
      return getPhysicsEngine().getRobots();
   }

   /**
    * Adds a robot to this simulation.
    * 
    * @param robotDefinition the definition to use for creating a new robot to add to this simulation.
    * @return the instantiated robot.
    */
   public Robot addRobot(RobotDefinition robotDefinition)
   {
      return simulationSession.addRobot(robotDefinition);
   }

   /**
    * Adds a robot to this simulation.
    * 
    * @param robot the robot to add.
    */
   public void addRobot(Robot robot)
   {
      simulationSession.addRobot(robot);
   }

   /**
    * Adds a robots to this simulation.
    * 
    * @param robots the robots to add.
    */
   public void addRobots(Collection<? extends Robot> robots)
   {
      simulationSession.addRobots(robots);
   }

   /**
    * Adds a terrain (static) object to the environment.
    * 
    * @param terrainObjectDefinition the definition used to create the new terrain object.
    */
   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      simulationSession.addTerrainObject(terrainObjectDefinition);
   }

   /**
    * Adds terrain (static) objects to the environment.
    * 
    * @param terrainObjectDefinitions the definitions used to create the new terrain objects.
    */
   public void addTerrainObjects(Collection<? extends TerrainObjectDefinition> terrainObjectDefinitions)
   {
      simulationSession.addTerrainObjects(terrainObjectDefinitions);
   }

   /**
    * Convenience method for start the simulation environment and configuring a couple things at once.
    * 
    * @param waitUntilVisualizerFullyUp setting it to {@code true} is recommended, especially if
    *                                   {@link ReferenceFrame}s or {@link YoVariable}s are to be added
    *                                   right after calling this method. This will let the visualizer
    *                                   initialize before returning.
    * @param stopSimulationThread       setting it to {@code true} is recommended if you are planning
    *                                   on calling {@link #simulateNow(long)} many times, for
    *                                   performance reason. You will need to restart the simulation
    *                                   thread for the visualizer to be operational.
    * @param disableJavaFXImplicitExit  in the case SCS2 will be closed and then reopened in the same
    *                                   JVM, the JavaFX implicit exit has to be disabled or SCS2 will
    *                                   throw an exception when trying to re-open it.
    */
   public void start(boolean waitUntilVisualizerFullyUp, boolean stopSimulationThread, boolean disableJavaFXImplicitExit)
   {
      if (disableJavaFXImplicitExit)
         setJavaFXThreadImplicitExit(false);

      startSimulationThread();

      if (waitUntilVisualizerFullyUp)
         waitUntilVisualizerFullyUp();

      if (stopSimulationThread)
         stopSimulationThread();
   }

   /**
    * Gets the variable holding the current time (in seconds) in this simulation.
    * 
    * @return the current time (in seconds) variable.
    */
   public YoDouble getTime()
   {
      return simulationSession.getTime();
   }

   /**
    * Gets the inertial frame used for this simulation.
    * <p>
    * It is typically <b>not</b> equal to {@link ReferenceFrame#getWorldFrame()}.
    * </p>
    * 
    * @return the inertial frame.
    */
   public ReferenceFrame getInertialFrame()
   {
      return simulationSession.getInertialFrame();
   }

   /**
    * Gets the internal reference to the gravitational acceleration.
    * <p>
    * This vector can be modified to change gravity for this simulation.
    * </p>
    * 
    * @return the gravity vector used for this simulation. Default value is {@code (0, 0, -9.81)}.
    */
   public YoFrameVector3D getGravity()
   {
      return simulationSession.getGravity();
   }

   /**
    * Reinitializes the physics engine.
    * <p>
    * Can be useful for resetting the simulation.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    */
   public void reinitializeSimulation()
   {
      simulationSession.reinitializeSession();
   }

   // ------------------------------------------------------------------------------- //
   // ----------------------------- YoVariables ------------------------------------- //
   // ------------------------------------------------------------------------------- //

   /**
    * Gets the simulation's root registry.
    * 
    * @return the root registry.
    */
   public YoRegistry getRootRegistry()
   {
      return simulationSession.getRootRegistry();
   }

   /**
    * Adds a registry to the simulation's root registry.
    * 
    * @param registry the registry to add.
    */
   public void addRegistry(YoRegistry registry)
   {
      getRootRegistry().addChild(registry);
   }

   /** {@inheritDoc} */
   @Override
   public List<YoVariable> getVariables()
   {
      return getRootRegistry().getVariables();
   }

   /** {@inheritDoc} */
   @Override
   public YoVariable findVariable(String namespaceEnding, String name)
   {
      return getRootRegistry().findVariable(namespaceEnding, name);
   }

   /** {@inheritDoc} */
   @Override
   public List<YoVariable> findVariables(String namespaceEnding, String name)
   {
      return getRootRegistry().findVariables(namespaceEnding, name);
   }

   /** {@inheritDoc} */
   @Override
   public List<YoVariable> findVariables(YoNamespace namespace)
   {
      return getRootRegistry().findVariables(namespace);
   }

   /** {@inheritDoc} */
   @Override
   public boolean hasUniqueVariable(String namespaceEnding, String name)
   {
      return getRootRegistry().hasUniqueVariable(namespaceEnding, name);
   }

   // ------------------------------------------------------------------------------- //
   // ------------------------ Simulation Properties -------------------------------- //
   // ------------------------------------------------------------------------------- //

   /** {@inheritDoc} */
   @Override
   public double getDT()
   {
      return simulationSessionControls.getDT();
   }

   /** {@inheritDoc} */
   @Override
   public void setDT(double dt)
   {
      simulationSessionControls.setDT(dt);
   }

   /** {@inheritDoc} */
   @Override
   public boolean isSimulationThreadRunning()
   {
      return simulationSessionControls.isSimulationThreadRunning();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isRealTimeRateSimulation()
   {
      return simulationSessionControls.isRealTimeRateSimulation();
   }

   /** {@inheritDoc} */
   @Override
   public double getPlaybackRealTimeRate()
   {
      return simulationSession.getPlaybackRealTimeRate();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isSimulating()
   {
      return simulationSessionControls.isSimulating();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isPlaying()
   {
      return simulationSessionControls.isPlaying();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isPaused()
   {
      return simulationSessionControls.isPaused();
   }

   // ------------------------------------------------------------------------------- //
   // ------------------------- Simulation Controls --------------------------------- //
   // ------------------------------------------------------------------------------- //

   /**
    * {@inheritDoc}
    * <p>
    * Starts the visualizer thread as well when called for the first and if the visualizer is enabled.
    * </p>
    * <p>
    * It is recommended to call {@link #waitUntilVisualizerFullyUp()} right after starting the
    * simulation thread.
    * </p>
    *
    * @see #setVisualizerEnabled(boolean)
    */
   @Override
   public boolean startSimulationThread()
   {
      if (simulationSession.isSessionShutdown())
         return false;

      if (!isSimulationThreadRunning())
         pause();

      boolean started = simulationSession.startSessionThread();

      if (visualizerEnabled && visualizerControls == null)
      {
         visualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession, javaFXThreadImplicitExit, shutdownSessionOnVisualizerClose);

         if (pendingVisualizerTasks != null && !pendingVisualizerTasks.isEmpty())
         {
            visualizerControls.waitUntilVisualizerFullyUp();

            // Executing the visualizer tasks 1-by-1 in sync with the JavaFX thread.
            // This way, the tasks should be executed in the same order as they were submitted.
            new ObservedAnimationTimer("VisualizerTasksExecutor")
            {
               @Override
               public void handleImpl(long now)
               {
                  if (pendingVisualizerTasks == null)
                  {
                     stop();
                     return;
                  }
                  else
                  {
                     if (!pendingVisualizerTasks.isEmpty())
                     {
                        pendingVisualizerTasks.poll().run();
                     }

                     if (pendingVisualizerTasks.isEmpty())
                     {
                        pendingVisualizerTasks = null;
                        stop();
                     }
                  }
               }
            }.start();
         }

         if (shutdownSessionOnVisualizerClose)
            visualizerControls.addVisualizerShutdownListener(this::shutdownSession);
      }
      return started;
   }

   /** {@inheritDoc} */
   @Override
   public boolean stopSimulationThread()
   {
      return simulationSessionControls.stopSimulationThread();
   }

   /** {@inheritDoc} */
   @Override
   public void setRealTimeRateSimulation(boolean enableRealTimeRate)
   {
      simulationSessionControls.setRealTimeRateSimulation(enableRealTimeRate);
   }

   /** {@inheritDoc} */
   @Override
   public void setPlaybackRealTimeRate(double realTimeRate)
   {
      simulationSession.submitPlaybackRealTimeRate(realTimeRate);
   }

   /** {@inheritDoc} */
   @Override
   public void simulate(double duration)
   {
      simulationSessionControls.simulate(duration);
   }

   /** {@inheritDoc} */
   @Override
   public void simulate(int numberOfTicks)
   {
      simulationSessionControls.simulate(numberOfTicks);
   }

   /** {@inheritDoc} */
   @Override
   public boolean simulateNow(long numberOfTicks)
   {
      return simulationSessionControls.simulateNow(numberOfTicks);
   }

   /** {@inheritDoc} */
   @Override
   public void addSimulationThrowableListener(Consumer<Throwable> listener)
   {
      simulationSessionControls.addSimulationThrowableListener(listener);
   }

   /** {@inheritDoc} */
   @Override
   public void addExternalTerminalCondition(SimulationTerminalCondition... externalTerminalConditions)
   {
      simulationSessionControls.addExternalTerminalCondition(externalTerminalConditions);
   }

   /** {@inheritDoc} */
   @Override
   public boolean removeExternalTerminalCondition(SimulationTerminalCondition externalTerminalCondition)
   {
      return simulationSessionControls.removeExternalTerminalCondition(externalTerminalCondition);
   }

   /** {@inheritDoc} */
   @Override
   public void clearExternalTerminalConditions()
   {
      simulationSessionControls.clearExternalTerminalConditions();
   }

   /** {@inheritDoc} */
   @Override
   public void play()
   {
      simulationSessionControls.play();
   }

   /** {@inheritDoc} */
   @Override
   public void pause()
   {
      simulationSessionControls.pause();
   }

   // ------------------------------------------------------------------------------- //
   // -------------------------- Buffer Properties ---------------------------------- //
   // ------------------------------------------------------------------------------- //

   /** {@inheritDoc} */
   @Override
   public YoBufferPropertiesReadOnly getBufferProperties()
   {
      return simulationSessionControls.getBufferProperties();
   }

   /** {@inheritDoc} */
   @Override
   public int getBufferRecordTickPeriod()
   {
      return simulationSessionControls.getBufferRecordTickPeriod();
   }

   /** {@inheritDoc} */
   @Override
   public YoSharedBuffer getBuffer()
   {
      return simulationSessionControls.getBuffer();
   }

   // ------------------------------------------------------------------------------- //
   // --------------------------- Buffer Controls ----------------------------------- //
   // ------------------------------------------------------------------------------- //

   /** {@inheritDoc} */
   @Override
   public boolean initializeBufferRecordTickPeriod(int bufferRecordTickPeriod)
   {
      return simulationSessionControls.initializeBufferRecordTickPeriod(bufferRecordTickPeriod);
   }

   /** {@inheritDoc} */
   @Override
   public void setBufferRecordTickPeriod(int bufferRecordTickPeriod)
   {
      simulationSessionControls.setBufferRecordTickPeriod(bufferRecordTickPeriod);
   }

   /** {@inheritDoc} */
   @Override
   public void tick()
   {
      simulationSessionControls.tick();
   }

   /** {@inheritDoc} */
   @Override
   public void gotoBufferIndex(int bufferIndexRequest)
   {
      simulationSessionControls.gotoBufferIndex(bufferIndexRequest);
   }

   /** {@inheritDoc} */
   @Override
   public void setBufferInPoint(int index)
   {
      simulationSessionControls.setBufferInPoint(index);
   }

   /** {@inheritDoc} */
   @Override
   public void setBufferOutPoint(int index)
   {
      simulationSessionControls.setBufferOutPoint(index);
   }

   /** {@inheritDoc} */
   @Override
   public void stepBufferIndexBackward(int stepSize)
   {
      stepBufferIndexBackward(stepSize);
   }

   /** {@inheritDoc} */
   @Override
   public void stepBufferIndexForward(int stepSize)
   {
      simulationSessionControls.stepBufferIndexForward(stepSize);
   }

   /** {@inheritDoc} */
   @Override
   public void cropBuffer(CropBufferRequest request)
   {
      simulationSessionControls.cropBuffer(request);
   }

   /** {@inheritDoc} */
   @Override
   public boolean initializeBufferSize(int bufferSize)
   {
      return simulationSessionControls.initializeBufferSize(bufferSize);
   }

   /** {@inheritDoc} */
   @Override
   public void changeBufferSize(int bufferSize)
   {
      simulationSessionControls.changeBufferSize(bufferSize);
   }

   /** {@inheritDoc} */
   @Override
   public void applyBufferProcessor(YoBufferProcessor processor)
   {
      simulationSessionControls.applyBufferProcessor(processor);
   }

   // ------------------------------------------------------------------------------- //
   // ---------------------------- Misc Controls ------------------------------------ //
   // ------------------------------------------------------------------------------- //

   /** {@inheritDoc} */
   @Override
   public String getSimulationName()
   {
      return simulationSessionControls.getSimulationName();
   }

   /** {@inheritDoc} */
   @Override
   public void exportData(SessionDataExportRequest request)
   {
      simulationSessionControls.exportData(request);
   }

   /** {@inheritDoc} */
   @Override
   public void addBeforePhysicsCallback(TimeConsumer beforePhysicsCallback)
   {
      simulationSessionControls.addBeforePhysicsCallback(beforePhysicsCallback);
   }

   /** {@inheritDoc} */
   @Override
   public boolean removeBeforePhysicsCallback(TimeConsumer beforePhysicsCallback)
   {
      return simulationSessionControls.removeBeforePhysicsCallback(beforePhysicsCallback);
   }

   /** {@inheritDoc} */
   @Override
   public void addAfterPhysicsCallback(TimeConsumer afterPhysicsCallback)
   {
      simulationSessionControls.addAfterPhysicsCallback(afterPhysicsCallback);
   }

   /** {@inheritDoc} */
   @Override
   public boolean removeAfterPhysicsCallback(TimeConsumer afterPhysicsCallback)
   {
      return simulationSessionControls.removeAfterPhysicsCallback(afterPhysicsCallback);
   }

   // ------------------------------------------------------------------------------- //
   // ------------------------- Visualizer Controls --------------------------------- //
   // ------------------------------------------------------------------------------- //

   private void executeOrScheduleVisualizerTask(Runnable task)
   {
      if (visualizerControls != null)
      {
         task.run();
      }
      else if (visualizerEnabled && !isVisualizerShutdown())
      {
         if (pendingVisualizerTasks == null)
            pendingVisualizerTasks = new ConcurrentLinkedQueue<>();
         pendingVisualizerTasks.add(task);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setCameraOrientation(double latitude, double longitude)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setCameraOrientation(latitude, longitude));
   }

   /** {@inheritDoc} */
   @Override
   public void setCameraPosition(double x, double y, double z)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setCameraPosition(x, y, z));
   }

   /** {@inheritDoc} */
   @Override
   public void setCameraFocusPosition(double x, double y, double z)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setCameraFocusPosition(x, y, z));
   }

   /** {@inheritDoc} */
   @Override
   public void setCameraZoom(double distanceFromFocus)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setCameraZoom(distanceFromFocus));
   }

   /** {@inheritDoc} */
   @Override
   public void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.requestCameraRigidBodyTracking(robotName, rigidBodyName));
   }

   /** {@inheritDoc} */
   @Override
   public void showOverheadPlotter2D(boolean show)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.showOverheadPlotter2D(show));
   }

   /** {@inheritDoc} */
   @Override
   public void requestPlotter2DCoordinateTracking(String xVariableName, String yVariableName, String frameName)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.requestPlotter2DCoordinateTracking(xVariableName, yVariableName, frameName));
   }

   /** {@inheritDoc} */
   @Override
   public void addStaticVisual(VisualDefinition visualDefinition)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.addStaticVisual(visualDefinition));
   }

   /** {@inheritDoc} */
   @Override
   public void removeStaticVisual(VisualDefinition visualDefinition)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.removeStaticVisual(visualDefinition));
   }

   /** {@inheritDoc} */
   @Override
   public void removeYoGraphic(String name)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.removeYoGraphic(name));
   }

   /** {@inheritDoc} */
   @Override
   public void setYoGraphicVisible(String name, boolean visible)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setYoGraphicVisible(name, visible));
   }

   /** {@inheritDoc} */
   @Override
   public void addYoGraphic(YoGraphicDefinition yoGraphicDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.addYoGraphic(yoGraphicDefinition);
      else // It is possible that the simulation hasn't been started yet, add the graphic to the session instead.
         simulationSession.addYoGraphicDefinition(yoGraphicDefinition);
   }

   /** {@inheritDoc} */
   @Override
   public void addYoEntry(String groupName, Collection<String> variableNames)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.addYoEntry(groupName, variableNames));
   }

   @Override
   public void clearAllSliderboards()
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.clearAllSliderboards());
   }

   @Override
   public void setSliderboards(YoSliderboardListDefinition sliderboardListDefinition)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setSliderboards(sliderboardListDefinition));
   }

   @Override
   public void setSliderboard(YoSliderboardDefinition sliderboardConfiguration)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setSliderboard(sliderboardConfiguration));
   }

   @Override
   public void removeSliderboard(String sliderboardName)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.removeSliderboard(sliderboardName));
   }

   @Override
   public void setSliderboardButton(String sliderboardName, YoButtonDefinition buttonDefinition)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setSliderboardButton(sliderboardName, buttonDefinition));
   }

   @Override
   public void clearSliderboardButton(String sliderboardName, int buttonIndex)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.clearSliderboardButton(sliderboardName, buttonIndex));
   }

   @Override
   public void setSliderboardKnob(String sliderboardName, YoKnobDefinition knobDefinition)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setSliderboardKnob(sliderboardName, knobDefinition));
   }

   @Override
   public void clearSliderboardKnob(String sliderboardName, int knobIndex)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.clearSliderboardSlider(sliderboardName, knobIndex));
   }

   @Override
   public void setSliderboardSlider(String sliderboardName, YoSliderDefinition sliderDefinition)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.setSliderboardSlider(sliderboardName, sliderDefinition));
   }

   @Override
   public void clearSliderboardSlider(String sliderboardName, int sliderIndex)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.clearSliderboardSlider(sliderboardName, sliderIndex));
   }

   @Override
   public void addSessionDataFilterParameters(SessionDataFilterParameters filterParameters)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.addSessionDataFilterParameters(filterParameters));
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public void exportVideo(SceneVideoRecordingRequest request)
   {
      if (visualizerControls != null)
         visualizerControls.exportVideo(request);
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public void disableGUIControls()
   {
      if (visualizerControls != null)
         visualizerControls.disableGUIControls();
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public void enableGUIControls()
   {
      if (visualizerControls != null)
         visualizerControls.enableGUIControls();
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public Window getPrimaryGUIWindow()
   {
      return visualizerControls == null ? null : visualizerControls.getPrimaryGUIWindow();
   }

   /** {@inheritDoc} */
   @Override
   public void addCustomGUIControl(Node control)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.addCustomGUIControl(control));
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public boolean removeCustomGUIControl(Node control)
   {
      if (visualizerControls != null)
         return visualizerControls.removeCustomGUIControl(control);
      else
         return false;
   }

   /** {@inheritDoc} */
   @Override
   public void loadCustomGUIPane(String name, URL fxmlResource)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.loadCustomGUIPane(name, fxmlResource));
   }

   /** {@inheritDoc} */
   @Override
   public void addCustomGUIPane(String name, Pane pane)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.addCustomGUIPane(name, pane));
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public boolean removeCustomGUIPane(String name)
   {
      if (visualizerControls != null)
         return visualizerControls.removeCustomGUIPane(name);
      else
         return false;
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public YoDoubleProperty newYoDoubleProperty(String variableName)
   {
      if (visualizerControls != null)
         return visualizerControls.newYoDoubleProperty(variableName);
      else
         return null;
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public YoBooleanProperty newYoBooleanProperty(String variableName)
   {
      if (visualizerControls != null)
         return visualizerControls.newYoBooleanProperty(variableName);
      else
         return null;
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public YoIntegerProperty newYoIntegerProperty(String variableName)
   {
      if (visualizerControls != null)
         return visualizerControls.newYoIntegerProperty(variableName);
      else
         return null;
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public YoLongProperty newYoLongProperty(String variableName)
   {
      if (visualizerControls != null)
         return visualizerControls.newYoLongProperty(variableName);
      else
         return null;
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public <E extends Enum<E>> YoEnumAsStringProperty<E> newYoEnumProperty(String variableName)
   {
      if (visualizerControls != null)
         return visualizerControls.newYoEnumProperty(variableName);
      else
         return null;
   }

   /** {@inheritDoc} */
   @Override
   public void addSessionChangedListener(SessionChangeListener listener)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.addSessionChangedListener(listener));
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public boolean removeSessionChangedListener(SessionChangeListener listener)
   {
      if (visualizerControls != null)
         return visualizerControls.removeSessionChangedListener(listener);
      else
         return false;
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public void waitUntilVisualizerFullyUp()
   {
      if (visualizerControls != null)
         visualizerControls.waitUntilVisualizerFullyUp();
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public void waitUntilVisualizerDown()
   {
      if (visualizerControls != null)
      {
         // We only going to use the visualizer, so let's make sure the simulation thread is running first.
         if (!simulationSessionControls.isSimulationThreadRunning())
            simulationSessionControls.startSimulationThread();
         visualizerControls.waitUntilVisualizerDown();
         visualizerControls = null;
      }
   }

   // TODO Missing setupGraph, setupGraphGroup

   // ------------------------------------------------------------------------------- //
   // -------------------------- Shutdown Controls ---------------------------------- //
   // ------------------------------------------------------------------------------- //

   /** {@inheritDoc} */
   @Override
   public boolean isSessionShutdown()
   {
      return simulationSessionControls.isSessionShutdown();
   }

   private boolean hasBeenDestroyed;

   /** {@inheritDoc} */
   @Override
   public void shutdownSession()
   {
      if (hasBeenDestroyed)
         return;

      hasBeenDestroyed = true;

      if (visualizerControls != null)
         visualizerControls.shutdownSession();

      // The visualizer will shutdown the session, just being extra cautious here.
      simulationSessionControls.shutdownSession();
   }

   /**
    * {@inheritDoc}
    * <p>
    * The visualizer has to be running for this method to be effective. See
    * {@link #start(boolean, boolean, boolean)} and {@link #setVisualizerEnabled(boolean)}.
    * </p>
    */
   @Override
   public void requestVisualizerShutdown()
   {
      if (visualizerControls != null)
         visualizerControls.requestVisualizerShutdown();
   }

   /** {@inheritDoc} */
   @Override
   public void addSessionShutdownListener(Runnable listener)
   {
      simulationSessionControls.addSessionShutdownListener(listener);
   }

   /** {@inheritDoc} */
   @Override
   public void addVisualizerShutdownListener(Runnable listener)
   {
      executeOrScheduleVisualizerTask(() -> visualizerControls.addVisualizerShutdownListener(listener));
   }

   /** {@inheritDoc} */
   @Override
   public boolean isVisualizerShutdown()
   {
      if (visualizerControls != null)
         return visualizerControls.isVisualizerShutdown();
      else if (visualizerEnabled)
         return false; // It has not been started yet.
      else
         return true; // The visualizer will not be created this time.
   }
}

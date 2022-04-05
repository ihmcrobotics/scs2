package us.ihmc.scs2.simulation;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.TopicListener;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.DaemonThreadFactory;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMessagerAPI;
import us.ihmc.scs2.session.SessionMessagerAPI.Sensors.SensorMessage;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sharedMemory.interfaces.LinkedYoVariableFactory;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.sensors.SimCameraSensor;
import us.ihmc.scs2.simulation.robot.sensors.SimCameraSensor.CameraDefinitionConsumer;
import us.ihmc.scs2.simulation.robot.sensors.SimCameraSensor.CameraFrameConsumer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class SimulationSession extends Session
{
   public static final ReferenceFrame DEFAULT_INERTIAL_FRAME = ReferenceFrameTools.constructARootFrame("worldFrame");

   private final PhysicsEngine physicsEngine;
   private final YoFrameVector3D gravity = new YoFrameVector3D("gravity", ReferenceFrame.getWorldFrame(), rootRegistry);
   private final String simulationName;
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();
   private final List<Consumer<SensorMessage<CameraSensorDefinition>>> cameraDefinitionListeners = new ArrayList<>();
   private final List<CameraDefinitionConsumer> cameraDefinitionNotifiers = new ArrayList<>();
   private final Map<String, Map<String, SimCameraSensor>> robotNameToSensorNameToCameraMap = new HashMap<>();
   private final ExecutorService cameraBroadcastExecutor = Executors.newSingleThreadExecutor(new DaemonThreadFactory("SCS2-Camera-Server"));
   private final List<BooleanSupplier> terminalConditions = new ArrayList<>();

   private final List<TimeConsumer> beforePhysicsCallbacks = new ArrayList<>();
   private final List<TimeConsumer> afterPhysicsCallbacks = new ArrayList<>();

   private final List<Runnable> cleanupActions = new ArrayList<>();

   private SimulationSessionControlsImpl controls = null;

   private boolean hasSessionStarted = false;

   public SimulationSession()
   {
      this(retrieveCallerName());
   }

   public SimulationSession(PhysicsEngineFactory physicsEngineFactory)
   {
      this(retrieveCallerName(), physicsEngineFactory);
   }

   public SimulationSession(String simulationName)
   {
      this(DEFAULT_INERTIAL_FRAME, simulationName);
   }

   public SimulationSession(ReferenceFrame inertialFrame, String simulationName)
   {
      this(inertialFrame, simulationName, PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory());
   }

   public SimulationSession(String simulationName, PhysicsEngineFactory physicsEngineFactory)
   {
      this(DEFAULT_INERTIAL_FRAME, simulationName, physicsEngineFactory);
   }

   public SimulationSession(ReferenceFrame inertialFrame, String simulationName, PhysicsEngineFactory physicsEngineFactory)
   {
      super(inertialFrame);

      if (!inertialFrame.isRootFrame())
         throw new IllegalArgumentException("The given inertialFrame is not a root frame: " + inertialFrame);

      this.simulationName = simulationName;

      physicsEngine = physicsEngineFactory.build(inertialFrame, rootRegistry);
      sessionRegistry.addChild(physicsEngine.getPhysicsEngineRegistry());

      setSessionDTSeconds(0.0001);
      setSessionMode(SessionMode.PAUSE);
      gravity.set(0.0, 0.0, -9.81);
   }

   public SimulationSessionControls getSimulationSessionControls()
   {
      if (controls == null)
         controls = new SimulationSessionControlsImpl();
      return controls;
   }

   @Override
   protected void initializeSession()
   {
      hasSessionStarted = true;
      super.initializeSession();
      physicsEngine.initialize(gravity);
   }

   @Override
   public void shutdownSession()
   {
      super.shutdownSession();
      cleanupActions.forEach(Runnable::run);
      cleanupActions.clear();
      cameraBroadcastExecutor.shutdown();
   }

   @Override
   protected void doGeneric(SessionMode currentMode)
   {
      super.doGeneric(currentMode);

      cameraDefinitionNotifiers.forEach(notifier -> notifier.nextDefinition(null));
   }

   @Override
   protected double doSpecificRunTick()
   {
      double dt = Conversions.nanosecondsToSeconds(getSessionDTNanoseconds());
      for (int i = 0; i < beforePhysicsCallbacks.size(); i++)
         beforePhysicsCallbacks.get(i).accept(time.getValue());
      physicsEngine.simulate(time.getValue(), dt, gravity);
      double newTime = time.getValue() + dt;
      for (int i = 0; i < afterPhysicsCallbacks.size(); i++)
         afterPhysicsCallbacks.get(i).accept(newTime);
      return newTime;
   }

   @Override
   protected void schedulingSessionMode(SessionMode previousMode, SessionMode newMode)
   {
      if (previousMode == newMode)
         return;

      if (previousMode == SessionMode.RUNNING)
      {
         physicsEngine.pause();
         finalizeRunTick(true);
      }
   }

   public void addRobot(Robot robot)
   {
      checkSessionHasNotStarted();
      physicsEngine.addRobot(robot);
   }

   public Robot addRobot(RobotDefinition robotDefinition)
   {
      Robot robot = new Robot(robotDefinition, inertialFrame);
      configureCameraSensors(robot);
      addRobot(robot);
      return robot;
   }

   private void configureCameraSensors(Robot robot)
   {
      for (SimJointBasics joint : robot.getAllJoints())
      {
         for (SimCameraSensor cameraSensor : joint.getAuxialiryData().getCameraSensors())
            configureCameraSensor(robot.getName(), cameraSensor);
      }
   }

   private void configureCameraSensor(String robotName, SimCameraSensor cameraSensor)
   {
      robotNameToSensorNameToCameraMap.computeIfAbsent(robotName, s -> new HashMap<>()).put(cameraSensor.getName(), cameraSensor);

      CameraDefinitionConsumer cameraDefinitionNotifier = newDefinition ->
      {
         String sensorName = cameraSensor.getName();
         CameraSensorDefinition definitionData = newDefinition != null ? newDefinition : cameraSensor.toCameraSensorDefinition();
         SensorMessage<CameraSensorDefinition> newMessage = new SensorMessage<>(robotName, sensorName, definitionData);
         cameraDefinitionListeners.forEach(listener -> listener.accept(newMessage));
      };
      cameraDefinitionNotifiers.add(cameraDefinitionNotifier);
      cameraSensor.addCameraDefinitionConsumer(cameraDefinitionNotifier);
   }

   @Override
   public void setupWithMessager(Messager messager)
   {
      super.setupWithMessager(messager);

      cameraDefinitionListeners.add(message -> messager.submitMessage(SessionMessagerAPI.Sensors.CameraSensorDefinitionData, message));
      TopicListener<SensorMessage<BufferedImage>> listener = message ->
      {
         long timestamp = Conversions.secondsToNanoseconds(time.getValue());

         SimCameraSensor cameraSensor = robotNameToSensorNameToCameraMap.getOrDefault(message.getRobotName(), Collections.emptyMap())
                                                                        .get(message.getSensorName());

         cameraBroadcastExecutor.execute(() ->
         {
            for (CameraFrameConsumer consumer : cameraSensor.getCameraFrameConsumers())
            {
               consumer.nextFrame(timestamp, message.getMessageContent());
            }
         });
      };
      messager.registerTopicListener(SessionMessagerAPI.Sensors.CameraSensorFrame, listener);
      cleanupActions.add(() -> messager.removeTopicListener(SessionMessagerAPI.Sensors.CameraSensorFrame, listener));
   }

   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      checkSessionHasNotStarted();
      physicsEngine.addTerrainObject(terrainObjectDefinition);
   }

   public void addYoGraphicDefinition(YoGraphicDefinition yoGraphicDefinition)
   {
      checkSessionHasNotStarted();
      yoGraphicDefinitions.add(yoGraphicDefinition);
   }

   public void addYoGraphicDefinitions(YoGraphicDefinition... yoGraphicDefinitions)
   {
      for (YoGraphicDefinition yoGraphicDefinition : yoGraphicDefinitions)
      {
         addYoGraphicDefinition(yoGraphicDefinition);
      }
   }

   public void addYoGraphicDefinitions(Iterable<? extends YoGraphicDefinition> yoGraphicDefinitions)
   {
      for (YoGraphicDefinition yoGraphicDefinition : yoGraphicDefinitions)
      {
         addYoGraphicDefinition(yoGraphicDefinition);
      }
   }

   @Override
   public LinkedYoVariableFactory getLinkedYoVariableFactory()
   {
      return sharedBuffer;
   }

   @Override
   public String getSessionName()
   {
      return simulationName;
   }

   public PhysicsEngine getPhysicsEngine()
   {
      return physicsEngine;
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      return physicsEngine.getRobotDefinitions();
   }

   @Override
   public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return physicsEngine.getTerrainObjectDefinitions();
   }

   @Override
   public List<YoGraphicDefinition> getYoGraphicDefinitions()
   {
      return yoGraphicDefinitions;
   }

   @Override
   public List<RobotStateDefinition> getCurrentRobotStateDefinitions(boolean initialState)
   {
      if (initialState)
         return physicsEngine.getBeforePhysicsRobotStateDefinitions();
      else
         return physicsEngine.getCurrentRobotStateDefinitions();
   }

   private void checkSessionHasNotStarted()
   {
      if (hasSessionStarted)
         throw new IllegalOperationException("Illegal operation after session has started.");
   }

   public void addBeforePhysicsCallback(TimeConsumer beforePhysicsCallback)
   {
      beforePhysicsCallbacks.add(beforePhysicsCallback);
   }

   public boolean removeBeforePhysicsCallback(TimeConsumer beforePhysicsCallback)
   {
      return beforePhysicsCallbacks.remove(beforePhysicsCallback);
   }

   public void addAfterPhysicsCallback(TimeConsumer afterPhysicsCallback)
   {
      afterPhysicsCallbacks.add(afterPhysicsCallback);
   }

   public boolean removeAfterPhysicsCallback(TimeConsumer afterPhysicsCallback)
   {
      return afterPhysicsCallbacks.remove(afterPhysicsCallback);
   }

   private class SimulationSessionControlsImpl implements SimulationSessionControls
   {
      @Override
      public void pause()
      {
         setSessionMode(SessionMode.PAUSE);
      }

      @Override
      public void simulate()
      {
         setSessionMode(SessionMode.RUNNING);
      }

      @Override
      public void simulate(double duration)
      {
         if (getActiveMode() == SessionMode.RUNNING)
            setSessionMode(SessionMode.PAUSE);

         double startTime = time.getValue();
         BooleanSupplier terminalCondition = () -> time.getValue() - startTime >= duration;
         setSessionMode(SessionMode.RUNNING, SessionModeTransition.newTransition(terminalCondition, SessionMode.PAUSE));
      }

      @Override
      public void simulate(int numberOfTicks)
      {
         if (getActiveMode() == SessionMode.RUNNING)
            setSessionMode(SessionMode.PAUSE);

         BooleanSupplier terminalCondition = new BooleanSupplier()
         {
            private int tickCounter = 0;

            @Override
            public boolean getAsBoolean()
            {
               tickCounter++;
               return tickCounter >= numberOfTicks || testTerminalConditions();
            }
         };
         setSessionMode(SessionMode.RUNNING, SessionModeTransition.newTransition(terminalCondition, SessionMode.PAUSE));
      }

      @Override
      public boolean simulateAndWait(double duration)
      {
         long numberOfTicks = Conversions.secondsToNanoseconds(duration) / getSessionDTNanoseconds();
         return simulateAndWait(numberOfTicks);
      }

      @Override
      public boolean simulateAndWait(long numberOfTicks)
      {
         boolean sessionStartedInitialValue = hasSessionStarted();

         if (sessionStartedInitialValue)
            stopSessionThread();

         try
         {
            boolean success = true;

            for (long tick = 0; tick < numberOfTicks; tick++)
            {
               success = runTick();

               if (!success)
                  break;

               if (testTerminalConditions())
                  break;
            }

            return success;
         }
         finally
         {
            // This ensures that the controller is being pause.
            physicsEngine.pause();

            if (sessionStartedInitialValue)
               startSessionThread();
         }
      }

      private boolean testTerminalConditions()
      {
         for (int i = 0; i < terminalConditions.size(); i++)
         {
            if (terminalConditions.get(i).getAsBoolean())
               return true;
         }

         return false;
      }

      @Override
      public void addExternalTerminalCondition(BooleanSupplier... externalTerminalConditions)
      {
         for (int i = 0; i < externalTerminalConditions.length; i++)
         {
            terminalConditions.add(externalTerminalConditions[i]);
         }
      }

      @Override
      public boolean removeExternalTerminalCondition(BooleanSupplier externalTerminalCondition)
      {
         return terminalConditions.remove(externalTerminalCondition);
      }

      @Override
      public void clearExternalTerminalConditions()
      {
         terminalConditions.clear();
      }

      @Override
      public void addSimulationThrowableListener(Consumer<Throwable> listener)
      {
         addRunThrowableListener(listener);
      }

      // Buffer controls
      @Override
      public void setBufferInPointIndexToCurrent()
      {
         submitBufferInPointIndexRequestAndWait(sharedBuffer.getProperties().getCurrentIndex());
      }

      @Override
      public void setBufferOutPointIndexToCurrent()
      {
         submitBufferOutPointIndexRequestAndWait(sharedBuffer.getProperties().getCurrentIndex());
      }

      @Override
      public void setBufferCurrentIndexToInPoint()
      {
         submitBufferIndexRequestAndWait(sharedBuffer.getProperties().getInPoint());
      }

      @Override
      public void setBufferCurrentIndexToOutPoint()
      {
         submitBufferIndexRequestAndWait(sharedBuffer.getProperties().getOutPoint());
      }

      @Override
      public void stepBufferIndexBackward(int stepSize)
      {
         submitDecrementBufferIndexRequestAndWait(stepSize);
      }

      @Override
      public void stepBufferIndexForward(int stepSize)
      {
         submitIncrementBufferIndexRequestAndWait(stepSize);
      }
   }
}

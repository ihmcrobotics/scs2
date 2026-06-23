package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.YoTimer;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParametersReadOnly;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.YoMujocoSimulationParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * SCS2 {@link PhysicsEngine} backed by MuJoCo via a custom JavaCPP preset.
 *
 * <p>Behavioural contract differs slightly from the Bullet engine because MuJoCo can't add bodies
 * after model compile. We accept {@code addRobot} / {@code addTerrainObject} calls before the
 * first {@code simulate}, then on the first simulate we assemble one composite MJCF, compile it,
 * and register every joint's MuJoCo address. Subsequent {@code addRobot} calls throw.
 */
public class MujocoPhysicsEngine implements PhysicsEngine
{
   static
   {
      if (!MujocoNativeLibrary.load())
         throw new RuntimeException("Failed to load MuJoCo native libraries.");
   }

   private final ReferenceFrame inertialFrame;
   private final YoRegistry rootRegistry;
   private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());

   private final List<Robot> pendingRobots = new ArrayList<>();
   private final List<TerrainObjectDefinition> pendingTerrain = new ArrayList<>();

   private final List<MujocoRobot> robotList = new ArrayList<>();
   private final List<MujocoTerrainObject> terrainObjectList = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();

   private final MujocoMultiBodyDynamicsWorld dynamicsWorld = new MujocoMultiBodyDynamicsWorld();
   private final YoMujocoSimulationParameters globalSimulationParameters;
   private final YoBoolean hasBeenCompiled;

   private static final int REALTIME_RATE_SAMPLES = 100;
   private double simulateDt = 0.0;
   private long simulateCallStartTime = 0;
   private long realtimeRateWindowStartTime = 0;
   private long realtimeRateSampleCounter = 0;

   private final YoDouble mujocoRealtimeRate = new YoDouble("MujocoRealtimeRate", physicsEngineRegistry);
   private final YoDouble mujocoSimulateTimeMs = new YoDouble("MujocoSimulateTime[ms]", physicsEngineRegistry);
   private final YoLong mujocoTick = new YoLong("MujocoTick", physicsEngineRegistry);
   private final YoTimer stepTimer = new YoTimer("mujocoStep", TimeUnit.MILLISECONDS, physicsEngineRegistry);
   private final YoInteger nContacts = new YoInteger("mujocoNContacts", physicsEngineRegistry);

   private File workingDirectory;
   private boolean hasBeenInitialized = false;

   public MujocoPhysicsEngine(ReferenceFrame inertialFrame, YoRegistry rootRegistry, MujocoSimulationParametersReadOnly parameters)
   {
      this.inertialFrame = inertialFrame;
      this.rootRegistry = rootRegistry;
      this.globalSimulationParameters = new YoMujocoSimulationParameters("globalMujoco", physicsEngineRegistry);
      this.hasBeenCompiled = new YoBoolean("mujocoHasBeenCompiled", physicsEngineRegistry);
      globalSimulationParameters.set(parameters);
   }

   @Override
   public void initialize(Vector3DReadOnly gravity)
   {
      compileIfNeeded();
      dynamicsWorld.setGravity(gravity);

      for (MujocoRobot robot : robotList)
      {
         robot.initializeState();
         robot.updateSensors(dynamicsWorld.getData().cfrc_ext());
         robot.getControllerManager().initializeControllers();
      }
      hasBeenInitialized = true;
   }

   @Override
   public void simulate(double currentTime, double dt, Vector3DReadOnly gravity)
   {
      if (!hasBeenInitialized)
      {
         initialize(gravity);
         return;
      }

      simulateDt = dt;
      long now = System.nanoTime();
      if (simulateCallStartTime != 0)
         mujocoSimulateTimeMs.set((now - simulateCallStartTime) * 1.0e-6);
      simulateCallStartTime = now;
      if (realtimeRateSampleCounter == 0)
      {
         if (realtimeRateWindowStartTime != 0)
         {
            double elapsed = (now - realtimeRateWindowStartTime) * 1.0e-9;
            mujocoRealtimeRate.set(REALTIME_RATE_SAMPLES * simulateDt / elapsed);
         }
         realtimeRateWindowStartTime = now;
         realtimeRateSampleCounter = REALTIME_RATE_SAMPLES;
      }
      realtimeRateSampleCounter--;
      mujocoTick.increment();

      dynamicsWorld.setGravity(gravity);

      for (MujocoRobot robot : robotList)
      {
         // updateSensors reads MuJoCo's cfrc_ext from the previous step. Controllers therefore
         // see one-tick-stale contact wrenches; same discrete-time convention as Bullet /
         // ContactPointBased. On the very first tick cfrc_ext is zero (mj_makeData default).
         // Frames are already current from pullStateFromMujoco() at the end of the previous step.
         robot.updateSensors(dynamicsWorld.getData().cfrc_ext());
         robot.getControllerManager().updateControllers(currentTime);
         robot.getControllerManager().writeControllerOutput(JointStateType.EFFORT);
         robot.getControllerManager().writeControllerOutputForJointsToIgnore(JointStateType.values());
         robot.saveRobotBeforePhysicsState();
      }

      // qfrc_applied is zeroed every step by MuJoCo, so we must push the controller torques on
      // every call before stepping.
      for (MujocoRobot robot : robotList)
      {
         robot.pushStateToMujoco(dynamicsWorld.getData().qfrc_applied(),
                                 dynamicsWorld.getData().qpos(),
                                 dynamicsWorld.getData().qvel());
         // External wrench points (e.g. from PushRobotController-style controllers) route to
         // MuJoCo's per-body xfrc_applied. Unlike qfrc_applied, xfrc_applied is NOT zeroed by
         // mj_step, so the push helper rezeros the slots it manages on each tick.
         robot.pushExternalWrenchesToMujoco(dynamicsWorld.getData().xfrc_applied());
      }

      stepTimer.start();
      dynamicsWorld.step(globalSimulationParameters.getSubSteps());
      stepTimer.stop();

      nContacts.set(dynamicsWorld.getData().ncon());

      for (MujocoRobot robot : robotList)
         robot.pullStateFromMujoco(gravity,
                                   dynamicsWorld.getData().qpos(),
                                   dynamicsWorld.getData().qvel(),
                                   dynamicsWorld.getData().qacc(),
                                   dynamicsWorld.getData().cacc());
   }

   private void compileIfNeeded()
   {
      if (hasBeenCompiled.getValue())
         return;

      try
      {
         workingDirectory = Files.createTempDirectory("scs2-mujoco-").toFile();
      }
      catch (java.io.IOException e)
      {
         throw new RuntimeException("Could not create MuJoCo working directory", e);
      }

      String mjcf = MujocoMultiBodyRobotFactory.buildWorldMjcf(pendingRobots,
                                                               pendingTerrain,
                                                               workingDirectory,
                                                               globalSimulationParameters);
      dynamicsWorld.compile(mjcf, new File(workingDirectory, "world.xml"));

      for (Robot robot : pendingRobots)
      {
         MujocoMultiBodyRobot mujocoMultiBodyRobot = MujocoMultiBodyRobotFactory.registerJoints(robot.getRobotDefinition(),
                                                                                                 dynamicsWorld.getModel());
         dynamicsWorld.addMujocoRobot(mujocoMultiBodyRobot);
         // Seed non-root qpos/qvel from RobotDefinition. HumanoidRobotInitialSetup writes the
         // half-squat pose onto each OneDoFJointDefinition's initial state; without this push the
         // robot spawns at q=0 (legs locked straight) and falls before the first controller tick.
         MujocoMultiBodyRobotFactory.seedInitialJointState(robot.getRobotDefinition(), mujocoMultiBodyRobot, dynamicsWorld.getData());

         MujocoRobot mujocoRobot = new MujocoRobot(robot, physicsEngineRegistry, mujocoMultiBodyRobot);
         // Robot.getRegistry() was already attached to rootRegistry in addRobot. Just attach the
         // physics-engine-specific secondary registry here.
         physicsEngineRegistry.addChild(mujocoRobot.getSecondaryRegistry());
         robotList.add(mujocoRobot);
      }
      for (TerrainObjectDefinition terrain : pendingTerrain)
      {
         MujocoTerrainObject terrainObject = new MujocoTerrainObject(terrain);
         dynamicsWorld.addMujocoTerrainObject(terrainObject);
         terrainObjectList.add(terrainObject);
         // terrainObjectDefinitions was populated in addTerrainObject; don't duplicate.
      }
      pendingRobots.clear();
      pendingTerrain.clear();

      hasBeenCompiled.set(true);
   }

   @Override
   public void pause()
   {
      for (MujocoRobot robot : robotList)
      {
         robot.updateFrames();
         robot.updateSensors(dynamicsWorld.getData().cfrc_ext());
         robot.getControllerManager().pauseControllers();
      }
   }

   @Override
   public void addRobot(Robot robot)
   {
      if (hasBeenCompiled.getValue())
         throw new IllegalStateException("MuJoCo model is already compiled; can't add robot '"
                                         + robot.getRobotDefinition().getName() + "' after simulate() has been called.");
      inertialFrame.checkReferenceFrameMatch(robot.getInertialFrame());
      pendingRobots.add(robot);
      // Eagerly attach the Robot's YoRegistry to root so the SCS2 visualizer wires its variable
      // tree at session-setup time -- which happens before initialize() runs. Without this the
      // sphere's joint variables don't show up and the visualizer has nothing to render.
      rootRegistry.addChild(robot.getRegistry());
   }

   @Override
   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      if (hasBeenCompiled.getValue())
         throw new IllegalStateException("MuJoCo model is already compiled; can't add terrain after simulate().");
      pendingTerrain.add(terrainObjectDefinition);
      // Mirror the eager exposure pattern for terrain so getTerrainObjectDefinitions() returns
      // it during session setup.
      terrainObjectDefinitions.add(terrainObjectDefinition);
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   @Override
   public List<? extends Robot> getRobots()
   {
      // Before compile, fall back to the pending list so SCS2's session setup (which queries
      // this before initialize() runs) sees the robots the user added.
      if (!hasBeenCompiled.getValue())
         return new ArrayList<>(pendingRobots);
      return robotList.stream().map(MujocoRobot::getRobot).collect(Collectors.toList());
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      if (!hasBeenCompiled.getValue())
         return pendingRobots.stream().map(Robot::getRobotDefinition).collect(Collectors.toList());
      return robotList.stream().map(RobotInterface::getRobotDefinition).collect(Collectors.toList());
   }

   @Override
   public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return terrainObjectDefinitions;
   }

   @Override
   public List<RobotStateDefinition> getBeforePhysicsRobotStateDefinitions()
   {
      return robotList.stream().map(RobotExtension::getRobotBeforePhysicsStateDefinition).collect(Collectors.toList());
   }

   @Override
   public YoRegistry getPhysicsEngineRegistry()
   {
      return physicsEngineRegistry;
   }

   @Override
   public void dispose()
   {
      robotList.clear();
      terrainObjectList.clear();
      pendingRobots.clear();
      pendingTerrain.clear();
      dynamicsWorld.dispose();
   }
}

package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParametersReadOnly;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.YoMujocoOptions;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.yoVariables.registry.YoRegistry;

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
   // Compile-time seeds, consumed once at MJCF generation; the MujocoOptions registry is the only
   // Yo mirror of runtime-tunable values.
   private final MujocoSimulationParameters seedParameters = new MujocoSimulationParameters();
   private final YoMujocoOptions options;
   private final MujocoStatistics statistics = new MujocoStatistics(physicsEngineRegistry);
   private final YoMujocoContactPool contactPool;
   // Plain boolean, deliberately not a YoVariable: this is engine control flow, and a GUI edit or
   // buffer scrub must not be able to flip it. The display mirror is statistics.hasBeenCompiled.
   private boolean modelCompiled = false;
   // subSteps value in effect this tick; refreshed only through pollSubStepsRequest so buffer
   // scrubs cannot change it and the timestep/step-count reads cannot desync mid-tick.
   private int appliedSubSteps;

   private static final int REALTIME_RATE_SAMPLES = 100;
   private double simulateDt = 0.0;
   private long simulateCallStartTime = 0;
   private long realtimeRateWindowStartTime = 0;
   private long realtimeRateSampleCounter = 0;


   private File workingDirectory;
   private boolean hasBeenInitialized = false;

   public MujocoPhysicsEngine(ReferenceFrame inertialFrame, YoRegistry rootRegistry, MujocoSimulationParametersReadOnly parameters)
   {
      this.inertialFrame = inertialFrame;
      this.rootRegistry = rootRegistry;
      seedParameters.set(parameters);
      this.options = new YoMujocoOptions(physicsEngineRegistry);
      options.subSteps.set(parameters.getSubSteps());
      appliedSubSteps = Math.max(1, parameters.getSubSteps());
      // Slot capacity must be fixed here: the YoVariables have to exist before the session buffer
      // is wired, well before the model compiles on the first simulate().
      int perContactCapacity = Math.max(0, parameters.getPerContactDiagnosticsCapacity());
      this.contactPool = perContactCapacity > 0 ? new YoMujocoContactPool(perContactCapacity, inertialFrame, physicsEngineRegistry) : null;
   }

   /** The live {@code mjOption} mirror; edits are pushed into the native model on the next {@code simulate()} tick. */
   public YoMujocoOptions getOptions()
   {
      return options;
   }

   /** The native world wrapper; the model/data are null until the first {@code simulate()} compiles the world. */
   public MujocoMultiBodyDynamicsWorld getDynamicsWorld()
   {
      return dynamicsWorld;
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
      // Edits (from any thread) only trip a dirty flag; the native write happens here on the
      // physics thread, cleared-before-apply so a concurrent edit lands next tick.
      if (options.pollUpdateRequest())
         dynamicsWorld.writeOptions(options);
      if (options.pollSubStepsRequest())
         appliedSubSteps = Math.max(1, options.subSteps.getValue());
      dynamicsWorld.setTimestep(dt / appliedSubSteps);
      long now = System.nanoTime();
      if (simulateCallStartTime != 0)
         statistics.mujocoSimulateTimeMs.set((now - simulateCallStartTime) * 1.0e-6);
      simulateCallStartTime = now;
      if (realtimeRateSampleCounter == 0)
      {
         if (realtimeRateWindowStartTime != 0)
         {
            double elapsed = (now - realtimeRateWindowStartTime) * 1.0e-9;
            statistics.mujocoRealtimeRate.set(REALTIME_RATE_SAMPLES * simulateDt / elapsed);
         }
         realtimeRateWindowStartTime = now;
         realtimeRateSampleCounter = REALTIME_RATE_SAMPLES;
      }
      realtimeRateSampleCounter--;
      statistics.mujocoTick.increment();

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

      statistics.stepTimer.start();
      dynamicsWorld.step(appliedSubSteps);
      statistics.stepTimer.stop();

      statistics.update();
      statistics.updateEnergy(options.enableEnergy.getValue());
      if (contactPool != null)
         contactPool.update();

      for (MujocoRobot robot : robotList)
         robot.pullStateFromMujoco(gravity,
                                   dynamicsWorld.getData().qpos(),
                                   dynamicsWorld.getData().qvel(),
                                   dynamicsWorld.getData().qacc(),
                                   dynamicsWorld.getData().cacc());
   }

   private void compileIfNeeded()
   {
      if (modelCompiled)
         return;

      String workingDirectoryProperty = System.getProperty("scs2.mujoco.workingDirectory");
      if (workingDirectoryProperty != null)
      {
         workingDirectory = new File(workingDirectoryProperty);
      }
      else
      {
         try
         {
            workingDirectory = Files.createTempDirectory("scs2-mujoco-").toFile();
         }
         catch (java.io.IOException e)
         {
            throw new RuntimeException("Could not create MuJoCo working directory", e);
         }
      }

      String mjcf = MujocoMultiBodyRobotFactory.buildWorldMjcf(pendingRobots,
                                                               pendingTerrain,
                                                               workingDirectory,
                                                               seedParameters);
      File mjcfFile = new File(workingDirectory, "world.xml");
      dynamicsWorld.compile(mjcf, mjcfFile);
      LogTools.info("MuJoCo world MJCF written to {}", mjcfFile.getAbsolutePath());

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

         if (contactPool != null)
            mujocoRobot.createContactAggregates(inertialFrame).forEach(contactPool::addBodyAggregate);
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

      // The MJCF emits no mjOption values, so the options group is the sole truth: push it into the
      // compiled model (this also applies values set before the first tick). The o_* override
      // entries first get the <default> geom block's values so flipping enableOverride on is
      // behavior-continuous; with the flag off they have no effect on the dynamics.
      options.o_margin.set(seedParameters.getContactMargin());
      options.o_solrefTimeconst.set(seedParameters.getContactSolrefTimeconst());
      options.o_solrefDampratio.set(seedParameters.getContactSolrefDampRatio());
      options.o_solimpDmin.set(seedParameters.getContactSolimpDmin());
      options.o_solimpDmax.set(seedParameters.getContactSolimpDmax());
      options.o_solimpWidth.set(seedParameters.getContactSolimpWidth());
      options.o_solimpMidpoint.set(seedParameters.getContactSolimpMidpoint());
      options.o_solimpPower.set(seedParameters.getContactSolimpPower());
      options.o_frictionSlide.set(seedParameters.getFrictionSlide());
      options.o_frictionSpin.set(seedParameters.getFrictionTorsional());
      options.o_frictionRoll.set(seedParameters.getFrictionRolling());
      dynamicsWorld.writeOptions(options);
      options.pollUpdateRequest(); // Discard the dirty flag the seeding just tripped.

      MujocoTools.logEffectiveModelSummary(dynamicsWorld.getModel());

      statistics.bind(dynamicsWorld.getModel(), dynamicsWorld.getData());
      if (contactPool != null)
         contactPool.bind(dynamicsWorld.getModel(), dynamicsWorld.getData());

      modelCompiled = true;
      statistics.hasBeenCompiled.set(true);
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
      if (modelCompiled)
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
      if (modelCompiled)
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
      if (!modelCompiled)
         return new ArrayList<>(pendingRobots);
      return robotList.stream().map(MujocoRobot::getRobot).collect(Collectors.toList());
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      if (!modelCompiled)
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

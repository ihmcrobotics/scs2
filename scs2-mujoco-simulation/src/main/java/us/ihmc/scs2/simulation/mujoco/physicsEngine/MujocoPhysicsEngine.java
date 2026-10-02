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

import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
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

   /** Per-tick solver, contact and warning counters, mirrored as YoVariables. */
   public MujocoStatistics getStatistics()
   {
      return statistics;
   }

   /** The native world wrapper; the model/data are null until the first {@code simulate()} compiles the world. */
   public MujocoMultiBodyDynamicsWorld getDynamicsWorld()
   {
      return dynamicsWorld;
   }

   /**
    * The low-level command block for one joint: feedforward torque, setpoints and gains.
    *
    * <p>Returns {@code null} before the world is compiled, and for joints with no actuator (the free
    * root, welded subtrees, and joint types the MJCF builder cannot map -- a caller that gets
    * {@code null} should fall back to writing the joint's effort as it does today). With more than
    * one robot in the world the first match wins, so prefer
    * {@link MujocoRobot#getJointActuation(String)} off the robot you mean.
    */
   public MujocoJointActuation getJointActuation(String jointName)
   {
      for (int i = 0; i < robotList.size(); i++)
      {
         MujocoJointActuation actuation = robotList.get(i).getJointActuation(jointName);
         if (actuation != null)
            return actuation;
      }
      return null;
   }

   /** The per-robot wrappers, available once the world has compiled. */
   public List<MujocoRobot> getMujocoRobots()
   {
      return robotList;
   }

   @Override
   public void initialize(Vector3DReadOnly gravity)
   {
      boolean wasAlreadyCompiled = modelCompiled;
      compileIfNeeded();
      // Re-initializing an already compiled world (SimulationSession.resetToInitialState) used to
      // reset only the SCS2 side while mjData carried on from wherever the simulation had got to.
      // MuJoCo owns velocities, warm-start and contact state, so it has to be reset too.
      if (wasAlreadyCompiled)
         resetNativeState();
      dynamicsWorld.setGravity(gravity);

      for (MujocoRobot robot : robotList)
      {
         robot.initializeState();
         robot.updateSensors(dynamicsWorld.getData().cfrc_ext(), dynamicsWorld.getData().xpos(), dynamicsWorld.getData().subtree_com());
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
         statistics.simulateTime.set((now - simulateCallStartTime) * 1.0e-6);
      simulateCallStartTime = now;
      if (realtimeRateSampleCounter == 0)
      {
         if (realtimeRateWindowStartTime != 0)
         {
            double elapsed = (now - realtimeRateWindowStartTime) * 1.0e-9;
            statistics.realtimeRate.set(REALTIME_RATE_SAMPLES * simulateDt / elapsed);
         }
         realtimeRateWindowStartTime = now;
         realtimeRateSampleCounter = REALTIME_RATE_SAMPLES;
      }
      realtimeRateSampleCounter--;
      statistics.tick.increment();

      dynamicsWorld.setGravity(gravity);

      for (MujocoRobot robot : robotList)
      {
         // updateSensors reads MuJoCo's cfrc_ext from the previous step. Controllers therefore
         // see one-tick-stale contact wrenches; same discrete-time convention as Bullet /
         // ContactPointBased. On the very first tick cfrc_ext is zero (mj_makeData default).
         // Frames are already current from pullStateFromMujoco() at the end of the previous step.
         robot.updateSensors(dynamicsWorld.getData().cfrc_ext(), dynamicsWorld.getData().xpos(), dynamicsWorld.getData().subtree_com());
         robot.getControllerManager().updateControllers(currentTime);
         robot.getControllerManager().writeControllerOutput(JointStateType.EFFORT);
         robot.getControllerManager().writeControllerOutputForJointsToIgnore(JointStateType.values());
         robot.saveRobotBeforePhysicsState();
      }

      // Push the SCS2 joint state and the controller torques on every call before stepping, so edits made to the
      // SCS2 joints since the last step (tests, GUI, buffer rewind) take effect.
      for (MujocoRobot robot : robotList)
      {
         // Joints flagged through SimJointBasics.setPinned are held by an equality constraint, so
         // the solver resolves them together with contact instead of them being frozen afterwards.
         // This runs first because it detects SCS2-side edits by comparing against MuJoCo's current
         // qpos, which pushStateToMujoco is about to overwrite.
         robot.pushPinnedJointsToMujoco(dynamicsWorld.getModel(), dynamicsWorld.getData());
         robot.pushStateToMujoco(dynamicsWorld.getData().qpos(), dynamicsWorld.getData().qvel());
         // Under JOINT_SERVO the setpoints and gains go to MuJoCo's actuators instead of a
         // finished torque, so the low-level loop closes on every physics step rather than being
         // held constant between controller ticks.
         robot.pushActuationToMujoco(dynamicsWorld.getModel(), dynamicsWorld.getData());
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
      statistics.updateSolverDiagnostics(options.enableFwdinv.getValue());
      if (contactPool != null)
         contactPool.update();

      for (MujocoRobot robot : robotList)
      {
         robot.pullStateFromMujoco(gravity,
                                   dynamicsWorld.getData().qpos(),
                                   dynamicsWorld.getData().qvel(),
                                   dynamicsWorld.getData().qacc(),
                                   dynamicsWorld.getData().cacc(),
                                   dynamicsWorld.getData().xpos(),
                                   dynamicsWorld.getData().subtree_com());
         robot.pullActuationFromMujoco(dynamicsWorld.getData());
      }
   }

   private void resetNativeState()
   {
      // The initial keyframe was filled from the seeded state at compile, so this restores the pose
      // the robots actually spawn in rather than qpos0, and clears velocities, warm-start
      // accelerations and contact state along with it.
      dynamicsWorld.resetToInitialKeyframe();
   }

   private void compileIfNeeded()
   {
      if (modelCompiled)
         return;

      // Only a path at this point: nothing is created here. A world of primitive collision shapes
      // never needs it, since the MJCF is compiled from memory and inline meshes carry their own
      // vertices; it only comes into existence if a file-backed mesh has to be staged.
      String workingDirectoryProperty = System.getProperty("scs2.mujoco.workingDirectory");
      boolean workingDirectoryWasRequested = workingDirectoryProperty != null;
      workingDirectory = workingDirectoryWasRequested ? new File(workingDirectoryProperty)
                                                      : new File(System.getProperty("java.io.tmpdir"), "scs2-mujoco-" + System.nanoTime());

      String mjcf = MujocoMultiBodyRobotFactory.buildWorldMjcf(pendingRobots,
                                                               pendingTerrain,
                                                               workingDirectory,
                                                               seedParameters);
      // Written out only when a directory was asked for; otherwise the compile reads it from memory
      // and nothing is left behind.
      File mjcfFile = workingDirectoryWasRequested ? new File(workingDirectory, "world.xml") : null;
      dynamicsWorld.compile(mjcf, mjcfFile);
      if (mjcfFile != null)
         LogTools.info("MuJoCo world MJCF written to {}", mjcfFile.getAbsolutePath());
      else
         LogTools.info("MuJoCo world compiled from memory; set -Dscs2.mujoco.workingDirectory to write the MJCF out");

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
      // Every robot has now been seeded, so the world's initial state is complete: store it in the
      // keyframe the MJCF declared, which is what resetNativeState restores.
      dynamicsWorld.captureInitialKeyframe();

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
      options.o_margin.set(seedParameters.get_margin());
      options.o_solref_timeconst.set(seedParameters.get_solref_timeconst());
      options.o_solref_dampratio.set(seedParameters.get_solref_dampratio());
      options.o_solimp_dmin.set(seedParameters.get_solimp_dmin());
      options.o_solimp_dmax.set(seedParameters.get_solimp_dmax());
      options.o_solimp_width.set(seedParameters.get_solimp_width());
      options.o_solimp_midpoint.set(seedParameters.get_solimp_midpoint());
      options.o_solimp_power.set(seedParameters.get_solimp_power());
      options.o_friction_slide.set(seedParameters.get_friction_slide());
      options.o_friction_spin.set(seedParameters.get_friction_spin());
      options.o_friction_roll.set(seedParameters.get_friction_roll());
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
         robot.updateSensors(dynamicsWorld.getData().cfrc_ext(), dynamicsWorld.getData().xpos(), dynamicsWorld.getData().subtree_com());
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

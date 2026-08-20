package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.YoTimer;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletContactSolverInfoParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletSimulationParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletContactSolverInfoParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletSimulationParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class BulletPhysicsEngine implements PhysicsEngine
{
   private final BulletMultiBodyDynamicsWorld bulletMultiBodyDynamicsWorld;
   private final ReferenceFrame inertialFrame;
   private final List<BulletRobot> robotList = new ArrayList<>();
   private final List<BulletTerrainObject> terrainObjectList = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();

   private final YoRegistry rootRegistry;
   private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());
   private final YoRegistry physicsEngineStatisticsRegistry = new YoRegistry("physicsEngineStatistics");
   private final YoDouble runTickExpectedTimeRate = new YoDouble("runTickExpectedTimeRate", physicsEngineStatisticsRegistry);
   private final YoTimer runBulletPhysicsEngineSimulateTimer = new YoTimer("runBulletPhysicsEngineSimulateTimer",
                                                                           TimeUnit.MILLISECONDS,
                                                                           physicsEngineStatisticsRegistry);
   private final YoTimer runBulletStepSimulateTimer = new YoTimer("runBulletStepSimulateTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoTimer runControllerManagerTimer = new YoTimer("runControllerManagerTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoTimer runPullStateFromBullet = new YoTimer("runPullStateFromBulletTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoTimer runPushStateToBulletTimer = new YoTimer("runPushStateToBullet", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoTimer runUpdateFramesSensorsTimer = new YoTimer("runUpdateFramesSensorsTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoDouble bulletPhysicsEngineRealTimeRate = new YoDouble("bulletPhysicsEngineRealTimeRate", physicsEngineStatisticsRegistry);
   private final ArrayList<YoPoint3D> contactPoints = new ArrayList<>();

   {
      for (int i = 0; i < 100; i++)
      {
         contactPoints.add(new YoPoint3D("contactPoint" + i, physicsEngineRegistry));
      }
   }

   private final YoBulletMultiBodyParameters globalMultiBodyParameters;
   private final YoBulletMultiBodyJointParameters globalMultiBodyJointParameters;
   private final YoBulletContactSolverInfoParameters globalContactSolverInfoParameters;
   private final YoBoolean hasGlobalBulletSimulationParameters;
   private final YoBulletSimulationParameters globalBulletSimulationParameters;
   private boolean hasBeenInitialized = false;

   public BulletPhysicsEngine(ReferenceFrame inertialFrame, YoRegistry rootRegistry)
   {
      this.inertialFrame = inertialFrame;
      this.rootRegistry = rootRegistry;

      physicsEngineRegistry.addChild(physicsEngineStatisticsRegistry);

      globalMultiBodyParameters = new YoBulletMultiBodyParameters("globalMultiBody", physicsEngineRegistry);
      globalMultiBodyJointParameters = new YoBulletMultiBodyJointParameters("globalMultiBodyJoint", physicsEngineRegistry);
      globalContactSolverInfoParameters = new YoBulletContactSolverInfoParameters("globalContactSolverInfo", physicsEngineRegistry);
      hasGlobalBulletSimulationParameters = new YoBoolean("hasGlobalSimulationParameters", physicsEngineRegistry);
      globalBulletSimulationParameters = new YoBulletSimulationParameters("globalSimulation", physicsEngineRegistry);
      setGlobalBulletMultiBodyParameters(BulletMultiBodyParameters.defaultBulletMultiBodyParameters());
      setGlobalBulletMultiBodyJointParameters(BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters());
      setGlobalContactSolverInfoParameters(BulletContactSolverInfoParameters.defaultBulletContactSolverInfoParameters());

      hasGlobalBulletSimulationParameters.set(false);

      bulletMultiBodyDynamicsWorld = new BulletMultiBodyDynamicsWorld();
   }

   @Override
   public void initialize(Vector3DReadOnly gravity)
   {
      for (BulletRobot robot : robotList)
      {
         robot.initializeState();
         robot.updateSensors();
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

      //set yoVariable Tick Expected Time Rate in milliseconds
      runTickExpectedTimeRate.set(dt * 1000.0);

      runBulletPhysicsEngineSimulateTimer.start();

      if (globalMultiBodyParameters.getUpdateGlobalMultiBodyParameters())
      {
         globalMultiBodyParameters.setUpdateGlobalMultiBodyParameters(false);
         bulletMultiBodyDynamicsWorld.updateAllMultiBodyParameters(globalMultiBodyParameters);
      }
      if (globalMultiBodyJointParameters.getUpdateGlobalMultiBodyJointParameters())
      {
         globalMultiBodyJointParameters.setUpdateGlobalMultiBodyJointParameters(false);
         bulletMultiBodyDynamicsWorld.updateAllMultiBodyJointParameters(globalMultiBodyJointParameters);
      }
      if (globalContactSolverInfoParameters.getUpdateGlobalContactSolverInfoParameters())
      {
         globalContactSolverInfoParameters.setUpdateGlobalContactSolverInfoParameters(false);
         bulletMultiBodyDynamicsWorld.updateContactSolverInfoParameters(globalContactSolverInfoParameters);
      }

      runControllerManagerTimer.start();
      for (BulletRobot robot : robotList)
      {
         robot.updateFrames();
         robot.updateSensors();
         robot.getControllerManager().updateControllers(currentTime);
         robot.getControllerManager().writeControllerOutput(JointStateType.EFFORT);
         robot.getControllerManager().writeControllerOutputForJointsToIgnore(JointStateType.values());
         robot.saveRobotBeforePhysicsState();
      }
      runControllerManagerTimer.stop();

      runPushStateToBulletTimer.start();
      for (BulletRobot robot : robotList)
      {
         robot.pushStateToBullet();
      }
      runPushStateToBulletTimer.stop();

      runBulletStepSimulateTimer.start();

      bulletMultiBodyDynamicsWorld.setGravity(gravity);

      if (hasGlobalBulletSimulationParameters.getValue())
         bulletMultiBodyDynamicsWorld.stepSimulation(globalBulletSimulationParameters.getTimeStamp(),
                                                     globalBulletSimulationParameters.getMaxSubSteps(),
                                                     globalBulletSimulationParameters.getFixedTimeStep());
      else
         bulletMultiBodyDynamicsWorld.stepSimulation(dt, 1, dt);

      runBulletStepSimulateTimer.stop();

      runPullStateFromBullet.start();
      for (BulletRobot robot : robotList)
      {
         robot.pullStateFromBullet(dt);
      }
      for (BulletTerrainObject terrainObject : terrainObjectList)
      {
         terrainObject.pullStateFromBullet();
      }

      runPullStateFromBullet.stop();

      runUpdateFramesSensorsTimer.start();
      runUpdateFramesSensorsTimer.stop();
      runBulletPhysicsEngineSimulateTimer.stop();
      bulletPhysicsEngineRealTimeRate.set(runTickExpectedTimeRate.getValue() / runBulletPhysicsEngineSimulateTimer.getTimer().getValue());
   }

   @Override
   public void pause()
   {
      for (BulletRobot robot : robotList)
      {
         robot.updateFrames();
         robot.updateSensors();
         robot.getControllerManager().pauseControllers();
      }
   }

   @Override
   public void addRobot(Robot robot)
   {
      inertialFrame.checkReferenceFrameMatch(robot.getInertialFrame());

      BulletMultiBodyRobot bulletMultiBodyRobot = BulletMultiBodyRobotFactory.newInstance(robot, globalMultiBodyParameters, globalMultiBodyJointParameters);
      bulletMultiBodyDynamicsWorld.addBulletMultiBodyRobot(bulletMultiBodyRobot);

      BulletRobot bulletRobot = new BulletRobot(robot, physicsEngineRegistry, bulletMultiBodyRobot);
      rootRegistry.addChild(bulletRobot.getRegistry());
      physicsEngineRegistry.addChild(bulletRobot.getSecondaryRegistry());
      robotList.add(bulletRobot);
   }

   @Override
   public void dispose()
   {
      robotList.clear(); // Clear references so they can be deallocated
      terrainObjectList.clear(); // Clear references so they can be deallocated
      bulletMultiBodyDynamicsWorld.dispose();
   }

   @Override
   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      addAndGetTerrainObject(terrainObjectDefinition);
   }

   public BulletTerrainObject addAndGetTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      BulletTerrainObject bulletTerrainObject = BulletTerrainFactory.newInstance(terrainObjectDefinition);
      terrainObjectList.add(bulletTerrainObject);
      terrainObjectDefinitions.add(terrainObjectDefinition);
      bulletMultiBodyDynamicsWorld.addBulletTerrainObject(bulletTerrainObject);
      return bulletTerrainObject;
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   @Override
   public List<? extends Robot> getRobots()
   {
      return robotList.stream().map(BulletRobot::getRobot).collect(Collectors.toList());
   }

   public List<BulletRobot> getBulletRobots()
   {
      return robotList;
   }

   public List<BulletTerrainObject> getTerrainObjects()
   {
      return terrainObjectList;
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
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

   public void setGlobalBulletMultiBodyParameters(BulletMultiBodyParameters bulletMultiBodyParameters)
   {
      globalMultiBodyParameters.set(bulletMultiBodyParameters);
   }

   public void setGlobalBulletMultiBodyJointParameters(BulletMultiBodyJointParameters bulletMultiBodyJointParameters)
   {
      globalMultiBodyJointParameters.set(bulletMultiBodyJointParameters);
   }

   public YoBulletMultiBodyParameters getGlobalBulletMultiBodyParameters()
   {
      return globalMultiBodyParameters;
   }

   public YoBulletMultiBodyJointParameters getGlobalBulletMultiBodyJointParameters()
   {
      return globalMultiBodyJointParameters;
   }

   public void setGlobalSimulationParameters(BulletSimulationParameters bulletSimulationParameters)
   {
      globalBulletSimulationParameters.set(bulletSimulationParameters);
      hasGlobalBulletSimulationParameters.set(true);
   }

   public YoBulletSimulationParameters getGlobalSimulationParameters()
   {
      return globalBulletSimulationParameters;
   }

   public BulletMultiBodyDynamicsWorld getBulletMultiBodyDynamicsWorld()
   {
      return bulletMultiBodyDynamicsWorld;
   }

   public void setGlobalContactSolverInfoParameters(BulletContactSolverInfoParameters bulletContactSolverInfoParameters)
   {
      globalContactSolverInfoParameters.set(bulletContactSolverInfoParameters);
   }

   public YoBulletContactSolverInfoParameters getGlobalContactSolverInfoParameters()
   {
      return globalContactSolverInfoParameters;
   }
}

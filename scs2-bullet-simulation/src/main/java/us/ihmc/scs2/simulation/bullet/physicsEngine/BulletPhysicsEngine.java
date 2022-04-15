package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.YoTimer;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BulletPhysicsEngine implements PhysicsEngine {
	static {
		Bullet.init();
		LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());
	}
	private BulletMultiBodyDynamicsWorld multiBodyDynamicsWorld;
	private final ReferenceFrame inertialFrame;
	private final List<BulletRobot> robotList = new ArrayList<>();
	private final ArrayList<BulletTerrainObject> terrainObjects = new ArrayList<>();
	private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();

	private final YoRegistry rootRegistry;
	private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());
	private final YoDouble runTickExpectedTimeRate = new YoDouble("runTickExpectedTimeRate", physicsEngineRegistry);
	private final YoTimer runBulletPhysicsEngineSimulateTimer = new YoTimer("runBulletPhysicsEngineSimulateTimer",
			TimeUnit.MILLISECONDS, physicsEngineRegistry);
	private final YoTimer runBulletStepSimulateTimer = new YoTimer("runBulletStepSimulateTimer", TimeUnit.MILLISECONDS,
			physicsEngineRegistry);
	private final YoTimer runControllerManagerTimer = new YoTimer("runControllerManagerTimer", TimeUnit.MILLISECONDS,
			physicsEngineRegistry);
	private final YoTimer runCopyDataFromBulletToSCSTimer = new YoTimer("runCopyDataFromBulletToSCSTimer",
			TimeUnit.MILLISECONDS, physicsEngineRegistry);
	private final YoTimer runCopyDataFromSCSToBulletTimer = new YoTimer("runCopyDataFromSCSToBulletTimer",
			TimeUnit.MILLISECONDS, physicsEngineRegistry);
	private final YoTimer runUpdateFramesSensorsTimer = new YoTimer("runUpdateFramesSensorsTimer",
			TimeUnit.MILLISECONDS, physicsEngineRegistry);
	private final YoDouble bulletPhysicsEngineRealTimeRate = new YoDouble("bulletPhysicsEngineRealTimeRate",
			physicsEngineRegistry);
	private final ArrayList<YoPoint3D> contactPoints = new ArrayList<>();
	{
		for (int i = 0; i < 100; i++) {
			contactPoints.add(new YoPoint3D("contactPoint" + i, physicsEngineRegistry));
		}
	}
	private final YoBulletMultiBodyParameters globalMultiBodyParameters;
	private boolean initialize = true;

	public BulletPhysicsEngine(ReferenceFrame inertialFrame, YoRegistry rootRegistry) {
		this.inertialFrame = inertialFrame;
		this.rootRegistry = rootRegistry;
		
		globalMultiBodyParameters = new YoBulletMultiBodyParameters("globalMultiBody", rootRegistry);

		multiBodyDynamicsWorld = new BulletMultiBodyDynamicsWorld();
	}

	@Override
	public boolean initialize(Vector3DReadOnly gravity) {
		if (!initialize)
			return false;

		rootRegistry.addChild(physicsEngineRegistry);

		for (BulletRobot robot : robotList) {
			robot.initializeState();
			robot.updateSensors();
			robot.getControllerManager().initializeControllers();
		}
		initialize = false;
		return true;
	}

	@Override
	public void simulate(double currentTime, double dt, Vector3DReadOnly gravity) {

		//set yoVariable Tick Expected Time Rate in milliseconds
		runTickExpectedTimeRate.set(dt * 1000);

		if (initialize(gravity))
			return;

		runBulletPhysicsEngineSimulateTimer.start();

		runControllerManagerTimer.start();
		for (BulletRobot robot : robotList) {
			robot.getControllerManager().updateControllers(currentTime);
			robot.getControllerManager().writeControllerOutput(JointStateType.EFFORT);
			robot.getControllerManager().writeControllerOutputForJointsToIgnore(JointStateType.values());
		}
		runControllerManagerTimer.stop();

		runCopyDataFromSCSToBulletTimer.start();
		for (BulletRobot robot : robotList) {
			robot.saveRobotBeforePhysicsState();
		}
		runCopyDataFromSCSToBulletTimer.stop();

		runBulletStepSimulateTimer.start();
		multiBodyDynamicsWorld.stepSimulation(dt);
		runBulletStepSimulateTimer.stop();

		runCopyDataFromBulletToSCSTimer.start();
		for (BulletRobot robot : robotList) {
			robot.updateFromBulletData(this, dt);
		}
		runCopyDataFromBulletToSCSTimer.stop();

		runUpdateFramesSensorsTimer.start();
		for (BulletRobot robot : robotList) {
			robot.updateFrames();
			robot.updateSensors();
		}
		runUpdateFramesSensorsTimer.stop();
		runBulletPhysicsEngineSimulateTimer.stop();
		bulletPhysicsEngineRealTimeRate.set(runTickExpectedTimeRate.getValue() / runBulletPhysicsEngineSimulateTimer.getTimer().getValue());
	}

	@Override
	public void pause() {
		for (BulletRobot robot : robotList) {
			robot.getControllerManager().pauseControllers();
		}
	}

	@Override
	public void addRobot(Robot robot) {
		inertialFrame.checkReferenceFrameMatch(robot.getInertialFrame());
		
		BulletMultiBodyRobot bulletMultiBodyRobot = BulletMultiBodyRobotFactory.newInstance(robot, globalMultiBodyParameters);
		BulletRobot bulletRobot = new BulletRobot(robot, physicsEngineRegistry, bulletMultiBodyRobot);
		multiBodyDynamicsWorld.addMultiBody(bulletMultiBodyRobot);
		//multiBodyDynamicsWorld.getMultiBodyDynamicsWorld().addMultiBody(bulletRobot.getBulletMultiBody());
		rootRegistry.addChild(bulletRobot.getRegistry());
		robotList.add(bulletRobot);
	}

	@Override
	public void dispose() {
		multiBodyDynamicsWorld.dispose();
	}

	@Override
	public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition) {
		terrainObjectDefinitions.add(terrainObjectDefinition);
		terrainObjects.add(new BulletTerrainObject(terrainObjectDefinition, multiBodyDynamicsWorld.getMultiBodyDynamicsWorld()));
	}

	public void addMultiBodyCollisionShape(btMultiBodyLinkCollider collisionShape, int collisionGroup,
			int collisionGroupMask) {
		multiBodyDynamicsWorld.getMultiBodyDynamicsWorld().addCollisionObject(collisionShape, collisionGroup,
				collisionGroupMask);
	}

	@Override
	public ReferenceFrame getInertialFrame() {
		return inertialFrame;
	}

	@Override
	public List<? extends Robot> getRobots() {
		return robotList.stream().map(BulletRobot::getRobot).collect(Collectors.toList());
	}

	@Override
	public List<RobotDefinition> getRobotDefinitions() {
		return robotList.stream().map(RobotInterface::getRobotDefinition).collect(Collectors.toList());
	}

	@Override
	public List<TerrainObjectDefinition> getTerrainObjectDefinitions() {
		return terrainObjectDefinitions;
	}

	@Override
	public List<RobotStateDefinition> getBeforePhysicsRobotStateDefinitions() {
		return robotList.stream().map(RobotExtension::getRobotBeforePhysicsStateDefinition)
				.collect(Collectors.toList());
	}

	@Override
	public YoRegistry getPhysicsEngineRegistry() {
		return physicsEngineRegistry;
	}
	
	public void setGlobalMultiBodyParameter(BulletMultiBodyParameters parameters)
	{
	   globalMultiBodyParameters.set(parameters);
	}

	public btMultiBodyDynamicsWorld getBulletMultiBodyDynamicsWorld() {
		return multiBodyDynamicsWorld.getMultiBodyDynamicsWorld();
	}

}

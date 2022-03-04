package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.btBroadphaseInterface;
import com.badlogic.gdx.physics.bullet.collision.btCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.collision.btCollisionDispatcher;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btDbvtBroadphase;
import com.badlogic.gdx.physics.bullet.collision.btDefaultCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraintSolver;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletBasedPhysicsEngine implements PhysicsEngine
{
   static
   {
      Bullet.init();
      LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());
   }
   private btCollisionConfiguration collisionConfiguration;
   private btCollisionDispatcher collisionDispatcher;
   private btBroadphaseInterface broadphase;
   private btMultiBodyConstraintSolver solver;
   private btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final ArrayList<btRigidBody> rigidBodies = new ArrayList<>();
   private final ArrayList<btMultiBody> multiBodies = new ArrayList<>();
   private final ArrayList<btCollisionObject> collisionObjects = new ArrayList<>();
   private final ArrayList<BulletTerrainObject> terrainObjects = new ArrayList<>();

   private final ReferenceFrame inertialFrame;
   private final YoRegistry rootRegistry;
   private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());
   
   private final List<BulletBasedRobot> robotList = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();

   private boolean initialize = true;
   boolean test = true;

   public BulletBasedPhysicsEngine(ReferenceFrame inertialFrame, YoRegistry rootRegistry)
   {
      System.out.println("create");
      this.inertialFrame = inertialFrame;
      this.rootRegistry = rootRegistry;
      
      collisionConfiguration = new btDefaultCollisionConfiguration();
      collisionDispatcher = new btCollisionDispatcher(collisionConfiguration);
      broadphase = new btDbvtBroadphase();
      solver = new btMultiBodyConstraintSolver();
      multiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(collisionDispatcher, broadphase, solver, collisionConfiguration);
      Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
      multiBodyDynamicsWorld.setGravity(gravity);
   }

   @Override
   public boolean initialize(Vector3DReadOnly gravity)
   {
      if (!initialize)
         return false;

      for (BulletBasedRobot robot : robotList)
      {
         robot.initializeState();
         robot.updateSensors();
         robot.getControllerManager().initializeControllers();
      }
      initialize = false;
      return true;
   }

   @Override
   public void simulate(double currentTime, double dt, Vector3DReadOnly gravity)
   {
     
      if (initialize(gravity))
         return;
      
      for (BulletBasedRobot robot : robotList)
      {
         robot.getControllerManager().updateControllers(currentTime); 
         robot.getControllerManager().writeControllerOutput(JointStateType.EFFORT); 
         robot.getControllerManager().writeControllerOutputForJointsToIgnore(JointStateType.values()); 
         robot.saveRobotBeforePhysicsState();
         robot.updateFromBulletData();
      }
 
//      for (int i = 0; i < multiBodyDynamicsWorld.getNumCollisionObjects(); i++)
//      {
//          btCollisionObject colObj = multiBodyDynamicsWorld.getCollisionObjectArray().atConst(i);
//          System.out.println(i  + " before transform" + colObj.getWorldTransform());
//      }
      
      int maxSubSteps = 1; // With SCS, we want every tick to get a buffer entry and step through things one step at a time.
      float fixedTimeStep = (float) dt;  // SCS has a fixed timestep already so let's just use it
      float timePassedSinceThisWasCalledLast = fixedTimeStep; // We are essentially disabling interpolation here
      multiBodyDynamicsWorld.stepSimulation(timePassedSinceThisWasCalledLast, maxSubSteps, fixedTimeStep);
      
//      for (int i = 0; i < multiBodyDynamicsWorld.getNumCollisionObjects(); i++)
//      {
//          btCollisionObject colObj = multiBodyDynamicsWorld.getCollisionObjectArray().atConst(i);
//          System.out.println(i  + " after transform" + colObj.getWorldTransform());
//      }

      for (BulletBasedRobot robot : robotList)
      {
         robot.updateFrames(); 
         robot.updateSensors();
         robot.afterSimulate();
      }

   }

   @Override
   public void pause()
   {
      for (BulletBasedRobot robot : robotList)
      {
         robot.getControllerManager().pauseControllers();
      }   
   }

   @Override
   public void addRobot(Robot robot)
   {
      inertialFrame.checkReferenceFrameMatch(robot.getInertialFrame());
      BulletBasedRobot bulletRobot = new BulletBasedRobot(robot, physicsEngineRegistry);
      rootRegistry.addChild(bulletRobot.getRegistry());
      robotList.add(bulletRobot);
      bulletRobot.createBulletMultiBody(this);
   }

   public void destroy()
   {
      System.out.println("Destroy");
      for (btRigidBody rigidBody : rigidBodies)
      {
         multiBodyDynamicsWorld.removeRigidBody(rigidBody);
         rigidBody.dispose();
      }
      
      for (btMultiBody multiBody : multiBodies)
      {
         multiBodyDynamicsWorld.removeMultiBody(multiBody);
         multiBody.dispose();
      }

      for (btCollisionObject shape : collisionObjects)
      {
         multiBodyDynamicsWorld.removeCollisionObject(shape);
         shape.dispose();
      }
      collisionObjects.clear();
      
      multiBodyDynamicsWorld.dispose();
      collisionConfiguration.dispose();
      collisionDispatcher.dispose();
      broadphase.dispose();
      solver.dispose();
   }

   @Override
   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      terrainObjectDefinitions.add(terrainObjectDefinition);
      terrainObjects.add(new BulletTerrainObject(terrainObjectDefinition, multiBodyDynamicsWorld));
   }

   public void addMultiBodyCollisionShape(btMultiBodyLinkCollider collisionShape)
   {
      BulletTools.addMultiBodyCollisionShapeToWorld(multiBodyDynamicsWorld, collisionShape);
   }
   
   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   @Override
   public List<? extends Robot> getRobots()
   {
      return robotList.stream().map(BulletBasedRobot::getRobot).collect(Collectors.toList());
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

}

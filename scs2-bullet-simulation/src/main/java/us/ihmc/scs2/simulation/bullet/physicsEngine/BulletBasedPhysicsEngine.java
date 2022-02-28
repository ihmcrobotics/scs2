package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.CollisionConstants;
import com.badlogic.gdx.physics.bullet.collision.btBroadphaseInterface;
import com.badlogic.gdx.physics.bullet.collision.btCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.collision.btCollisionDispatcher;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btDbvtBroadphase;
import com.badlogic.gdx.physics.bullet.collision.btDefaultCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraintSolver;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
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
   
   private final ReferenceFrame inertialFrame;
   private final YoRegistry rootRegistry;
   private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());
   
   private final List<BulletBasedRobot> robotList = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();

   private boolean initialize = true;

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
      }
 
      int maxSubSteps = 1000; // 0 means use variable time step
      float fixedTimeStep = 1.0f / 240f;  
      multiBodyDynamicsWorld.stepSimulation((float)currentTime, maxSubSteps, fixedTimeStep); // FIXME: Sometimes EXCEPTION_ACCESS_VIOLATION
      
      for (BulletBasedRobot robot : robotList)
      {
         robot.updateFrames(); 
         robot.updateSensors();
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
      addBulletRobot(robot);
   }
   
   public BulletBasedRobot addBulletRobot(Robot robot)
   {
      inertialFrame.checkReferenceFrameMatch(robot.getInertialFrame());
      BulletBasedRobot bulletRobot = new BulletBasedRobot(robot, physicsEngineRegistry);
      rootRegistry.addChild(bulletRobot.getRegistry());
      robotList.add(bulletRobot);
      
      return bulletRobot;
   }
   
//   public void addMultiBodyRobot(Robot robot, btMultiBody multiBody, btMultiBodyLinkCollider collisionShape)
//   {
//      BulletBasedRobot bulletRobot = addBulletRobot(robot);
//      
//      multiBodyDynamicsWorld.addMultiBody(multiBody);
//      multiBodies.add(multiBody);
//      
//      int collisionGroup = 2; // Multi bodies need to be in a separate collision group
//      int collisionGroupMask = 1 + 2; // But allowed to interact with group 1, which is rigid and static bodies
//      multiBodyDynamicsWorld.addCollisionObject(collisionShape, collisionGroup, collisionGroupMask);
//      
//      //bulletRobot.setbtMultiBody(multiBody);
//   }
   
   public Robot addRobot(RobotDefinition robotDefinition)
   {
      Robot robot = new Robot(robotDefinition, inertialFrame);
      addRobot(robot);
      return robot;
   }
   
   public Robot addRigidBodyRobot(RobotDefinition robotDefinition, btCollisionShape collisionShape, float mass, Matrix4 transformToWorld, boolean isKinematicObject)
   {
      Robot robot = new Robot(robotDefinition, inertialFrame);
      addRigidBodyRobot(robot, collisionShape, mass, transformToWorld, isKinematicObject);
      return robot;
   }
   
   public btRigidBody addRigidBodyRobot(Robot robot, btCollisionShape collisionShape, float mass, Matrix4 transformToWorld, boolean isKinematicObject)
   {
      boolean isDynamic = (mass != 0.f);
      Vector3 localInertia = new Vector3(0, 0, 0);
      if (isDynamic)
          collisionShape.calculateLocalInertia(mass, localInertia);

      BulletBasedRobot bulletRobot = addBulletRobot(robot);
      bulletRobot.getBulletMotionState().setWorldTransform(transformToWorld);

      btRigidBody rigidBody = new btRigidBody(mass, bulletRobot.getBulletMotionState(), collisionShape, localInertia);

      int collisionGroup = 1; // group 1 is rigid and static bodies
      int collisionGroupMask = 1 + 2; // Allow interaction with group 2, which is multi bodies
      multiBodyDynamicsWorld.addRigidBody(rigidBody, collisionGroup, collisionGroupMask);

      if (isKinematicObject)
      {
         rigidBody.setCollisionFlags(rigidBody.getCollisionFlags() | btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
         rigidBody.setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
      }

      rigidBodies.add(rigidBody);
      
      return rigidBody;
   }

   public void destroy()
   {
      System.out.println("Destroy");
      for (btRigidBody rigidBody : rigidBodies)
      {
         multiBodyDynamicsWorld.removeRigidBody(rigidBody);
      }
      
      for (btMultiBody multiBody : multiBodies)
      {
         multiBodyDynamicsWorld.removeMultiBody(multiBody);
      }
      
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
   }
   
   public btCollisionObject addStaticObject(btCollisionShape collisionShape, Matrix4 transformToWorld)
   {
      btCollisionObject staticObject = new btCollisionObject();
      staticObject.setCollisionShape(collisionShape);
      staticObject.setWorldTransform(transformToWorld);
      multiBodyDynamicsWorld.addCollisionObject(staticObject);
      collisionObjects.add(staticObject);    
      
      return staticObject;
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

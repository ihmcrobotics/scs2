package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;

public class BulletSimulationSession extends SimulationSession
{

   public BulletSimulationSession(PhysicsEngineFactory physicsEngineFactory)
   {
      super(physicsEngineFactory);
   }
   
   public void shutdownSession()
   {
      getPhysicsEngine().destroy();
      
      super.shutdownSession();
   }
   
   public void addBulletMultiBodyRobot(RobotDefinition robotDefinition, btMultiBody bulletMultiBodyRobot)
   {
      getPhysicsEngine().addBulletMultiBodyRobot(robotDefinition, bulletMultiBodyRobot);
   }
   
   public void addRigidBodyRobot(Robot robot, btCollisionShape collisionShape, float mass, Matrix4 transformToWorld, boolean isKinematicObject)
   {
      getPhysicsEngine().addRigidBodyRobot(robot, collisionShape, mass, transformToWorld, isKinematicObject);
   }
   
   public void addRigidBodyRobot(RobotDefinition robotDefinition, btCollisionShape collisionShape, float mass, Matrix4 transformToWorld, boolean isKinematicObject)
   {
      getPhysicsEngine().addRigidBodyRobot(robotDefinition, collisionShape, mass, transformToWorld, isKinematicObject);
   }
  
   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition, btCollisionShape collisionShape, Matrix4 transformToWorld)
   {
      getPhysicsEngine().addTerrainObject(terrainObjectDefinition);
      getPhysicsEngine().addStaticObject(collisionShape, transformToWorld);
   };

   public BulletBasedPhysicsEngine getPhysicsEngine()
   {
      return (BulletBasedPhysicsEngine)super.getPhysicsEngine();
   }
   

}

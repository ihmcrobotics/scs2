package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

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

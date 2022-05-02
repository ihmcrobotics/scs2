package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

public class BulletTerrainFactory
{
   public static BulletTerrainObject newInstance(TerrainObjectDefinition terrainObjectDefinition)
   {
      btCompoundShape bulletCompoundCollisionShape = new btCompoundShape();

      for (CollisionShapeDefinition collisionShapeDefinition : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         btCollisionShape bulletCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         Matrix4 bulletTransformToWorld = new Matrix4();
         
         RigidBodyTransform collisionShapeDefinitionTransformToWorld = new RigidBodyTransform(collisionShapeDefinition.getOriginPose().getRotation(),
                                                                                              collisionShapeDefinition.getOriginPose().getTranslation());
         
         BulletTools.toBullet(collisionShapeDefinitionTransformToWorld, bulletTransformToWorld);
         bulletCompoundCollisionShape.addChildShape(bulletTransformToWorld, bulletCollisionShape);
      }

      float mass = 0.0f;
      BulletTerrainObject bulletTerrainObject = new BulletTerrainObject(mass, bulletCompoundCollisionShape);

      return bulletTerrainObject;
   }
}

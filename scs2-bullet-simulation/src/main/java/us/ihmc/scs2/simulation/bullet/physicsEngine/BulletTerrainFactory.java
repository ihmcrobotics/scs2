package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;

import org.bytedeco.bullet.BulletCollision.btCollisionShape;
import org.bytedeco.bullet.BulletCollision.btCompoundShape;
import org.bytedeco.bullet.LinearMath.btTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

public interface BulletTerrainFactory
{
   public static BulletTerrainObject newInstance(TerrainObjectDefinition terrainObjectDefinition)
   {
      btCompoundShape bulletCompoundCollisionShape = new btCompoundShape();
      ArrayList<btCollisionShape> btCollisionShapes = new ArrayList<>();

      for (CollisionShapeDefinition collisionShapeDefinition : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         btCollisionShape bulletCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         btTransform bulletTransformToWorld = new btTransform();

         RigidBodyTransform collisionShapeDefinitionTransformToWorld = new RigidBodyTransform(collisionShapeDefinition.getOriginPose().getRotation(),
                                                                                              collisionShapeDefinition.getOriginPose().getTranslation());

         BulletTools.toBullet(collisionShapeDefinitionTransformToWorld, bulletTransformToWorld);
         bulletCompoundCollisionShape.addChildShape(bulletTransformToWorld, bulletCollisionShape);
         btCollisionShapes.add(bulletCollisionShape);
      }

      float mass = 0.0f;
      BulletTerrainObject bulletTerrainObject = new BulletTerrainObject(mass, bulletCompoundCollisionShape, btCollisionShapes);

      return bulletTerrainObject;
   }
}

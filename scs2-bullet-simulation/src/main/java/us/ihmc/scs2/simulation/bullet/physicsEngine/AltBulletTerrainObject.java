package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

public class AltBulletTerrainObject
{
   private final btMotionState bulletMotionState = new btMotionState()
   {
      @Override
      public void setWorldTransform(Matrix4 transformToWorld)
      {
         // Should be always 0, the child shapes are statically placed
      }

      @Override
      public void getWorldTransform(Matrix4 transformToWorld)
      {
         // Should be always 0, the child shapes are statically placed
      }
   };
   private final btRigidBody bulletRigidBody;
   private final btMultiBodyDynamicsWorld multiBodyDynamicsWorld;

   public AltBulletTerrainObject(TerrainObjectDefinition terrainObjectDefinition, btMultiBodyDynamicsWorld multiBodyDynamicsWorld)
   {
      this.multiBodyDynamicsWorld = multiBodyDynamicsWorld;

      btCompoundShape bulletCompoundCollisionShape = new btCompoundShape();

      for (CollisionShapeDefinition collisionShapeDefinition : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         btCollisionShape bulletCollisionShape = null;
         if (collisionShapeDefinition.getGeometryDefinition() instanceof Box3DDefinition)
         {
            Box3DDefinition boxGeometryDefinition = (Box3DDefinition) collisionShapeDefinition.getGeometryDefinition();
            btBoxShape boxShape = new btBoxShape(new Vector3((float) boxGeometryDefinition.getSizeX() / 2.0f,
                                                             (float) boxGeometryDefinition.getSizeY() / 2.0f,
                                                             (float) boxGeometryDefinition.getSizeZ() / 2.0f));
            bulletCollisionShape = boxShape;
         }
         else if (collisionShapeDefinition.getGeometryDefinition() instanceof Sphere3DDefinition)
         {
            Sphere3DDefinition sphereGeometryDefinition = (Sphere3DDefinition) collisionShapeDefinition.getGeometryDefinition();
            btSphereShape sphereShape = new btSphereShape((float) sphereGeometryDefinition.getRadius());
            bulletCollisionShape = sphereShape;
         }
         else if (collisionShapeDefinition.getGeometryDefinition() instanceof Cylinder3DDefinition)
         {
            Cylinder3DDefinition cylinderGeometryDefinition = (Cylinder3DDefinition) collisionShapeDefinition.getGeometryDefinition();
            btCylinderShapeZ cylinderShape = new btCylinderShapeZ(new Vector3((float) cylinderGeometryDefinition.getRadius(),
                                                                              (float) cylinderGeometryDefinition.getRadius(),
                                                                              (float) cylinderGeometryDefinition.getLength() / 2.0f));
            bulletCollisionShape = cylinderShape;
         }
         else
         {
            LogTools.warn("Implement collision for {}", collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName());
         }

         Matrix4 bulletTransformToWorld = new Matrix4();
         RigidBodyTransform collisionShapeDefinitionTransformToWorld
               = new RigidBodyTransform(collisionShapeDefinition.getOriginPose().getRotation(),
                                        collisionShapeDefinition.getOriginPose().getTranslation());
         BulletTools.toBullet(collisionShapeDefinitionTransformToWorld, bulletTransformToWorld);
         bulletCompoundCollisionShape.addChildShape(bulletTransformToWorld, bulletCollisionShape);
      }

      bulletRigidBody = BulletTools.addStaticObjectToBulletWorld(multiBodyDynamicsWorld, bulletCompoundCollisionShape, bulletMotionState);
   }

   public btRigidBody getBulletRigidBody()
   {
      return bulletRigidBody;
   }
}

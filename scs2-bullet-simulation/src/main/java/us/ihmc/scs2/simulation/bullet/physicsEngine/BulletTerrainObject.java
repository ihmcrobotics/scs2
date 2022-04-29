package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

public class BulletTerrainObject
{
   private final btMotionState btMotionState = new btMotionState()
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
   private final btRigidBody btRigidBody;
   private final int collisionGroup = 1; // group 1 is rigid and static bodies
   private final int collisionGroupMask = -1; // Allows interaction with all groups (including custom groups)

   public BulletTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
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
         RigidBodyTransform collisionShapeDefinitionTransformToWorld = new RigidBodyTransform(collisionShapeDefinition.getOriginPose().getRotation(),
                                                                                              collisionShapeDefinition.getOriginPose().getTranslation());
         BulletTools.toBullet(collisionShapeDefinitionTransformToWorld, bulletTransformToWorld);
         bulletCompoundCollisionShape.addChildShape(bulletTransformToWorld, bulletCollisionShape);
      }

      Vector3 localInertia = new Vector3();
      bulletCompoundCollisionShape.calculateLocalInertia(0.0f, localInertia);
      btRigidBody = new btRigidBody(0.0f, btMotionState, bulletCompoundCollisionShape, localInertia);
   }

   public btRigidBody getBtRigidBody()
   {
      return btRigidBody;
   }

   public int getCollisionGroup()
   {
      return collisionGroup;
   }

   public int getCollisionGroupMask()
   {
      return collisionGroupMask;
   }

}

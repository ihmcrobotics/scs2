package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.BroadphaseNativeTypes;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCapsuleShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.collision.btConeShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btCylinderShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class BulletTerrainFactoryTest
{
   static
   {
      Bullet.init();
      LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());
   }

   private static final double EPSILON = 1e-5;
   private static final int ITERATIONS = 1000;

   private static final Vector3 boxVertex = new Vector3();

   @Test
   public void testNewInstance()
   {
      Random random = new Random(223174);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3DDefinition terrainGeometry = new Box3DDefinition(random.nextDouble(), random.nextDouble(), random.nextDouble());
         RigidBodyTransform terrainPose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                            terrainGeometry,
                                                                                            new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                       new CollisionShapeDefinition(terrainPose, terrainGeometry));

         BulletTerrainObject bulletTerrainObject = BulletTerrainFactory.newInstance(terrain);
         btCompoundShape compoundShape = (btCompoundShape) bulletTerrainObject.getBtRigidBody().getCollisionShape();

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE);
         assertEquals(bulletTerrainObject.getCollisionGroup(), 1);
         assertEquals(bulletTerrainObject.getCollisionGroupMask(), -1);

         Matrix4 childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btBoxShape btBoxShape = (btBoxShape) compoundShape.getChildShape(0);

         for (int j = 0; j < btBoxShape.getNumEdges(); j++)
         {
            btBoxShape.getVertex(j, boxVertex);

            assertEquals(Math.abs(boxVertex.x), (float) terrainGeometry.getSizeX() / 2.0f, EPSILON);
            assertEquals(Math.abs(boxVertex.y), (float) terrainGeometry.getSizeY() / 2.0f, EPSILON);
            assertEquals(Math.abs(boxVertex.z), (float) terrainGeometry.getSizeZ() / 2.0f, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3DDefinition terrainGeometry = new Sphere3DDefinition(random.nextDouble());
         RigidBodyTransform terrainPose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                            terrainGeometry,
                                                                                            new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                       new CollisionShapeDefinition(terrainPose, terrainGeometry));

         BulletTerrainObject bulletTerrainObject = BulletTerrainFactory.newInstance(terrain);
         btCompoundShape compoundShape = (btCompoundShape) bulletTerrainObject.getBtRigidBody().getCollisionShape();

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE);

         Matrix4 childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btSphereShape sphereShape = (btSphereShape) compoundShape.getChildShape(0);
         assertEquals(sphereShape.getRadius(), (float) terrainGeometry.getRadius());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3DDefinition terrainGeometry = new Cylinder3DDefinition(random.nextDouble(), random.nextDouble());
         RigidBodyTransform terrainPose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                            terrainGeometry,
                                                                                            new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                       new CollisionShapeDefinition(terrainPose, terrainGeometry));

         BulletTerrainObject bulletTerrainObject = BulletTerrainFactory.newInstance(terrain);
         btCompoundShape compoundShape = (btCompoundShape) bulletTerrainObject.getBtRigidBody().getCollisionShape();

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE);

         Matrix4 childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btCylinderShapeZ cylinderShape = (btCylinderShapeZ) compoundShape.getChildShape(0);

         assertEquals(cylinderShape.getRadius(), (float) terrainGeometry.getRadius(), EPSILON);
         assertEquals(cylinderShape.getHalfExtentsWithMargin().z, (float) terrainGeometry.getLength() / 2.0f, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cone3DDefinition terrainGeometry = new Cone3DDefinition(random.nextDouble(), random.nextDouble());
         RigidBodyTransform terrainPose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                            terrainGeometry,
                                                                                            new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                       new CollisionShapeDefinition(terrainPose, terrainGeometry));

         BulletTerrainObject bulletTerrainObject = BulletTerrainFactory.newInstance(terrain);
         btCompoundShape compoundShape = (btCompoundShape) bulletTerrainObject.getBtRigidBody().getCollisionShape();

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE);

         Matrix4 childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btConeShapeZ coneShape = (btConeShapeZ) compoundShape.getChildShape(0);
         assertEquals(coneShape.getRadius(), (float) terrainGeometry.getRadius(), EPSILON);
         assertEquals(coneShape.getHeight(), (float) terrainGeometry.getHeight(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3DDefinition terrainGeometry = new Capsule3DDefinition(random.nextDouble(), random.nextDouble());
         RigidBodyTransform terrainPose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                            terrainGeometry,
                                                                                            new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                       new CollisionShapeDefinition(terrainPose, terrainGeometry));

         BulletTerrainObject bulletTerrainObject = BulletTerrainFactory.newInstance(terrain);
         btCompoundShape compoundShape = (btCompoundShape) bulletTerrainObject.getBtRigidBody().getCollisionShape();

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE);

         Matrix4 childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btCapsuleShapeZ capsuleShape = (btCapsuleShapeZ) compoundShape.getChildShape(0);
         assertEquals(capsuleShape.getRadius(), (float) terrainGeometry.getRadiusX(), EPSILON);
         assertEquals(capsuleShape.getRadius(), (float) terrainGeometry.getRadiusY(), EPSILON);
         assertEquals(capsuleShape.getRadius(), (float) terrainGeometry.getRadiusZ(), EPSILON);
         assertEquals(capsuleShape.getHalfHeight(), (float) terrainGeometry.getLength() / 2.0f, EPSILON);
      }
   }

   private static void assertChildTransformEqualToTerrainPose(RigidBodyTransform terrainPose, Matrix4 childTransform)
   {
      assertEquals(childTransform.val[Matrix4.M00], (float) terrainPose.getM00(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M01], (float) terrainPose.getM01(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M02], (float) terrainPose.getM02(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M10], (float) terrainPose.getM10(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M11], (float) terrainPose.getM11(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M12], (float) terrainPose.getM12(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M20], (float) terrainPose.getM20(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M21], (float) terrainPose.getM21(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M22], (float) terrainPose.getM22(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M03], (float) terrainPose.getM03(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M13], (float) terrainPose.getM13(), EPSILON);
      assertEquals(childTransform.val[Matrix4.M23], (float) terrainPose.getM23(), EPSILON);
   }
}

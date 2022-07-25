package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.bytedeco.bullet.BulletCollision.btBoxShape;
import org.bytedeco.bullet.BulletCollision.btCapsuleShapeZ;
import org.bytedeco.bullet.BulletCollision.btCompoundShape;
import org.bytedeco.bullet.BulletCollision.btConeShapeZ;
import org.bytedeco.bullet.BulletCollision.btCylinderShapeZ;
import org.bytedeco.bullet.BulletCollision.btSphereShape;
import org.bytedeco.bullet.LinearMath.btTransform;
import org.bytedeco.bullet.LinearMath.btVector3;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletTools.BroadphaseNativeTypes;

public class BulletTerrainFactoryTest
{
   private static final double EPSILON = 1e-5;
   private static final int ITERATIONS = 1000;

   private static final btVector3 boxVertex = new btVector3();

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
         //TODO: get with ByteDeco to see why getCollisionShape can not be cast as btCompoundShape - the logic gets an exception here
         btCompoundShape compoundShape = (btCompoundShape) bulletTerrainObject.getBtRigidBody().getCollisionShape();

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE.ordinal());
         assertEquals(bulletTerrainObject.getCollisionGroup(), 1);
         assertEquals(bulletTerrainObject.getCollisionGroupMask(), -1);

         btTransform childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btBoxShape btBoxShape = (btBoxShape) compoundShape.getChildShape(0);

         for (int j = 0; j < btBoxShape.getNumEdges(); j++)
         {
            btBoxShape.getVertex(j, boxVertex);

            assertEquals(Math.abs(boxVertex.getX()), terrainGeometry.getSizeX() / 2.0, EPSILON);
            assertEquals(Math.abs(boxVertex.getY()), terrainGeometry.getSizeY() / 2.0, EPSILON);
            assertEquals(Math.abs(boxVertex.getZ()), terrainGeometry.getSizeZ() / 2.0, EPSILON);
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

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE.ordinal());

         btTransform childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btSphereShape sphereShape = (btSphereShape) compoundShape.getChildShape(0);
         assertEquals(sphereShape.getRadius(), terrainGeometry.getRadius());
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

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE.ordinal());

         btTransform childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btCylinderShapeZ cylinderShape = (btCylinderShapeZ) compoundShape.getChildShape(0);

         assertEquals(cylinderShape.getRadius(), terrainGeometry.getRadius(), EPSILON);
         assertEquals(cylinderShape.getHalfExtentsWithMargin().getZ(), terrainGeometry.getLength() / 2.0, EPSILON);
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

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE.ordinal());

         btTransform childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btConeShapeZ coneShape = (btConeShapeZ) compoundShape.getChildShape(0);
         assertEquals(coneShape.getRadius(), terrainGeometry.getRadius(), EPSILON);
         assertEquals(coneShape.getHeight(), terrainGeometry.getHeight(), EPSILON);
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

         assertEquals(compoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE.ordinal());

         btTransform childTransform = compoundShape.getChildTransform(0);
         assertChildTransformEqualToTerrainPose(terrainPose, childTransform);

         btCapsuleShapeZ capsuleShape = (btCapsuleShapeZ) compoundShape.getChildShape(0);
         assertEquals(capsuleShape.getRadius(), terrainGeometry.getRadiusX(), EPSILON);
         assertEquals(capsuleShape.getRadius(), terrainGeometry.getRadiusY(), EPSILON);
         assertEquals(capsuleShape.getRadius(), terrainGeometry.getRadiusZ(), EPSILON);
         assertEquals(capsuleShape.getHalfHeight(), terrainGeometry.getLength() / 2.0, EPSILON);
      }
   }

   private static void assertChildTransformEqualToTerrainPose(RigidBodyTransform terrainPose, btTransform childTransform)
   {
      assertEquals(childTransform.getOrigin().getX(), terrainPose.getM00(), EPSILON);
      assertEquals(childTransform.getOrigin().getY(), terrainPose.getM01(), EPSILON);
      assertEquals(childTransform.getOrigin().getZ(), terrainPose.getM02(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(0).getX(), terrainPose.getM10(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(0).getY(), terrainPose.getM11(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(0).getZ(), terrainPose.getM12(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(1).getX(), terrainPose.getM20(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(1).getY(), terrainPose.getM21(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(1).getZ(), terrainPose.getM22(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(2).getX(), terrainPose.getM03(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(2).getY(), terrainPose.getM13(), EPSILON);
      assertEquals(childTransform.getBasis().getRow(2).getZ(), terrainPose.getM23(), EPSILON);
   }
}

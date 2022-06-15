package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;

import org.bytedeco.bullet.BulletCollision.btBoxShape;
import org.bytedeco.bullet.BulletCollision.btCapsuleShapeZ;
import org.bytedeco.bullet.BulletCollision.btCollisionShape;
import org.bytedeco.bullet.BulletCollision.btConeShapeZ;
import org.bytedeco.bullet.BulletCollision.btCylinderShapeZ;
import org.bytedeco.bullet.BulletCollision.btSphereShape;
import org.bytedeco.bullet.LinearMath.btQuaternion;
import org.bytedeco.bullet.LinearMath.btTransform;
import org.bytedeco.bullet.LinearMath.btVector3;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletTools.BroadphaseNativeTypes;

public class BulletToolsTest
{
//   static
//   {
//      Bullet.init();
//      LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());
//   }

   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1e-5;

   //private final btTransform bulletAffineToPack = new btTransform();
   private final RigidBodyTransform rigidBodyTransformToPack = new RigidBodyTransform();
   private final btVector3 translation = new btVector3();
   private final btTransform bulletAffine = new btTransform();
   private final btVector3 vector1 = new btVector3();
   //private final btVector3 vector2 = new btVector3();
   private final btQuaternion bulletQuaternion = new btQuaternion();
   private final Vector3DBasics euclidVector3D32 = new Vector3D();
   private final Point3DBasics euclidPoint3D32 = new Point3D();

   @Test
   public void testRigidBodyTransformToMatrix4()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         Random random = new Random(14474);
         RigidBodyTransform rigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

//         BulletTools.toBullet(rigidBodyTransform, bulletAffineToPack);
//
//         bulletAffineToPack.getTranslation(translation);
//
//         assertEquals(translation.x, (float) rigidBodyTransform.getTranslationX());
//         assertEquals(translation.y, (float) rigidBodyTransform.getTranslationY());
//         assertEquals(translation.z, (float) rigidBodyTransform.getTranslationZ());
//         assertEquals(bulletAffineToPack.val[Matrix4.M00], (float) rigidBodyTransform.getM00());
//         assertEquals(bulletAffineToPack.val[Matrix4.M01], (float) rigidBodyTransform.getM01());
//         assertEquals(bulletAffineToPack.val[Matrix4.M02], (float) rigidBodyTransform.getM02());
//         assertEquals(bulletAffineToPack.val[Matrix4.M10], (float) rigidBodyTransform.getM10());
//         assertEquals(bulletAffineToPack.val[Matrix4.M11], (float) rigidBodyTransform.getM11());
//         assertEquals(bulletAffineToPack.val[Matrix4.M12], (float) rigidBodyTransform.getM12());
//         assertEquals(bulletAffineToPack.val[Matrix4.M20], (float) rigidBodyTransform.getM20());
//         assertEquals(bulletAffineToPack.val[Matrix4.M21], (float) rigidBodyTransform.getM21());
//         assertEquals(bulletAffineToPack.val[Matrix4.M22], (float) rigidBodyTransform.getM22());
      }
   }

   @Test
   public void testMatrix4ToRigidBodyTransform()
   {
      Random random = new Random(14474);

      for (int i = 0; i < ITERATIONS; i++)
      {
         vector1.setX(random.nextFloat());
         vector1.setY(random.nextFloat());
         vector1.setZ(random.nextFloat());

         bulletQuaternion.setRotation(vector1, random.nextFloat());

         bulletAffine.setRotation(bulletQuaternion);  

         translation.setX(random.nextFloat());
         translation.setY(random.nextFloat());
         translation.setZ(random.nextFloat());
         bulletAffine.setOrigin(translation);

         BulletTools.toEuclid(bulletAffine, rigidBodyTransformToPack);

         assertEquals(bulletAffine.getOrigin().getX(), (float) rigidBodyTransformToPack.getTranslationX(), EPSILON);
         assertEquals(bulletAffine.getOrigin().getY(), (float) rigidBodyTransformToPack.getTranslationY(), EPSILON);
         assertEquals(bulletAffine.getOrigin().getZ(), (float) rigidBodyTransformToPack.getTranslationZ(), EPSILON);

//         assertEquals(bulletAffine.val[Matrix4.M00], (float) rigidBodyTransformToPack.getM00(), EPSILON);
//         assertEquals(bulletAffine.val[Matrix4.M01], (float) rigidBodyTransformToPack.getM01(), EPSILON);
//         assertEquals(bulletAffine.val[Matrix4.M02], (float) rigidBodyTransformToPack.getM02(), EPSILON);
//         assertEquals(bulletAffine.val[Matrix4.M10], (float) rigidBodyTransformToPack.getM10(), EPSILON);
//         assertEquals(bulletAffine.val[Matrix4.M11], (float) rigidBodyTransformToPack.getM11(), EPSILON);
//         assertEquals(bulletAffine.val[Matrix4.M12], (float) rigidBodyTransformToPack.getM12(), EPSILON);
//         assertEquals(bulletAffine.val[Matrix4.M20], (float) rigidBodyTransformToPack.getM20(), EPSILON);
//         assertEquals(bulletAffine.val[Matrix4.M21], (float) rigidBodyTransformToPack.getM21(), EPSILON);
//         assertEquals(bulletAffine.val[Matrix4.M22], (float) rigidBodyTransformToPack.getM22(), EPSILON);
      }
   }

   @Test
   public void testEuclidQuaternionToBulletQuaternion()
   {
      Random random = new Random(21714);

      for (int i = 0; i < ITERATIONS; i++)
      {
         us.ihmc.euclid.tuple4D.Quaternion euclidQuaternion = EuclidCoreRandomTools.nextQuaternion(random);

         BulletTools.toBullet(euclidQuaternion, bulletQuaternion);

         assertEquals(euclidQuaternion.getX(), bulletQuaternion.getX(), EPSILON);
         assertEquals(euclidQuaternion.getY(), bulletQuaternion.getY(), EPSILON);
         assertEquals(euclidQuaternion.getZ(), bulletQuaternion.getZ(), EPSILON);
         assertEquals(euclidQuaternion.getS(), bulletQuaternion.getW(), EPSILON);
      }
   }

   @Test
   public void testTupleToVector3()
   {
      Random random = new Random(42518);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Tuple3DBasics euclidTuple = EuclidCoreRandomTools.nextPoint3D(random);

         BulletTools.toBullet(euclidTuple, vector1);

         assertEquals(euclidTuple.getX(), vector1.getX(), EPSILON);
         assertEquals(euclidTuple.getY(), vector1.getY(), EPSILON);
         assertEquals(euclidTuple.getZ(), vector1.getZ(), EPSILON);
      }
   }

   @Test
   public void testVector3ToVector3D()
   {
      Random random = new Random(96571);

      for (int i = 0; i < ITERATIONS; i++)
      {
         vector1.setX(random.nextFloat());
         vector1.setY(random.nextFloat());
         vector1.setZ(random.nextFloat());

         BulletTools.toEuclid(vector1, euclidVector3D32);

         assertEquals(euclidVector3D32.getX(), vector1.getX(), EPSILON);
         assertEquals(euclidVector3D32.getY(), vector1.getY(), EPSILON);
         assertEquals(euclidVector3D32.getZ(), vector1.getZ(), EPSILON);
      }
   }

   @Test
   public void testVector3ToPoint3DBasics()
   {
      Random random = new Random(54214);

      for (int i = 0; i < ITERATIONS; i++)
      {
         vector1.setX(random.nextFloat());
         vector1.setY(random.nextFloat());
         vector1.setZ(random.nextFloat());

         BulletTools.toEuclid(vector1, euclidPoint3D32);

         assertEquals(euclidPoint3D32.getX(), vector1.getX(), EPSILON);
         assertEquals(euclidPoint3D32.getY(), vector1.getY(), EPSILON);
         assertEquals(euclidPoint3D32.getZ(), vector1.getZ(), EPSILON);
      }
   }

   @Test
   public void testCreateBulletCollisionShape()
   {
      Random random = new Random(957123);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3DDefinition boxGeometryDefinition = new Box3DDefinition(random.nextDouble(), random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(boxGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE);

         btBoxShape btBoxShape = (btBoxShape) btCollisionShape;

         for (int j = 0; j < btBoxShape.getNumEdges(); j++)
         {
            btBoxShape.getVertex(j, vector1);

            assertEquals(Math.abs(vector1.getX()), (float) boxGeometryDefinition.getSizeX() / 2.0f, EPSILON);
            assertEquals(Math.abs(vector1.getY()), (float) boxGeometryDefinition.getSizeY() / 2.0f, EPSILON);
            assertEquals(Math.abs(vector1.getZ()), (float) boxGeometryDefinition.getSizeZ() / 2.0f, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3DDefinition sphereGeometryDefinition = new Sphere3DDefinition(random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(sphereGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE);

         btSphereShape sphereShape = (btSphereShape) btCollisionShape;

         assertEquals(sphereShape.getRadius(), (float) sphereGeometryDefinition.getRadius());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3DDefinition cylinderGeometryDefinition = new Cylinder3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(cylinderGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE);

         btCylinderShapeZ cylinderShape = (btCylinderShapeZ) btCollisionShape;

         assertEquals(cylinderShape.getRadius(), (float) cylinderGeometryDefinition.getRadius(), EPSILON);
         assertEquals(cylinderShape.getHalfExtentsWithMargin().getZ(), (float) cylinderGeometryDefinition.getLength() / 2.0f, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cone3DDefinition coneGeometryDefinition = new Cone3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(coneGeometryDefinition);

         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE);

         btConeShapeZ coneShape = (btConeShapeZ) btCollisionShape;
         assertEquals(coneShape.getRadius(), (float) coneGeometryDefinition.getRadius(), EPSILON);
         assertEquals(coneShape.getHeight(), (float) coneGeometryDefinition.getHeight(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3DDefinition capsuleGeometryDefinition = new Capsule3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(capsuleGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE);

         btCapsuleShapeZ capsuleShape = (btCapsuleShapeZ) btCollisionShape;
         assertEquals(capsuleShape.getRadius(), (float) capsuleGeometryDefinition.getRadiusX(), EPSILON);
         assertEquals(capsuleShape.getRadius(), (float) capsuleGeometryDefinition.getRadiusY(), EPSILON);
         assertEquals(capsuleShape.getRadius(), (float) capsuleGeometryDefinition.getRadiusZ(), EPSILON);
         assertEquals(capsuleShape.getHalfHeight(), (float) capsuleGeometryDefinition.getLength() / 2.0f, EPSILON);
      }

      Ellipsoid3DDefinition polytypeGeometryDefinition = new Ellipsoid3DDefinition(1.0, 1.0, 1.0);
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(polytypeGeometryDefinition);

      Assertions.assertThrows(UnsupportedOperationException.class, () ->
      {
         BulletTools.createBulletCollisionShape(collisionShapeDefinition);
      });

   }
}

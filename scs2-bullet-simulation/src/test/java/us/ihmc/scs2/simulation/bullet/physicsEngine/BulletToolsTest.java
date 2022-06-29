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
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1e-5;

   private final btTransform bulletAffineToPack = new btTransform();
   private final RigidBodyTransform rigidBodyTransformToPack = new RigidBodyTransform();
   private final btVector3 translation = new btVector3();
   private final btTransform bulletAffine = new btTransform();
   private final btVector3 vector1 = new btVector3();
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

         BulletTools.toBullet(rigidBodyTransform, bulletAffineToPack);

         assertEquals(bulletAffineToPack.getOrigin().getX(), (float) rigidBodyTransform.getTranslationX(), EPSILON);
         assertEquals(bulletAffineToPack.getOrigin().getY(), (float) rigidBodyTransform.getTranslationY(), EPSILON);
         assertEquals(bulletAffineToPack.getOrigin().getZ(), (float) rigidBodyTransform.getTranslationZ(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(0).getX(), (float) rigidBodyTransform.getM00(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(0).getY(), (float) rigidBodyTransform.getM01(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(0).getZ(), (float) rigidBodyTransform.getM02(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(1).getX(), (float) rigidBodyTransform.getM10(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(1).getY(), (float) rigidBodyTransform.getM11(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(1).getZ(), (float) rigidBodyTransform.getM12(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(2).getX(), (float) rigidBodyTransform.getM20(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(2).getY(), (float) rigidBodyTransform.getM21(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(2).getZ(), (float) rigidBodyTransform.getM22(), EPSILON);
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
         assertEquals(bulletAffine.getBasis().getRow(0).getX(), (float) rigidBodyTransformToPack.getM00(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(0).getY(), (float) rigidBodyTransformToPack.getM01(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(0).getZ(), (float) rigidBodyTransformToPack.getM02(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(1).getX(), (float) rigidBodyTransformToPack.getM10(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(1).getY(), (float) rigidBodyTransformToPack.getM11(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(1).getZ(), (float) rigidBodyTransformToPack.getM12(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(2).getX(), (float) rigidBodyTransformToPack.getM20(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(2).getY(), (float) rigidBodyTransformToPack.getM21(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(2).getZ(), (float) rigidBodyTransformToPack.getM22(), EPSILON);
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

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE.ordinal());

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

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE.ordinal());

         btSphereShape sphereShape = (btSphereShape) btCollisionShape;

         assertEquals(sphereShape.getRadius(), (float) sphereGeometryDefinition.getRadius());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3DDefinition cylinderGeometryDefinition = new Cylinder3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(cylinderGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE.ordinal());

         btCylinderShapeZ cylinderShape = (btCylinderShapeZ) btCollisionShape;

         assertEquals(cylinderShape.getRadius(), (float) cylinderGeometryDefinition.getRadius(), EPSILON);
         assertEquals(cylinderShape.getHalfExtentsWithMargin().getZ(), (float) cylinderGeometryDefinition.getLength() / 2.0f, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cone3DDefinition coneGeometryDefinition = new Cone3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(coneGeometryDefinition);

         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE.ordinal());

         btConeShapeZ coneShape = (btConeShapeZ) btCollisionShape;
         assertEquals(coneShape.getRadius(), (float) coneGeometryDefinition.getRadius(), EPSILON);
         assertEquals(coneShape.getHeight(), (float) coneGeometryDefinition.getHeight(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3DDefinition capsuleGeometryDefinition = new Capsule3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(capsuleGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE.ordinal());

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

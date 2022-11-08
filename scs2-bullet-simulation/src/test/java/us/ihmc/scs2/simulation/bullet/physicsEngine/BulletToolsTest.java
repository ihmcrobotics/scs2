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
   private final Vector3DBasics euclidVector3D = new Vector3D();
   private final Point3DBasics euclidPoint3D = new Point3D();

   @Test
   public void testRigidBodyTransformToMatrix4()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         Random random = new Random(14474);
         RigidBodyTransform rigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         BulletTools.toBullet(rigidBodyTransform, bulletAffineToPack);

         assertEquals(bulletAffineToPack.getOrigin().getX(), rigidBodyTransform.getTranslationX(), EPSILON);
         assertEquals(bulletAffineToPack.getOrigin().getY(), rigidBodyTransform.getTranslationY(), EPSILON);
         assertEquals(bulletAffineToPack.getOrigin().getZ(), rigidBodyTransform.getTranslationZ(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(0).getX(), rigidBodyTransform.getRotation().getM00(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(0).getY(), rigidBodyTransform.getRotation().getM01(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(0).getZ(), rigidBodyTransform.getRotation().getM02(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(1).getX(), rigidBodyTransform.getRotation().getM10(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(1).getY(), rigidBodyTransform.getRotation().getM11(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(1).getZ(), rigidBodyTransform.getRotation().getM12(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(2).getX(), rigidBodyTransform.getRotation().getM20(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(2).getY(), rigidBodyTransform.getRotation().getM21(), EPSILON);
         assertEquals(bulletAffineToPack.getBasis().getRow(2).getZ(), rigidBodyTransform.getRotation().getM22(), EPSILON);
      }
   }

   @Test
   public void testMatrix4ToRigidBodyTransform()
   {
      Random random = new Random(14474);

      for (int i = 0; i < ITERATIONS; i++)
      {
         vector1.setX(random.nextDouble());
         vector1.setY(random.nextDouble());
         vector1.setZ(random.nextDouble());

         bulletQuaternion.setRotation(vector1, random.nextDouble());

         bulletAffine.setRotation(bulletQuaternion);

         translation.setX(random.nextDouble());
         translation.setY(random.nextDouble());
         translation.setZ(random.nextDouble());
         bulletAffine.setOrigin(translation);

         BulletTools.toEuclid(bulletAffine, rigidBodyTransformToPack);

         assertEquals(bulletAffine.getOrigin().getX(), rigidBodyTransformToPack.getTranslationX(), EPSILON);
         assertEquals(bulletAffine.getOrigin().getY(), rigidBodyTransformToPack.getTranslationY(), EPSILON);
         assertEquals(bulletAffine.getOrigin().getZ(), rigidBodyTransformToPack.getTranslationZ(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(0).getX(), rigidBodyTransformToPack.getM00(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(0).getY(), rigidBodyTransformToPack.getM01(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(0).getZ(), rigidBodyTransformToPack.getM02(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(1).getX(), rigidBodyTransformToPack.getM10(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(1).getY(), rigidBodyTransformToPack.getM11(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(1).getZ(), rigidBodyTransformToPack.getM12(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(2).getX(), rigidBodyTransformToPack.getM20(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(2).getY(), rigidBodyTransformToPack.getM21(), EPSILON);
         assertEquals(bulletAffine.getBasis().getRow(2).getZ(), rigidBodyTransformToPack.getM22(), EPSILON);
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
         vector1.setX(random.nextDouble());
         vector1.setY(random.nextDouble());
         vector1.setZ(random.nextDouble());

         BulletTools.toEuclid(vector1, euclidVector3D);

         assertEquals(euclidVector3D.getX(), vector1.getX(), EPSILON);
         assertEquals(euclidVector3D.getY(), vector1.getY(), EPSILON);
         assertEquals(euclidVector3D.getZ(), vector1.getZ(), EPSILON);
      }
   }

   @Test
   public void testVector3ToPoint3DBasics()
   {
      Random random = new Random(54214);

      for (int i = 0; i < ITERATIONS; i++)
      {
         vector1.setX(random.nextDouble());
         vector1.setY(random.nextDouble());
         vector1.setZ(random.nextDouble());

         BulletTools.toEuclid(vector1, euclidPoint3D);

         assertEquals(euclidPoint3D.getX(), vector1.getX(), EPSILON);
         assertEquals(euclidPoint3D.getY(), vector1.getY(), EPSILON);
         assertEquals(euclidPoint3D.getZ(), vector1.getZ(), EPSILON);
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

            assertEquals(Math.abs(vector1.getX()), boxGeometryDefinition.getSizeX() / 2.0, EPSILON);
            assertEquals(Math.abs(vector1.getY()), boxGeometryDefinition.getSizeY() / 2.0, EPSILON);
            assertEquals(Math.abs(vector1.getZ()), boxGeometryDefinition.getSizeZ() / 2.0, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3DDefinition sphereGeometryDefinition = new Sphere3DDefinition(random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(sphereGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE.ordinal());

         btSphereShape sphereShape = (btSphereShape) btCollisionShape;

         assertEquals(sphereShape.getRadius(), sphereGeometryDefinition.getRadius());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3DDefinition cylinderGeometryDefinition = new Cylinder3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(cylinderGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE.ordinal());

         btCylinderShapeZ cylinderShape = (btCylinderShapeZ) btCollisionShape;

         assertEquals(cylinderShape.getRadius(), cylinderGeometryDefinition.getRadius(), EPSILON);
         assertEquals(cylinderShape.getHalfExtentsWithMargin().getZ(), cylinderGeometryDefinition.getLength() / 2.0, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cone3DDefinition coneGeometryDefinition = new Cone3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(coneGeometryDefinition);

         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE.ordinal());

         btConeShapeZ coneShape = (btConeShapeZ) btCollisionShape;
         assertEquals(coneShape.getRadius(), coneGeometryDefinition.getRadius(), EPSILON);
         assertEquals(coneShape.getHeight(), coneGeometryDefinition.getHeight(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3DDefinition capsuleGeometryDefinition = new Capsule3DDefinition(random.nextDouble(), random.nextDouble());
         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(capsuleGeometryDefinition);
         btCollisionShape btCollisionShape = BulletTools.createBulletCollisionShape(collisionShapeDefinition);

         assertEquals(btCollisionShape.getShapeType(), BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE.ordinal());

         btCapsuleShapeZ capsuleShape = (btCapsuleShapeZ) btCollisionShape;
         assertEquals(capsuleShape.getRadius(), capsuleGeometryDefinition.getRadiusX(), EPSILON);
         assertEquals(capsuleShape.getRadius(), capsuleGeometryDefinition.getRadiusY(), EPSILON);
         assertEquals(capsuleShape.getRadius(), capsuleGeometryDefinition.getRadiusZ(), EPSILON);
         assertEquals(capsuleShape.getHalfHeight(), capsuleGeometryDefinition.getLength() / 2.0, EPSILON);
      }

      Ellipsoid3DDefinition polytypeGeometryDefinition = new Ellipsoid3DDefinition(1.0, 1.0, 1.0);
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(polytypeGeometryDefinition);

      Assertions.assertThrows(UnsupportedOperationException.class, () ->
      {
         BulletTools.createBulletCollisionShape(collisionShapeDefinition);
      });

   }
}

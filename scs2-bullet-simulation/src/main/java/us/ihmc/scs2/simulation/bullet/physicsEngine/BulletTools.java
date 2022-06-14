package us.ihmc.scs2.simulation.bullet.physicsEngine;

import org.bytedeco.bullet.BulletCollision.btBoxShape;
import org.bytedeco.bullet.BulletCollision.btCapsuleShapeZ;
import org.bytedeco.bullet.BulletCollision.btCollisionShape;
import org.bytedeco.bullet.BulletCollision.btCompoundFromGimpactShape;
import org.bytedeco.bullet.BulletCollision.btCompoundShape;
import org.bytedeco.bullet.BulletCollision.btConeShapeZ;
import org.bytedeco.bullet.BulletCollision.btConvexHullShape;
import org.bytedeco.bullet.BulletCollision.btConvexTriangleMeshShape;
import org.bytedeco.bullet.BulletCollision.btCylinderShapeZ;
import org.bytedeco.bullet.BulletCollision.btGImpactMeshShape;
import org.bytedeco.bullet.BulletCollision.btSphereShape;
import org.bytedeco.bullet.BulletCollision.btTriangleMesh;
import org.bytedeco.bullet.LinearMath.btMatrix3x3;
import org.bytedeco.bullet.LinearMath.btQuaternion;
import org.bytedeco.bullet.LinearMath.btTransform;
import org.bytedeco.bullet.LinearMath.btVector3;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;

public class BulletTools
{
   private static btMatrix3x3 btMatrix3x3ToPack = new btMatrix3x3();
   private static btVector3 btVector3 = new btVector3();
   
   public static void toBullet(RigidBodyTransform rigidBodyTransform, btTransform bulletAffineToPack)
   {
      btMatrix3x3ToPack.setValue((float) rigidBodyTransform.getM00(), (float) rigidBodyTransform.getM01(), (float) rigidBodyTransform.getM02(), (float) rigidBodyTransform.getM10(), (float) rigidBodyTransform.getM11(), (float) rigidBodyTransform.getM12(), (float) rigidBodyTransform.getM20(), (float) rigidBodyTransform.getM21(), (float) rigidBodyTransform.getM22());
      bulletAffineToPack.setBasis(btMatrix3x3ToPack);
      btVector3.setValue((float) rigidBodyTransform.getM03(), (float) rigidBodyTransform.getM03(), (float) rigidBodyTransform.getM23());
      bulletAffineToPack.setOrigin(btVector3);
//      bulletAffineToPack.val[Matrix4.M00] = (float) rigidBodyTransform.getM00();
//      bulletAffineToPack.val[Matrix4.M01] = (float) rigidBodyTransform.getM01();
//      bulletAffineToPack.val[Matrix4.M02] = (float) rigidBodyTransform.getM02();
//      bulletAffineToPack.val[Matrix4.M10] = (float) rigidBodyTransform.getM10();
//      bulletAffineToPack.val[Matrix4.M11] = (float) rigidBodyTransform.getM11();
//      bulletAffineToPack.val[Matrix4.M12] = (float) rigidBodyTransform.getM12();
//      bulletAffineToPack.val[Matrix4.M20] = (float) rigidBodyTransform.getM20();
//      bulletAffineToPack.val[Matrix4.M21] = (float) rigidBodyTransform.getM21();
//      bulletAffineToPack.val[Matrix4.M22] = (float) rigidBodyTransform.getM22();
//      bulletAffineToPack.val[Matrix4.M03] = (float) rigidBodyTransform.getM03();
//      bulletAffineToPack.val[Matrix4.M13] = (float) rigidBodyTransform.getM03();
//      bulletAffineToPack.val[Matrix4.M23] = (float) rigidBodyTransform.getM23();
   }

   public static void toEuclid(btTransform bulletAffine, RigidBodyTransform rigidBodyTransform)
   {
      btMatrix3x3ToPack = bulletAffine.getBasis();
      rigidBodyTransform.getRotation().setAndNormalize(btMatrix3x3ToPack.getColumn(0).getX(),
                                                       btMatrix3x3ToPack.getColumn(0).getY(),
                                                       btMatrix3x3ToPack.getColumn(0).getZ(),
                                                       btMatrix3x3ToPack.getColumn(1).getX(),
                                                       btMatrix3x3ToPack.getColumn(1).getY(),
                                                       btMatrix3x3ToPack.getColumn(1).getZ(),
                                                       btMatrix3x3ToPack.getColumn(2).getX(),
                                                       btMatrix3x3ToPack.getColumn(2).getY(),
                                                       btMatrix3x3ToPack.getColumn(2).getZ());
      rigidBodyTransform.getTranslation().setX(btVector3.getX());
      rigidBodyTransform.getTranslation().setY(btVector3.getY());
      rigidBodyTransform.getTranslation().setZ(btVector3.getZ());
   }

   public static void toBullet(us.ihmc.euclid.tuple4D.Quaternion euclidQuaternion, btQuaternion bulletQuaternion)
   {
      bulletQuaternion.setValue(euclidQuaternion.getX32(), euclidQuaternion.getY32(), euclidQuaternion.getZ32(), euclidQuaternion.getS32()); 
   }

   public static void toBullet(Tuple3DReadOnly euclidTuple, btVector3 bulletVector3)
   {
      bulletVector3.setValue(euclidTuple.getX32(), euclidTuple.getY32(), euclidTuple.getZ32());
   }

   public static void toEuclid(btVector3 bulletVector3, Vector3DBasics euclidVector3D32)
   {
      euclidVector3D32.set(bulletVector3.getX(), bulletVector3.getY(), bulletVector3.getZ());
   }

   public static void toEuclid(btVector3 bulletVector3, Point3DBasics euclidPoint3D32)
   {
      euclidPoint3D32.set(bulletVector3.getX(), bulletVector3.getY(), bulletVector3.getZ());
   }

   public static btTriangleMesh convertTriangleMesh3D(RigidBodyTransformReadOnly meshPose, TriangleMesh3DDefinition triangleMesh3DDefinition)
   {

      btTriangleMesh btTriangleMesh = new btTriangleMesh(false, false);

      int[] triangleIndices = triangleMesh3DDefinition.getTriangleIndices();
      Point3D32[] vertices = triangleMesh3DDefinition.getVertices();

      boolean ignorePose = meshPose == null || (!meshPose.hasRotation() && !meshPose.hasTranslation());
      
      Point3D32 v0 = new Point3D32();
      Point3D32 v1 = new Point3D32();
      Point3D32 v2 = new Point3D32();

      for (int i = 0; i < triangleIndices.length; i += 3)
      {
         if (ignorePose)
         {
            v0 = vertices[triangleIndices[i]];
            v1 = vertices[triangleIndices[i + 1]];
            v2 = vertices[triangleIndices[i + 2]];
         }
         else
         {
            meshPose.transform(vertices[triangleIndices[i]], v0);
            meshPose.transform(vertices[triangleIndices[i + 1]], v1);
            meshPose.transform(vertices[triangleIndices[i + 2]], v2);
         }

         btTriangleMesh.addTriangle(new btVector3(v0.getX32(), v0.getY32(), v0.getZ32()),
                                    new btVector3(v1.getX32(), v1.getY32(), v1.getZ32()),
                                    new btVector3(v2.getX32(), v2.getY32(), v2.getZ32()));
      }
      //btTriangleMesh.releaseOwnership();

      return btTriangleMesh;
   }

   public static btCollisionShape createBulletCollisionShape(CollisionShapeDefinition collisionShapeDefinition)
   {
      btCollisionShape btCollisionShape = null;

      if (collisionShapeDefinition.getGeometryDefinition() instanceof TriangleMesh3DDefinition)
      {
         TriangleMesh3DDefinition triangleMesh3DDefinition = (TriangleMesh3DDefinition) collisionShapeDefinition.getGeometryDefinition();

         btTransform identity = new btTransform();
         btTriangleMesh btTriangleMesh = convertTriangleMesh3D(collisionShapeDefinition.getOriginPose(), triangleMesh3DDefinition);

         if (collisionShapeDefinition.isConcave())
         {
            btGImpactMeshShape btGImpactMeshShape = new btGImpactMeshShape(btTriangleMesh);
            btGImpactMeshShape.updateBound();

            btCompoundFromGimpactShape compoundFromGimpactShape = new btCompoundFromGimpactShape();

            btGImpactMeshShape.setMargin(0.01f);
            compoundFromGimpactShape.addChildShape(identity, btGImpactMeshShape);

            btCollisionShape = compoundFromGimpactShape;
         }
         else
         {
            btConvexTriangleMeshShape btConvexTriangleMeshShape = new btConvexTriangleMeshShape(btTriangleMesh);

            btCompoundShape btCompoundShape = new btCompoundShape();

            btConvexTriangleMeshShape.setMargin(0.01f);
            btCompoundShape.addChildShape(identity, btConvexTriangleMeshShape);

            btCollisionShape = btCompoundShape;
         }
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Box3DDefinition)
      {
         Box3DDefinition boxGeometryDefinition = (Box3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btBoxShape boxShape = new btBoxShape(new btVector3((float) boxGeometryDefinition.getSizeX() / 2.0f,
                                                          (float) boxGeometryDefinition.getSizeY() / 2.0f,
                                                          (float) boxGeometryDefinition.getSizeZ() / 2.0f));
         btCollisionShape = boxShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Sphere3DDefinition)
      {
         Sphere3DDefinition sphereGeometryDefinition = (Sphere3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btSphereShape sphereShape = new btSphereShape((float) sphereGeometryDefinition.getRadius());
         btCollisionShape = sphereShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Cylinder3DDefinition)
      {
         Cylinder3DDefinition cylinderGeometryDefinition = (Cylinder3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btCylinderShapeZ cylinderShape = new btCylinderShapeZ(new btVector3((float) cylinderGeometryDefinition.getRadius(),
                                                                           (float) cylinderGeometryDefinition.getRadius(),
                                                                           (float) cylinderGeometryDefinition.getLength() / 2.0f));
         btCollisionShape = cylinderShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Cone3DDefinition)
      {
         Cone3DDefinition coneGeometryDefinition = (Cone3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btConeShapeZ coneShape = new btConeShapeZ((float) coneGeometryDefinition.getRadius(), (float) coneGeometryDefinition.getHeight());
         btCollisionShape = coneShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Capsule3DDefinition)
      {
         Capsule3DDefinition capsuleGeometryDefinition = (Capsule3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         if (capsuleGeometryDefinition.getRadiusX() != capsuleGeometryDefinition.getRadiusY()
               || capsuleGeometryDefinition.getRadiusX() != capsuleGeometryDefinition.getRadiusZ()
               || capsuleGeometryDefinition.getRadiusY() != capsuleGeometryDefinition.getRadiusZ())
            LogTools.warn("Bullet capsule does not fully represent the intended capsule!");
         btCapsuleShapeZ capsuleShape = new btCapsuleShapeZ((float) capsuleGeometryDefinition.getRadiusX(), (float) capsuleGeometryDefinition.getLength());
         btCollisionShape = capsuleShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof ConvexPolytope3DDefinition)
      {
         ConvexPolytope3DDefinition convexPolytopeDefinition = (ConvexPolytope3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btConvexHullShape convexHullShape = new btConvexHullShape();
         for (Face3DReadOnly face : convexPolytopeDefinition.getConvexPolytope().getFaces())
         {
            for (Vertex3DReadOnly vertex : face.getVertices())
            {
               convexHullShape.addPoint(new btVector3(vertex.getX32(), vertex.getY32(), vertex.getZ32()));
            }
         }
         btCollisionShape = convexHullShape;
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName());
      }

      return btCollisionShape;
   }
}

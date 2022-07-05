package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCapsuleShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundFromGimpactShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.collision.btConeShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btConvexHullShape;
import com.badlogic.gdx.physics.bullet.collision.btConvexTriangleMeshShape;
import com.badlogic.gdx.physics.bullet.collision.btCylinderShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btGImpactMeshShape;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import com.badlogic.gdx.physics.bullet.collision.btTriangleMesh;

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
   public static void toBullet(RigidBodyTransform rigidBodyTransform, Matrix4 bulletAffineToPack)
   {
      bulletAffineToPack.val[Matrix4.M00] = (float) rigidBodyTransform.getM00();
      bulletAffineToPack.val[Matrix4.M01] = (float) rigidBodyTransform.getM01();
      bulletAffineToPack.val[Matrix4.M02] = (float) rigidBodyTransform.getM02();
      bulletAffineToPack.val[Matrix4.M10] = (float) rigidBodyTransform.getM10();
      bulletAffineToPack.val[Matrix4.M11] = (float) rigidBodyTransform.getM11();
      bulletAffineToPack.val[Matrix4.M12] = (float) rigidBodyTransform.getM12();
      bulletAffineToPack.val[Matrix4.M20] = (float) rigidBodyTransform.getM20();
      bulletAffineToPack.val[Matrix4.M21] = (float) rigidBodyTransform.getM21();
      bulletAffineToPack.val[Matrix4.M22] = (float) rigidBodyTransform.getM22();
      bulletAffineToPack.val[Matrix4.M03] = (float) rigidBodyTransform.getM03();
      bulletAffineToPack.val[Matrix4.M13] = (float) rigidBodyTransform.getM13();
      bulletAffineToPack.val[Matrix4.M23] = (float) rigidBodyTransform.getM23();
   }

   public static void toEuclid(Matrix4 bulletAffine, RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.getRotation().setAndNormalize(bulletAffine.val[Matrix4.M00],
                                                       bulletAffine.val[Matrix4.M01],
                                                       bulletAffine.val[Matrix4.M02],
                                                       bulletAffine.val[Matrix4.M10],
                                                       bulletAffine.val[Matrix4.M11],
                                                       bulletAffine.val[Matrix4.M12],
                                                       bulletAffine.val[Matrix4.M20],
                                                       bulletAffine.val[Matrix4.M21],
                                                       bulletAffine.val[Matrix4.M22]);
      rigidBodyTransform.getTranslation().setX(bulletAffine.val[Matrix4.M03]);
      rigidBodyTransform.getTranslation().setY(bulletAffine.val[Matrix4.M13]);
      rigidBodyTransform.getTranslation().setZ(bulletAffine.val[Matrix4.M23]);
   }

   public static void toBullet(us.ihmc.euclid.tuple4D.Quaternion euclidQuaternion, Quaternion bulletQuaternion)
   {
      bulletQuaternion.x = euclidQuaternion.getX32();
      bulletQuaternion.y = euclidQuaternion.getY32();
      bulletQuaternion.z = euclidQuaternion.getZ32();
      bulletQuaternion.w = euclidQuaternion.getS32();
   }

   public static void toBullet(Tuple3DReadOnly euclidTuple, Vector3 bulletVector3)
   {
      bulletVector3.set(euclidTuple.getX32(), euclidTuple.getY32(), euclidTuple.getZ32());
   }

   public static void toEuclid(Vector3 bulletVector3, Vector3DBasics euclidVector3D32)
   {
      euclidVector3D32.set(bulletVector3.x, bulletVector3.y, bulletVector3.z);
   }

   public static void toEuclid(Vector3 bulletVector3, Point3DBasics euclidPoint3D32)
   {
      euclidPoint3D32.set(bulletVector3.x, bulletVector3.y, bulletVector3.z);
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

         btTriangleMesh.addTriangle(new Vector3(v0.getX32(), v0.getY32(), v0.getZ32()),
                                    new Vector3(v1.getX32(), v1.getY32(), v1.getZ32()),
                                    new Vector3(v2.getX32(), v2.getY32(), v2.getZ32()));
      }
      btTriangleMesh.releaseOwnership();

      return btTriangleMesh;
   }

   public static btCollisionShape createBulletCollisionShape(CollisionShapeDefinition collisionShapeDefinition)
   {
      btCollisionShape btCollisionShape = null;

      if (collisionShapeDefinition.getGeometryDefinition() instanceof TriangleMesh3DDefinition)
      {
         TriangleMesh3DDefinition triangleMesh3DDefinition = (TriangleMesh3DDefinition) collisionShapeDefinition.getGeometryDefinition();

         Matrix4 identity = new Matrix4();
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
         btBoxShape boxShape = new btBoxShape(new Vector3((float) boxGeometryDefinition.getSizeX() / 2.0f,
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
         btCylinderShapeZ cylinderShape = new btCylinderShapeZ(new Vector3((float) cylinderGeometryDefinition.getRadius(),
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
               convexHullShape.addPoint(new Vector3(vertex.getX32(), vertex.getY32(), vertex.getZ32()));
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

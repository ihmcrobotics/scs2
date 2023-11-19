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
import org.bytedeco.bullet.LinearMath.btQuaternion;
import org.bytedeco.bullet.LinearMath.btTransform;
import org.bytedeco.bullet.LinearMath.btVector3;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.*;

public class BulletTools
{
   public static void toBullet(RigidBodyTransform rigidBodyTransform, btTransform bulletAffineToPack)
   {
      bulletAffineToPack.getOrigin().setValue(rigidBodyTransform.getTranslationX(),
                                              rigidBodyTransform.getTranslationY(),
                                              rigidBodyTransform.getTranslationZ());
      bulletAffineToPack.getBasis().setValue(rigidBodyTransform.getM00(),
                                             rigidBodyTransform.getM01(),
                                             rigidBodyTransform.getM02(),
                                             rigidBodyTransform.getM10(),
                                             rigidBodyTransform.getM11(),
                                             rigidBodyTransform.getM12(),
                                             rigidBodyTransform.getM20(),
                                             rigidBodyTransform.getM21(),
                                             rigidBodyTransform.getM22());
   }

   public static void toEuclid(btTransform bulletAffine, RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.getRotation().setAndNormalize(bulletAffine.getBasis().getRow(0).getX(),
                                                       bulletAffine.getBasis().getRow(0).getY(),
                                                       bulletAffine.getBasis().getRow(0).getZ(),
                                                       bulletAffine.getBasis().getRow(1).getX(),
                                                       bulletAffine.getBasis().getRow(1).getY(),
                                                       bulletAffine.getBasis().getRow(1).getZ(),
                                                       bulletAffine.getBasis().getRow(2).getX(),
                                                       bulletAffine.getBasis().getRow(2).getY(),
                                                       bulletAffine.getBasis().getRow(2).getZ());
      rigidBodyTransform.getTranslation().setX(bulletAffine.getOrigin().getX());
      rigidBodyTransform.getTranslation().setY(bulletAffine.getOrigin().getY());
      rigidBodyTransform.getTranslation().setZ(bulletAffine.getOrigin().getZ());
   }

   public static void toBullet(Quaternion euclidQuaternion, btQuaternion bulletQuaternion)
   {
      bulletQuaternion.setValue(euclidQuaternion.getX(), euclidQuaternion.getY(), euclidQuaternion.getZ(), euclidQuaternion.getS());
   }

   public static void toEuclid(btQuaternion bulletQuaternion, Quaternion euclidQuaternion)
   {
      euclidQuaternion.set(bulletQuaternion.getX(), bulletQuaternion.getY(), bulletQuaternion.getZ(), bulletQuaternion.getW());
   }

   public static void toBullet(Tuple3DReadOnly euclidTuple, btVector3 bulletVector3)
   {
      bulletVector3.setValue(euclidTuple.getX(), euclidTuple.getY(), euclidTuple.getZ());
   }

   public static void toEuclid(btVector3 bulletVector3, Vector3DBasics euclidVector3D)
   {
      euclidVector3D.set(bulletVector3.getX(), bulletVector3.getY(), bulletVector3.getZ());
   }

   public static void toEuclid(btVector3 bulletVector3, Point3DBasics euclidPoint3D)
   {
      euclidPoint3D.set(bulletVector3.getX(), bulletVector3.getY(), bulletVector3.getZ());
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

         btTriangleMesh.addTriangle(new btVector3(v0.getX(), v0.getY(), v0.getZ()),
                                    new btVector3(v1.getX(), v1.getY(), v1.getZ()),
                                    new btVector3(v2.getX(), v2.getY(), v2.getZ()));
      }

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

            btGImpactMeshShape.setMargin(0.01);
            compoundFromGimpactShape.addChildShape(identity, btGImpactMeshShape);

            btCollisionShape = compoundFromGimpactShape;
         }
         else
         {
            btConvexTriangleMeshShape btConvexTriangleMeshShape = new btConvexTriangleMeshShape(btTriangleMesh);

            btCompoundShape btCompoundShape = new btCompoundShape();

            btConvexTriangleMeshShape.setMargin(0.01);
            btCompoundShape.addChildShape(identity, btConvexTriangleMeshShape);

            btCollisionShape = btCompoundShape;
         }
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Box3DDefinition)
      {
         Box3DDefinition boxGeometryDefinition = (Box3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btBoxShape boxShape = new btBoxShape(new btVector3(boxGeometryDefinition.getSizeX() / 2.0,
                                                            boxGeometryDefinition.getSizeY() / 2.0,
                                                            boxGeometryDefinition.getSizeZ() / 2.0));
         btCollisionShape = boxShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Sphere3DDefinition)
      {
         Sphere3DDefinition sphereGeometryDefinition = (Sphere3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btSphereShape sphereShape = new btSphereShape(sphereGeometryDefinition.getRadius());
         btCollisionShape = sphereShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Cylinder3DDefinition)
      {
         Cylinder3DDefinition cylinderGeometryDefinition = (Cylinder3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btCylinderShapeZ cylinderShape = new btCylinderShapeZ(new btVector3(cylinderGeometryDefinition.getRadius(),
                                                                             cylinderGeometryDefinition.getRadius(),
                                                                             cylinderGeometryDefinition.getLength() / 2.0));
         btCollisionShape = cylinderShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Cone3DDefinition)
      {
         Cone3DDefinition coneGeometryDefinition = (Cone3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btConeShapeZ coneShape = new btConeShapeZ(coneGeometryDefinition.getRadius(), coneGeometryDefinition.getHeight());
         btCollisionShape = coneShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Capsule3DDefinition)
      {
         Capsule3DDefinition capsuleGeometryDefinition = (Capsule3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         if (capsuleGeometryDefinition.getRadiusX() != capsuleGeometryDefinition.getRadiusY()
               || capsuleGeometryDefinition.getRadiusX() != capsuleGeometryDefinition.getRadiusZ()
               || capsuleGeometryDefinition.getRadiusY() != capsuleGeometryDefinition.getRadiusZ())
            LogTools.warn("Bullet capsule does not fully represent the intended capsule!");
         btCapsuleShapeZ capsuleShape = new btCapsuleShapeZ(capsuleGeometryDefinition.getRadiusX(), capsuleGeometryDefinition.getLength());
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
               convexHullShape.addPoint(new btVector3(vertex.getX(), vertex.getY(), vertex.getZ()));
            }
         }
         btCollisionShape = convexHullShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Ellipsoid3DDefinition)
      {
         Ellipsoid3DDefinition ellipsoidGeometryDefinition = (Ellipsoid3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btSphereShape ellipsoidShape = new btSphereShape(1.0f);
         // Scale the sphere to be an ellipsoid
         btVector3 scaling = new btVector3(ellipsoidGeometryDefinition.getRadiusX(),
                                           ellipsoidGeometryDefinition.getRadiusY(),
                                           ellipsoidGeometryDefinition.getRadiusZ());
         ellipsoidShape.setLocalScaling(scaling);
         btCollisionShape = ellipsoidShape;
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName());
      }

      return btCollisionShape;
   }

   public enum eFeatherstoneJointType
   {
      eRevolute, ePrismatic, eSpherical, ePlanar, eFixed, eInvalid
   }
}

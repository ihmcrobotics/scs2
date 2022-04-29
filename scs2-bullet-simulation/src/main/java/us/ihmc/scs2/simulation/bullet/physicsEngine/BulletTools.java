package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.List;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.simulation.bullet.physicsEngine.modelLoader.AssimpLoader;

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

   public static List<btConvexTriangleMeshShape> loadConvexTriangleMeshShapeFromFile(String modelFilePath)
   {
      List<List<Point3D32>> vertexLists = AssimpLoader.loadTriangleVertexPositionsAsList(modelFilePath);
      List<btConvexTriangleMeshShape> shapes = new ArrayList<>();

      for (List<Point3D32> vertexList : vertexLists)
      {
         btTriangleMesh bulletTriangleMesh = new btTriangleMesh(false, false);

         for (int i = 0; i < vertexList.size(); i += 3)
         {
            bulletTriangleMesh.addTriangle(new Vector3(vertexList.get(i).getX32(), vertexList.get(i).getY32(), vertexList.get(i).getZ32()),
                                           new Vector3(vertexList.get(i + 1).getX32(), vertexList.get(i + 1).getY32(), vertexList.get(i + 1).getZ32()),
                                           new Vector3(vertexList.get(i + 2).getX32(), vertexList.get(i + 2).getY32(), vertexList.get(i + 2).getZ32()));
         }
         bulletTriangleMesh.releaseOwnership();

         btConvexTriangleMeshShape bulletConvexTriangleMeshShape = new btConvexTriangleMeshShape(bulletTriangleMesh);

         shapes.add(bulletConvexTriangleMeshShape);
      }
      return shapes;
   }

   public static List<btGImpactMeshShape> loadConcaveGImpactMeshShapeFromFile(String modelFilePath)
   {
      List<List<Point3D32>> vertexLists = AssimpLoader.loadTriangleVertexPositionsAsList(modelFilePath);
      List<btGImpactMeshShape> shapes = new ArrayList<>();

      for (List<Point3D32> vertexList : vertexLists)
      {
         btTriangleMesh bulletTriangleMesh = new btTriangleMesh(false, false);

         for (int i = 0; i < vertexList.size(); i += 3)
         {
            bulletTriangleMesh.addTriangle(new Vector3(vertexList.get(i).getX32(), vertexList.get(i).getY32(), vertexList.get(i).getZ32()),
                                           new Vector3(vertexList.get(i + 1).getX32(), vertexList.get(i + 1).getY32(), vertexList.get(i + 1).getZ32()),
                                           new Vector3(vertexList.get(i + 2).getX32(), vertexList.get(i + 2).getY32(), vertexList.get(i + 2).getZ32()));
         }
         bulletTriangleMesh.releaseOwnership();

         btGImpactMeshShape bulletConvexTriangleMeshShape = new btGImpactMeshShape(bulletTriangleMesh);
         bulletConvexTriangleMeshShape.updateBound();

         shapes.add(bulletConvexTriangleMeshShape);
      }
      return shapes;
   }

   public static btCollisionShape createBulletCollisionShape(CollisionShapeDefinition collisionShapeDefinition)
   {
      btCollisionShape bulletCollisionShape = null;

      if (collisionShapeDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
      {
         ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) collisionShapeDefinition.getGeometryDefinition();

         Matrix4 identity = new Matrix4();
         if (collisionShapeDefinition.isConcave())
         {
            List<btGImpactMeshShape> shapes = BulletTools.loadConcaveGImpactMeshShapeFromFile(modelFileGeometryDefinition.getFileName());
            btCompoundFromGimpactShape compoundFromGimpactShape = new btCompoundFromGimpactShape();

            for (btCollisionShape shape : shapes)
            {
               shape.setMargin(0.01f);
               compoundFromGimpactShape.addChildShape(identity, shape);
            }

            bulletCollisionShape = compoundFromGimpactShape;
         }
         else
         {
            List<btConvexTriangleMeshShape> shapes = BulletTools.loadConvexTriangleMeshShapeFromFile(modelFileGeometryDefinition.getFileName());

            btCompoundShape compoundShape = new btCompoundShape();

            for (btCollisionShape shape : shapes)
            {
               shape.setMargin(0.01f);
               compoundShape.addChildShape(identity, shape);
            }

            bulletCollisionShape = compoundShape;
         }
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Box3DDefinition)
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
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Cone3DDefinition)
      {
         Cone3DDefinition coneGeometryDefinition = (Cone3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btConeShapeZ coneShape = new btConeShapeZ((float) coneGeometryDefinition.getRadius(), (float) coneGeometryDefinition.getHeight());
         bulletCollisionShape = coneShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Capsule3DDefinition)
      {
         Capsule3DDefinition capsuleGeometryDefinition = (Capsule3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         if (capsuleGeometryDefinition.getRadiusX() != capsuleGeometryDefinition.getRadiusY()
               || capsuleGeometryDefinition.getRadiusX() != capsuleGeometryDefinition.getRadiusZ()
               || capsuleGeometryDefinition.getRadiusY() != capsuleGeometryDefinition.getRadiusZ())
            LogTools.warn("Bullet capsule does not fully represent the intended capsule!");
         btCapsuleShapeZ capsuleShape = new btCapsuleShapeZ((float) capsuleGeometryDefinition.getRadiusX(), (float) capsuleGeometryDefinition.getLength());
         bulletCollisionShape = capsuleShape;
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName());
      }

      return bulletCollisionShape;
   }
}

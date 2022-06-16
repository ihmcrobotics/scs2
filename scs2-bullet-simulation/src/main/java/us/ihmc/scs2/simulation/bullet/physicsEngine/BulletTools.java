package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.List;
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
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.simulation.bullet.physicsEngine.modelLoader.AssimpLoader;

public class BulletTools
{
   
   public static void toBullet(RigidBodyTransform rigidBodyTransform, btTransform bulletAffineToPack)
   {
      bulletAffineToPack.getOrigin().setValue((float) rigidBodyTransform.getTranslationX(), (float) rigidBodyTransform.getTranslationY(), (float) rigidBodyTransform.getTranslationZ());
      bulletAffineToPack.getBasis().setValue((float) rigidBodyTransform.getM00(),
                                             (float) rigidBodyTransform.getM01(),
                                             (float) rigidBodyTransform.getM02(),
                                             (float) rigidBodyTransform.getM10(),
                                             (float) rigidBodyTransform.getM11(),
                                             (float) rigidBodyTransform.getM12(),
                                             (float) rigidBodyTransform.getM20(),
                                             (float) rigidBodyTransform.getM21(),
                                             (float) rigidBodyTransform.getM22());
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

   public static List<btConvexTriangleMeshShape> loadConvexTriangleMeshShapeFromFile(String modelFilePath)
   {
      List<List<Point3D32>> vertexLists = AssimpLoader.loadTriangleVertexPositionsAsList(modelFilePath);
      List<btConvexTriangleMeshShape> shapes = new ArrayList<>();

      for (List<Point3D32> vertexList : vertexLists)
      {
         btTriangleMesh bulletTriangleMesh = new btTriangleMesh(false, false);

         for (int i = 0; i < vertexList.size(); i += 3)
         {
            bulletTriangleMesh.addTriangle(new btVector3(vertexList.get(i).getX32(), vertexList.get(i).getY32(), vertexList.get(i).getZ32()),
                                           new btVector3(vertexList.get(i + 1).getX32(), vertexList.get(i + 1).getY32(), vertexList.get(i + 1).getZ32()),
                                           new btVector3(vertexList.get(i + 2).getX32(), vertexList.get(i + 2).getY32(), vertexList.get(i + 2).getZ32()));
         }
         //bulletTriangleMesh.releaseOwnership();

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
            bulletTriangleMesh.addTriangle(new btVector3(vertexList.get(i).getX32(), vertexList.get(i).getY32(), vertexList.get(i).getZ32()),
                                           new btVector3(vertexList.get(i + 1).getX32(), vertexList.get(i + 1).getY32(), vertexList.get(i + 1).getZ32()),
                                           new btVector3(vertexList.get(i + 2).getX32(), vertexList.get(i + 2).getY32(), vertexList.get(i + 2).getZ32()));
         }
         //bulletTriangleMesh.releaseOwnership();

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

         btTransform identity = new btTransform();
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
         btBoxShape boxShape = new btBoxShape(new btVector3((float) boxGeometryDefinition.getSizeX() / 2.0f,
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
         btCylinderShapeZ cylinderShape = new btCylinderShapeZ(new btVector3((float) cylinderGeometryDefinition.getRadius(),
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
         bulletCollisionShape = convexHullShape;
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName());
      }

      return bulletCollisionShape;
   }
   
   public enum BroadphaseNativeTypes
   {
      // polyhedral convex shapes
      BOX_SHAPE_PROXYTYPE,
      TRIANGLE_SHAPE_PROXYTYPE,
      TETRAHEDRAL_SHAPE_PROXYTYPE,
      CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
      CONVEX_HULL_SHAPE_PROXYTYPE,
      CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
      CUSTOM_POLYHEDRAL_SHAPE_TYPE,
      //implicit convex shapes
      IMPLICIT_CONVEX_SHAPES_START_HERE,
      SPHERE_SHAPE_PROXYTYPE,
      MULTI_SPHERE_SHAPE_PROXYTYPE,
      CAPSULE_SHAPE_PROXYTYPE,
      CONE_SHAPE_PROXYTYPE,
      CONVEX_SHAPE_PROXYTYPE,
      CYLINDER_SHAPE_PROXYTYPE,
      UNIFORM_SCALING_SHAPE_PROXYTYPE,
      MINKOWSKI_SUM_SHAPE_PROXYTYPE,
      MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
      BOX_2D_SHAPE_PROXYTYPE,
      CONVEX_2D_SHAPE_PROXYTYPE,
      CUSTOM_CONVEX_SHAPE_TYPE,
      //concave shapes
      CONCAVE_SHAPES_START_HERE,
      //keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
      TRIANGLE_MESH_SHAPE_PROXYTYPE,
      SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
      ///used for demo integration FAST/Swift collision library and Bullet
      FAST_CONCAVE_MESH_PROXYTYPE,
      //terrain
      TERRAIN_SHAPE_PROXYTYPE,
      ///Used for GIMPACT Trimesh integration
      GIMPACT_SHAPE_PROXYTYPE,
      ///Multimaterial mesh
      MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,

      EMPTY_SHAPE_PROXYTYPE,
      STATIC_PLANE_PROXYTYPE,
      CUSTOM_CONCAVE_SHAPE_TYPE,
      CONCAVE_SHAPES_END_HERE,

      COMPOUND_SHAPE_PROXYTYPE,

      SOFTBODY_SHAPE_PROXYTYPE,
      HFFLUID_SHAPE_PROXYTYPE,
      HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
      INVALID_SHAPE_PROXYTYPE,

      MAX_BROADPHASE_COLLISION_TYPES,
      
      NOT_DEFINED_TYPE
   };
   
   public enum eFeatherstoneJointType
   {
      eRevolute,
      ePrismatic,
      eSpherical,
      ePlanar,
      eFixed,
      eInvalid
   };
}

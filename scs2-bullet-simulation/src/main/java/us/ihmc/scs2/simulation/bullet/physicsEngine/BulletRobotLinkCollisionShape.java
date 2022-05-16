package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.List;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.*;

public class BulletRobotLinkCollisionShape
{
   private final YawPitchRollTransformDefinition collisionShapeToFrameAfterParentJoint;
   private final ReferenceFrame linkCenterOfMassFrame;
   private final Matrix4 collisionShapeDefinitionToCenterOfMassFrameTransformGDX = new Matrix4();
   private final RigidBodyTransform collisionShapeDefinitionToCenterOfMassFrameTransformEuclid = new RigidBodyTransform();
   private final ReferenceFrame collisionShapeDefinitionFrame;
   private btCollisionShape bulletCollisionShape;
   private final GeometryDefinition geometryDefinition;

   public BulletRobotLinkCollisionShape(CollisionShapeDefinition collisionShapeDefinition,
                                        ReferenceFrame frameAfterParentJoint,
                                        ReferenceFrame linkCenterOfMassFrame)
   {
      this.linkCenterOfMassFrame = linkCenterOfMassFrame;

      collisionShapeToFrameAfterParentJoint = collisionShapeDefinition.getOriginPose();
      collisionShapeDefinitionFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(frameAfterParentJoint,
                                                                                                               new RigidBodyTransform(collisionShapeToFrameAfterParentJoint.getRotation(),
                                                                                                                                      collisionShapeToFrameAfterParentJoint.getTranslation()));

      this.geometryDefinition = collisionShapeDefinition.getGeometryDefinition();
      // Just need to make sure the vertices for the libGDX shapes and the bullet shapes are the same
      //Color color = new Color(Color.WHITE);
      // TODO: Get to this later for the fingers
      if (collisionShapeDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
      {
         ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) collisionShapeDefinition.getGeometryDefinition();
//         List<btConvexHullShape> shapes = BulletTools.loadConvexHullShapeFromFile(modelFileGeometryDefinition.getFileName());
//         List<btConvexPointCloudShape> shapes = BulletTools.loadConvexPointCloudShapesFromFile(modelFileGeometryDefinition.getFileName());

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
               convexHullShape.addPoint(new Vector3(vertex.getX32(), vertex.getY32(), vertex.getZ32()));
            }
         }
         bulletCollisionShape = convexHullShape;
      }
      else
      {
         LogTools.warn("Implement collision for {}", collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName());
      }
   }

   public void addToCompoundShape(btCompoundShape bulletCompoundShape)
   {
      collisionShapeDefinitionFrame.getTransformToDesiredFrame(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, linkCenterOfMassFrame);
      BulletTools.toBullet(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, collisionShapeDefinitionToCenterOfMassFrameTransformGDX);
      if (bulletCollisionShape == null)
         throw new RuntimeException("Collision shape is null! Preventing native crash.");
      bulletCompoundShape.addChildShape(collisionShapeDefinitionToCenterOfMassFrameTransformGDX, bulletCollisionShape);
   }

   public btCollisionShape getBulletCollisionShape()
   {
      return bulletCollisionShape;
   }

}

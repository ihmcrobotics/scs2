package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.collision.btCylinderShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;

public class BulletRobotLinkCollisionShape
{
   private final YawPitchRollTransformDefinition collisionShapeToFrameAfterParentJoint;
   private final ReferenceFrame linkCenterOfMassFrame;
   private final Matrix4 collisionShapeDefinitionToCenterOfMassFrameTransformGDX = new Matrix4();
   private final RigidBodyTransform collisionShapeDefinitionToCenterOfMassFrameTransformEuclid = new RigidBodyTransform();
   private final ReferenceFrame collisionShapeDefinitionFrame;
   private btCollisionShape bulletCollisionShape;

   public BulletRobotLinkCollisionShape(CollisionShapeDefinition collisionShapeDefinition,
                                        ReferenceFrame frameAfterParentJoint,
                                        ReferenceFrame linkCenterOfMassFrame)
   {
      this.linkCenterOfMassFrame = linkCenterOfMassFrame;

      collisionShapeToFrameAfterParentJoint = collisionShapeDefinition.getOriginPose();
      collisionShapeDefinitionFrame
            = ReferenceFrameMissingTools
            .constructFrameWithUnchangingTransformToParent(frameAfterParentJoint,
                                                           new RigidBodyTransform(collisionShapeToFrameAfterParentJoint.getRotation(),
                                                                                  collisionShapeToFrameAfterParentJoint.getTranslation()));

      // Just need to make sure the vertices for the libGDX shapes and the bullet shapes are the same
      //Color color = new Color(Color.WHITE);
      // TODO: Get to this later for the fingers
      //            if (collisionShapeDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
      //            {
      //               ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) collisionShapeDefinition.getGeometryDefinition();
      //               btConvexHullShape convexHullShape = BulletTools.createConcaveHullShapeFromMesh(collisionModel.meshParts.get(0).mesh);
      //               convexHullShape.setMargin(0.01f);
      //               bulletCollisionShape = convexHullShape;
      //            }
      if (collisionShapeDefinition.getGeometryDefinition() instanceof Box3DDefinition)
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
      else
      {
         LogTools.warn("Implement collision for {}", collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName());
      }
   }

   public void addToCompoundShape(btCompoundShape bulletCompoundShape)
   {
      collisionShapeDefinitionFrame.getTransformToDesiredFrame(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, linkCenterOfMassFrame);
      BulletTools.toBullet(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, collisionShapeDefinitionToCenterOfMassFrameTransformGDX);
      bulletCompoundShape.addChildShape(collisionShapeDefinitionToCenterOfMassFrameTransformGDX, bulletCollisionShape);
   }

   public btCollisionShape getBulletCollisionShape()
   {
      return bulletCollisionShape;
   }

}

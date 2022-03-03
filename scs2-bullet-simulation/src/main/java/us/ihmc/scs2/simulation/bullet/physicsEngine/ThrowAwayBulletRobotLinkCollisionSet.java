package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;

import java.util.ArrayList;
import java.util.List;

public class ThrowAwayBulletRobotLinkCollisionSet
{
   private final btCompoundShape bulletCompoundShape;
   private final ArrayList<ThrowAwayBulletRobotLinkCollisionShape> collisionShapes = new ArrayList<>();

   public ThrowAwayBulletRobotLinkCollisionSet(List<CollisionShapeDefinition> collisionShapeDefinitions,
                                         ReferenceFrame frameAfterParentJoint,
                                         ReferenceFrame linkCenterOfMassFrame)
   {
      bulletCompoundShape = new btCompoundShape();

      for (CollisionShapeDefinition shapeDefinition : collisionShapeDefinitions)
      {
         ThrowAwayBulletRobotLinkCollisionShape collisionShape = new ThrowAwayBulletRobotLinkCollisionShape(shapeDefinition, frameAfterParentJoint, linkCenterOfMassFrame);
         collisionShape.addToCompoundShape(bulletCompoundShape);
         collisionShapes.add(collisionShape);
      }
   }

   public void updateFromMecanoRigidBody()
   {
      for (ThrowAwayBulletRobotLinkCollisionShape collisionShape : collisionShapes)
      {
         collisionShape.updateTransforms();
      }
   }

   public btCompoundShape getBulletCompoundShape()
   {
      return bulletCompoundShape;
   }

   public ArrayList<ThrowAwayBulletRobotLinkCollisionShape> getCollisionShapes()
   {
      return collisionShapes;
   }
}

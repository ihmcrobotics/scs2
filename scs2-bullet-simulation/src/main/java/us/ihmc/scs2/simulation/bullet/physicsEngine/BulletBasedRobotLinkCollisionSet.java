package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.List;

import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;

public class BulletBasedRobotLinkCollisionSet
{
   private final btCompoundShape bulletCompoundShape;
   private final ArrayList<BulletBasedRobotLinkCollisionShape> collisionShapes = new ArrayList<>();

   public BulletBasedRobotLinkCollisionSet(List<CollisionShapeDefinition> collisionShapeDefinitions,
                                           ReferenceFrame frameAfterParentJoint,
                                           ReferenceFrame linkCenterOfMassFrame)
     {
        bulletCompoundShape = new btCompoundShape();

        for (CollisionShapeDefinition shapeDefinition : collisionShapeDefinitions)
        {
           BulletBasedRobotLinkCollisionShape collisionShape = new BulletBasedRobotLinkCollisionShape(shapeDefinition,
                                                                                                      frameAfterParentJoint,
                                                                                                      linkCenterOfMassFrame);
           collisionShape.addToCompoundShape(bulletCompoundShape);
           collisionShapes.add(collisionShape);
        }
     }

   public btCompoundShape getBulletCompoundShape()
   {
      return bulletCompoundShape;
   }

   public ArrayList<BulletBasedRobotLinkCollisionShape> getCollisionShapes()
   {
      return collisionShapes;
   }

}

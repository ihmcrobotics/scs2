package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.List;

import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;

public class BulletRobotLinkCollisionSet
{
   private final btCompoundShape bulletCompoundShape;
   private final ArrayList<BulletRobotLinkCollisionShape> collisionShapes = new ArrayList<>();

   public BulletRobotLinkCollisionSet(List<CollisionShapeDefinition> collisionShapeDefinitions,
                                      ReferenceFrame frameAfterParentJoint,
                                      ReferenceFrame linkCenterOfMassFrame)
     {
        bulletCompoundShape = new btCompoundShape();

        for (CollisionShapeDefinition shapeDefinition : collisionShapeDefinitions)
        {
           BulletRobotLinkCollisionShape collisionShape = new BulletRobotLinkCollisionShape(shapeDefinition,
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

   public ArrayList<BulletRobotLinkCollisionShape> getCollisionShapes()
   {
      return collisionShapes;
   }

}

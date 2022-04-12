package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.List;

import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;

public class AltBulletRobotLinkCollisionSet
{
   private final btCompoundShape bulletCompoundShape;
   private final ArrayList<AltBulletRobotLinkCollisionShape> collisionShapes = new ArrayList<>();

   public AltBulletRobotLinkCollisionSet(List<CollisionShapeDefinition> collisionShapeDefinitions,
                                      ReferenceFrame frameAfterParentJoint,
                                      ReferenceFrame linkCenterOfMassFrame)
     {
        bulletCompoundShape = new btCompoundShape();
        
        for (CollisionShapeDefinition shapeDefinition : collisionShapeDefinitions)
        {
           AltBulletRobotLinkCollisionShape collisionShape = new AltBulletRobotLinkCollisionShape(shapeDefinition,
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

   public ArrayList<AltBulletRobotLinkCollisionShape> getCollisionShapes()
   {
      return collisionShapes;
   }

}

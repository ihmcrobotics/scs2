package us.ihmc.scs2.examples.ball;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.BoxGeometryDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition.MaterialDefinition;

public class SlopeGroundDefinition extends TerrainObjectDefinition
{
   public SlopeGroundDefinition()
   {
      this(Math.toRadians(15.0));
   }

   public SlopeGroundDefinition(double slopeAngle)
   {
      super();
      setName("Ground");
      RigidBodyTransform originPose = new RigidBodyTransform();
      originPose.getRotation().setToPitchOrientation(slopeAngle);
      originPose.appendTranslation(0.0, 0.0, -0.25);

      GeometryDefinition groundGeometryDefinition = new BoxGeometryDefinition(100.0, 100.0, 0.50);
      addVisualDefinition(new VisualDefinition(originPose, groundGeometryDefinition, new MaterialDefinition(ColorDefinitions.DeepSkyBlue())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(originPose, groundGeometryDefinition));
   }
}

package us.ihmc.scs2.definition.terrain;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class FlatGroundDefinition extends TerrainObjectDefinition
{
   public FlatGroundDefinition()
   {
      this(ColorDefinitions.DeepSkyBlue());
   }

   public FlatGroundDefinition(ColorDefinition colorDefinition)
   {
      this(new MaterialDefinition(colorDefinition));
   }

   public FlatGroundDefinition(MaterialDefinition materialDefinition)
   {
      super();

      RigidBodyTransform originPose = new RigidBodyTransform();
      originPose.appendTranslation(0.0, 0.0, -0.25);

      GeometryDefinition groundGeometryDefinition = new Box3DDefinition(10000.0, 10000.0, 0.50);
      addVisualDefinition(new VisualDefinition(originPose, groundGeometryDefinition, materialDefinition));
      addCollisionShapeDefinition(new CollisionShapeDefinition(originPose, groundGeometryDefinition));
   }
}

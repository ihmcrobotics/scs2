package us.ihmc.scs2.definition.terrain;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.visual.HeightMapDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public abstract class HeightMapGroundDefinition extends TerrainObjectDefinition
{
   public HeightMapGroundDefinition(HeightMapDefinition heightMapDefinition)
   {
      TriangleMesh3DDefinition meshDefinition = TriangleMesh3DFactories.HeightMap(heightMapDefinition);

      // TODO generate the mesh from the height map

      addVisualDefinition(new VisualDefinition(new Pose3D(), meshDefinition, ColorDefinitions.Grey()));
      addCollisionShapeDefinition(new CollisionShapeDefinition(new Pose3D(), meshDefinition));
   }
}

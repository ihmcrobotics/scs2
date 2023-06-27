package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class TransparencyTest
{
   public static void main(String[] args)
   {
      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2("transparencyTest");

      ColorDefinition partiallyTransparentGreen = ColorDefinitions.DarkGreen();
      partiallyTransparentGreen.setAlpha(0.0);

      ColorDefinition partiallyTransparentGray = ColorDefinitions.Gray();
      partiallyTransparentGray.setAlpha(0.5);

      Sphere3DDefinition sphereGeometry = new Sphere3DDefinition(1.0);
      Box3DDefinition boxGeometry = new Box3DDefinition(5.0, 5.0, 0.02);

      scs2.addTerrainObject(new TerrainObjectDefinition(new VisualDefinition(sphereGeometry, partiallyTransparentGreen),
                                                        new CollisionShapeDefinition(sphereGeometry)));
      scs2.addTerrainObject(new TerrainObjectDefinition(new VisualDefinition(boxGeometry, partiallyTransparentGray),
                                                        new CollisionShapeDefinition(boxGeometry)));

      scs2.start(true, true, true);
   }
}
package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

/**
 * Emits MJCF {@code <geom>} fragments for SCS2 {@link TerrainObjectDefinition}s. Returned strings
 * are dropped into the {@code <worldbody>} block of the composite MJCF assembled by
 * {@link MujocoMultiBodyRobotFactory}.
 *
 * <p>v1 covers the primitive shapes used by every existing SCS2 example. Mesh/convex polytope
 * terrain is left as TODO because it requires writing the mesh to a sidecar STL/OBJ and adding an
 * {@code <asset>} block; not on the v1 path.
 */
public final class MujocoTerrainFactory
{
   private static final int TERRAIN_INDENT = 2;

   private MujocoTerrainFactory()
   {
   }

   public static String toMjcfWorldbodyFragment(TerrainObjectDefinition terrainObjectDefinition)
   {
      StringBuilder sb = new StringBuilder();
      int i = 0;
      for (CollisionShapeDefinition shape : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         MujocoTools.appendGeom(sb, "terrain", "terrain_" + (i++), shape, TERRAIN_INDENT);
      }
      return sb.toString();
   }
}

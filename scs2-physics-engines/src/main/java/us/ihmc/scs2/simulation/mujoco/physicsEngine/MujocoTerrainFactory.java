package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

/**
 * Emits MJCF {@code <geom>} fragments for SCS2 {@link TerrainObjectDefinition}s. Returned strings
 * are dropped into the {@code <worldbody>} block of the composite MJCF assembled by
 * {@link MujocoMultiBodyRobotFactory}.
 *
 * <p>Primitive shapes (box / sphere / cylinder / capsule) are emitted directly. Convex polytopes and
 * ramps -- which the standard avatar test environments use for the ground and obstacles -- are
 * emitted as MuJoCo meshes: {@link #toMjcfAssetFragment} declares one inline {@code <mesh>} per such
 * shape and {@link #toMjcfWorldbodyFragment} emits the matching {@code type="mesh"} geom.
 */
public final class MujocoTerrainFactory
{
   private static final int TERRAIN_INDENT = 2;
   private static final int ASSET_INDENT = 2;

   private MujocoTerrainFactory()
   {
   }

   /**
    * Emit the {@code <worldbody>} geom fragments for one terrain object. {@code namePrefix} must be
    * unique per terrain object so geom (and paired mesh) names don't collide across objects.
    */
   public static String toMjcfWorldbodyFragment(TerrainObjectDefinition terrainObjectDefinition, String namePrefix)
   {
      StringBuilder sb = new StringBuilder();
      int i = 0;
      for (CollisionShapeDefinition shape : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         MujocoTools.appendGeom(sb, "terrain", namePrefix + (i++), shape, TERRAIN_INDENT);
      }
      return sb.toString();
   }

   /**
    * Emit the {@code <asset>} {@code <mesh>} entries for the mesh-backed shapes (convex polytopes,
    * ramps) of one terrain object. Uses the same {@code namePrefix}/index scheme as
    * {@link #toMjcfWorldbodyFragment} so meshes pair with their geoms. Returns "" if the object has
    * no mesh-backed shapes.
    */
   public static String toMjcfAssetFragment(TerrainObjectDefinition terrainObjectDefinition, String namePrefix)
   {
      StringBuilder sb = new StringBuilder();
      int i = 0;
      for (CollisionShapeDefinition shape : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         MujocoTools.appendMeshAsset(sb, namePrefix + (i++), shape, ASSET_INDENT);
      }
      return sb.toString();
   }
}

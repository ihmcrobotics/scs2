package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.terrain.FlatGroundDefinition;
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

      if (terrainObjectDefinition instanceof FlatGroundDefinition)
      {
         appendGroundPlane(sb, namePrefix + "0");
         return sb.toString();
      }

      int i = 0;
      for (CollisionShapeDefinition shape : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         MujocoTools.appendGeom(sb, "terrain", namePrefix + (i++), shape, TERRAIN_INDENT);
      }
      return sb.toString();
   }

   /**
    * Emit a {@link FlatGroundDefinition} as MuJoCo's {@code plane} primitive rather than the
    * 10 km box the definition is built from.
    *
    * <p>A plane has exact, cheap collision and no edge anywhere, and it keeps {@code mjModel.stat.extent}
    * sane rather than scaled by a box the size of a city. The box's top face sits at z = 0 -- the
    * definition offsets a 0.5 m thick slab down by 0.25 m -- so the plane goes at the origin.
    *
    * <p>Worth knowing: the avatar simulations do not reach this today. They build terrain through
    * {@code TerrainObjectDefinitionTools}, which produces a plain {@code TerrainObjectDefinition}
    * of boxes and never a {@code FlatGroundDefinition}. Pointing them at it would change the
    * terrain geometry for every engine, not just MuJoCo, so it is deliberately a separate decision.
    */
   private static void appendGroundPlane(StringBuilder sb, String name)
   {
      // Plane size is (x half-extent, y half-extent, grid spacing) for rendering only; zeros mean
      // the collision surface is infinite.
      sb.append("  ".repeat(TERRAIN_INDENT))
        .append("<geom class=\"terrain\" name=\"").append(name).append("\" type=\"plane\" size=\"0 0 1\"/>\n");
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
      if (terrainObjectDefinition instanceof FlatGroundDefinition)
         return ""; // A plane is a primitive; it has no mesh asset.

      int i = 0;
      for (CollisionShapeDefinition shape : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         MujocoTools.appendMeshAsset(sb, namePrefix + (i++), shape, ASSET_INDENT);
      }
      return sb.toString();
   }
}

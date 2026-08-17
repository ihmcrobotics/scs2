package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

/**
 * Lightweight reference to a piece of static terrain in the compiled MuJoCo model.
 *
 * <p>Unlike Bullet, MuJoCo doesn't expose a long-lived "terrain object" handle: terrain geoms live
 * inside `mjModel` and are addressed by integer id. We keep this class for symmetry with
 * {@code BulletTerrainObject} and for future extensions (e.g., moving terrain via mjData.xpos
 * overrides for kinematic objects).
 */
public class MujocoTerrainObject
{
   private final TerrainObjectDefinition definition;

   public MujocoTerrainObject(TerrainObjectDefinition definition)
   {
      this.definition = definition;
   }

   public TerrainObjectDefinition getDefinition()
   {
      return definition;
   }
}

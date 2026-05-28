package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
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
   private MujocoTerrainFactory()
   {
   }

   public static String toMjcfWorldbodyFragment(TerrainObjectDefinition terrainObjectDefinition)
   {
      StringBuilder sb = new StringBuilder();
      int i = 0;
      for (CollisionShapeDefinition shape : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         appendGeom(sb, "terrain_" + (i++), shape);
      }
      return sb.toString();
   }

   private static void appendGeom(StringBuilder sb, String name, CollisionShapeDefinition shape)
   {
      GeometryDefinition geometry = shape.getGeometryDefinition();
      RigidBodyTransformReadOnly pose = shape.getOriginPose();

      sb.append("    <geom class=\"terrain\" name=\"").append(name).append("\" ");
      sb.append(MujocoTools.toPosQuatAttributes(pose));
      sb.append(' ');

      if (geometry instanceof Box3DDefinition box)
      {
         // MuJoCo box size is half-extents.
         sb.append("type=\"box\" size=\"")
           .append(box.getSizeX() / 2.0).append(' ')
           .append(box.getSizeY() / 2.0).append(' ')
           .append(box.getSizeZ() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Sphere3DDefinition sphere)
      {
         sb.append("type=\"sphere\" size=\"").append(sphere.getRadius()).append("\"/>\n");
      }
      else if (geometry instanceof Cylinder3DDefinition cylinder)
      {
         // MuJoCo cylinder size: (radius, half-length).
         sb.append("type=\"cylinder\" size=\"")
           .append(cylinder.getRadius()).append(' ')
           .append(cylinder.getLength() / 2.0)
           .append("\"/>\n");
      }
      else if (geometry instanceof Capsule3DDefinition capsule)
      {
         // MuJoCo capsule size: (radius, half-length-of-cylinder-section).
         sb.append("type=\"capsule\" size=\"")
           .append(capsule.getRadiusX()).append(' ')
           .append(capsule.getLength() / 2.0)
           .append("\"/>\n");
      }
      else
      {
         // TODO: meshes / convex polytope require <asset><mesh .../></asset> entries. Skipping
         // in v1 emits a comment so the failure is loud at MJCF compile time.
         sb.append("/><!-- TODO unsupported geometry: ").append(geometry.getClass().getSimpleName()).append(" -->\n");
      }
   }
}

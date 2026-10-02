package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParametersReadOnly;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoCone;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoIntegrator;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoJacobian;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSolver;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;

/**
 * Pure-Java MJCF generation helpers used by the MuJoCo SCS2 integration. No JNI in this class.
 */
public final class MujocoTools
{
   private MujocoTools()
   {
   }

   /**
    * MuJoCo XML uses {@code pos="x y z"} and {@code quat="w x y z"} (w-first). Converts a
    * {@link RigidBodyTransformReadOnly} into the MJCF attribute fragment
    * {@code "pos=\"x y z\" quat=\"w x y z\""}.
    */
   public static String toPosQuatAttributes(RigidBodyTransformReadOnly transform)
   {
      return toPosQuatAttributes(transform.getTranslation(), transform.getRotation());
   }

   /**
    * Same as {@link #toPosQuatAttributes(RigidBodyTransformReadOnly)} but takes the translation
    * and orientation separately. Accepts any {@link Orientation3DReadOnly} (quaternion, rotation
    * matrix, or axis-angle); a temporary {@link Quaternion} is used to pull out the (w, x, y, z)
    * components in the MuJoCo order.
    */
   public static String toPosQuatAttributes(Tuple3DReadOnly translation, Orientation3DReadOnly rotation)
   {
      Quaternion q = new Quaternion();
      q.set(rotation);
      return new StringBuilder()
         .append("pos=\"")
         .append(translation.getX()).append(' ')
         .append(translation.getY()).append(' ')
         .append(translation.getZ()).append("\" quat=\"")
         .append(q.getS()).append(' ')
         .append(q.getX()).append(' ')
         .append(q.getY()).append(' ')
         .append(q.getZ()).append('"')
         .toString();
   }

   /**
    * Emit a {@code <geom>} for one collision shape under the given collision class ("robot" or
    * "terrain"). Supports box / sphere / cylinder / capsule / ellipsoid directly. Convex polytopes and ramps are
    * emitted as {@code type="mesh"} referencing a {@code <mesh>} that {@link #appendMeshAsset} must
    * have declared under the same {@code name} (MuJoCo builds the convex hull from the vertex cloud).
    * Any other geometry emits a TODO comment instead.
    */
   static void appendGeom(StringBuilder sb, String geomClass, String name, CollisionShapeDefinition shape, int indent)
   {
      String pad = "  ".repeat(indent);
      GeometryDefinition geometry = shape.getGeometryDefinition();
      sb.append(pad).append("<geom class=\"").append(geomClass).append("\" name=\"").append(name).append('"');
      if (!isIdentity(shape.getOriginPose()))
         sb.append(' ').append(toPosQuatAttributes(shape.getOriginPose()));

      if (geometry instanceof Box3DDefinition box)
      {
         // MuJoCo box size is half-extents.
         sb.append(" type=\"box\" size=\"")
           .append(box.getSizeX() / 2.0).append(' ')
           .append(box.getSizeY() / 2.0).append(' ')
           .append(box.getSizeZ() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Sphere3DDefinition sphere)
      {
         sb.append(" type=\"sphere\" size=\"").append(sphere.getRadius()).append("\"/>\n");
      }
      else if (geometry instanceof Cylinder3DDefinition cylinder)
      {
         // MuJoCo cylinder size: (radius, half-length).
         sb.append(" type=\"cylinder\" size=\"")
           .append(cylinder.getRadius()).append(' ')
           .append(cylinder.getLength() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Capsule3DDefinition capsule)
      {
         // MuJoCo capsule size: (radius, half-length-of-cylinder-section).
         sb.append(" type=\"capsule\" size=\"")
           .append(capsule.getRadiusX()).append(' ')
           .append(capsule.getLength() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Ellipsoid3DDefinition ellipsoid)
      {
         // MuJoCo ellipsoid size is the three semi-axis radii.
         sb.append(" type=\"ellipsoid\" size=\"")
           .append(ellipsoid.getRadiusX()).append(' ')
           .append(ellipsoid.getRadiusY()).append(' ')
           .append(ellipsoid.getRadiusZ()).append("\"/>\n");
      }
      else if (geometry instanceof ModelFileGeometryDefinition modelFile)
      {
         if (isSupportedModelFileMesh(modelFile))
         {
            // Mesh assets carry no size; the geom just references the <mesh> declared by appendMeshAsset.
            sb.append(" type=\"mesh\" mesh=\"").append(meshName(name)).append("\"/>\n");
         }
         else
         {
            sb.append("/><!-- skipped unsupported model file: ").append(modelFile.getFileName()).append(" -->\n");
         }
      }
      else if (isMeshGeometry(geometry))
      {
         // Mesh assets carry no size; the geom just references the <mesh> declared by appendMeshAsset.
         sb.append(" type=\"mesh\" mesh=\"").append(meshName(name)).append("\"/>\n");
      }
      else
      {
         sb.append("/><!-- TODO unsupported geometry: ").append(geometry.getClass().getSimpleName()).append(" -->\n");
      }
   }

   /** True if the geometry is emitted as a MuJoCo mesh (polytope, ramp, or supported model file). */
   static boolean isMeshGeometry(GeometryDefinition geometry)
   {
      if (geometry instanceof ConvexPolytope3DDefinition || geometry instanceof Ramp3DDefinition)
         return true;
      return geometry instanceof ModelFileGeometryDefinition modelFile && isSupportedModelFileMesh(modelFile);
   }

   static boolean isSupportedModelFileMesh(ModelFileGeometryDefinition modelFile)
   {
      String fileName = modelFile.getFileName();
      if (fileName == null)
         return false;
      String lower = fileName.toLowerCase();
      return lower.endsWith(".stl") || lower.endsWith(".obj");
   }

   /** The {@code <mesh>} asset name paired with the {@code <geom>} of the same {@code name}. */
   static String meshName(String geomName)
   {
      return geomName + "_mesh";
   }

   /**
    * If {@code shape} is a mesh geometry, emit its {@code <mesh>} asset. Convex polytopes / ramps use an
    * inline vertex cloud (MuJoCo builds the convex hull). {@link ModelFileGeometryDefinition} meshes
    * (STL/OBJ) are copied into {@code workingDirectory} and referenced via {@code file="..."}.
    * No-op for primitives. Pass {@code workingDirectory == null} only when no model-file meshes are present.
    */
   static void appendMeshAsset(StringBuilder sb, String name, CollisionShapeDefinition shape, int indent, File workingDirectory)
   {
      GeometryDefinition geometry = shape.getGeometryDefinition();
      if (!isMeshGeometry(geometry))
         return;

      if (geometry instanceof ModelFileGeometryDefinition modelFile)
      {
         appendModelFileMeshAsset(sb, name, modelFile, indent, workingDirectory);
         return;
      }

      sb.append("  ".repeat(indent)).append("<mesh name=\"").append(meshName(name)).append("\" vertex=\"");
      if (geometry instanceof ConvexPolytope3DDefinition polytope)
      {
         for (Vertex3DReadOnly vertex : polytope.getConvexPolytope().getVertices())
            appendVertex(sb, vertex.getX(), vertex.getY(), vertex.getZ());
      }
      else if (geometry instanceof Ramp3DDefinition ramp)
      {
         appendRampVertices(sb, ramp);
      }
      sb.append("\"/>\n");
   }

   /** Backward-compatible overload used by terrain (no model-file meshes). */
   static void appendMeshAsset(StringBuilder sb, String name, CollisionShapeDefinition shape, int indent)
   {
      appendMeshAsset(sb, name, shape, indent, null);
   }

   private static void appendModelFileMeshAsset(StringBuilder sb,
                                                String name,
                                                ModelFileGeometryDefinition modelFile,
                                                int indent,
                                                File workingDirectory)
   {
      if (workingDirectory == null)
      {
         LogTools.warn("Skipping MuJoCo mesh asset '{}': working directory is null", modelFile.getFileName());
         return;
      }

      String sourceFileName = modelFile.getFileName();
      if (!isSupportedModelFileMesh(modelFile))
      {
         LogTools.warn("Skipping MuJoCo mesh asset '{}': only .stl/.obj are supported", sourceFileName);
         return;
      }

      try
      {
         URL sourceURL = DefinitionIOTools.resolveModelFileURL(modelFile);
         String lower = sourceFileName.toLowerCase();
         String extension = lower.substring(lower.lastIndexOf('.'));
         // Created here rather than up front: a world whose collision shapes are all primitives, or
         // whose meshes are inline vertex clouds, never needs a directory on disk at all.
         if (!workingDirectory.exists() && !workingDirectory.mkdirs())
            throw new IOException("Could not create MuJoCo working directory: " + workingDirectory);
         File meshFile = new File(workingDirectory, meshName(name) + extension);
         try (InputStream in = sourceURL.openStream())
         {
            Files.copy(in, meshFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
         }

         sb.append("  ".repeat(indent)).append("<mesh name=\"").append(meshName(name))
           .append("\" file=\"").append(meshFile.getName()).append('"');
         Vector3DReadOnly scale = modelFile.getScale();
         if (scale != null && (scale.getX() != 1.0 || scale.getY() != 1.0 || scale.getZ() != 1.0))
         {
            sb.append(" scale=\"").append(scale.getX()).append(' ').append(scale.getY()).append(' ').append(scale.getZ()).append('"');
         }
         sb.append("/>\n");
      }
      catch (IOException | RuntimeException e)
      {
         throw new RuntimeException("Failed to stage MuJoCo mesh '" + sourceFileName + "' into " + workingDirectory, e);
      }
   }

   /**
    * Append the 6 corner vertices of a ramp (a triangular prism) as a flat vertex list. Matches the
    * SCS2 / Bullet convention: the bottom face is centered at the origin and the slope rises toward
    * +x, reaching {@code sizeZ} at the +x end.
    */
   private static void appendRampVertices(StringBuilder sb, Ramp3DDefinition ramp)
   {
      double x = 0.5 * ramp.getSizeX();
      double y = 0.5 * ramp.getSizeY();
      double z = ramp.getSizeZ();
      // bottom rectangle (z=0) then the two high-edge corners at the +x end (z=sizeZ).
      appendVertex(sb, -x, -y, 0.0);
      appendVertex(sb, -x, y, 0.0);
      appendVertex(sb, x, y, 0.0);
      appendVertex(sb, x, -y, 0.0);
      appendVertex(sb, x, y, z);
      appendVertex(sb, x, -y, z);
   }

   private static void appendVertex(StringBuilder sb, double x, double y, double z)
   {
      if (sb.charAt(sb.length() - 1) != '"')
         sb.append("  ");
      sb.append(x).append(' ').append(y).append(' ').append(z);
   }

   /**
    * Emit a body's {@code <inertial>} element (CoM offset, mass, and full inertia tensor).
    *
    * <p>SCS2 expresses the moment of inertia in the frame given by
    * {@link RigidBodyDefinition#getInertiaPose()}, whose translation is the CoM offset and whose
    * rotation is the inertia frame's orientation relative to the body frame. MuJoCo's
    * {@code fullinertia} is read in the body frame (the compiler eigen-decomposes it and derives
    * the inertial frame orientation itself), so a rotated inertia frame has to be folded in first
    * as {@code R * I * R^T}. Most URDFs, Alex included, give zero rpy on every inertial, in which
    * case this is a no-op.
    */
   static void appendInertial(StringBuilder sb, RigidBodyDefinition body, int indent)
   {
      String pad = "  ".repeat(indent);
      Tuple3DReadOnly comOffset = body.getCenterOfMassOffset();
      Matrix3D inertia = new Matrix3D(body.getMomentOfInertia());
      Orientation3DReadOnly inertiaFrameRotation = body.getInertiaPose().getRotation();
      if (!inertiaFrameRotation.isZeroOrientation())
         inertiaFrameRotation.transform(inertia);

      sb.append(pad).append("<inertial")
        .append(" pos=\"").append(comOffset.getX()).append(' ').append(comOffset.getY()).append(' ').append(comOffset.getZ()).append('"')
        .append(" mass=\"").append(body.getMass()).append('"');
      if (body.getMass() > 0.0)
      {
         // fullinertia order: Ixx Iyy Izz Ixy Ixz Iyz
         sb.append(" fullinertia=\"")
           .append(inertia.getM00()).append(' ')
           .append(inertia.getM11()).append(' ')
           .append(inertia.getM22()).append(' ')
           .append(inertia.getM01()).append(' ')
           .append(inertia.getM02()).append(' ')
           .append(inertia.getM12())
           .append('"');
      }
      sb.append("/>\n");
   }

   /** Emit a joint's MJCF element: {@code <freejoint>} for the root, {@code <joint>} for 1-DoF. */
   static void appendJoint(StringBuilder sb,
                           JointDefinition joint,
                           String namePrefix,
                           MujocoSimulationParametersReadOnly parameters,
                           int indent)
   {
      String pad = "  ".repeat(indent);
      if (joint instanceof SixDoFJointDefinition)
      {
         sb.append(pad).append("<freejoint name=\"").append(namePrefix).append(joint.getName()).append("\"/>\n");
      }
      else if (joint instanceof RevoluteJointDefinition rev)
      {
         appendOneDofJoint(sb, pad, "hinge", namePrefix, rev, parameters);
      }
      else if (joint instanceof PrismaticJointDefinition pris)
      {
         appendOneDofJoint(sb, pad, "slide", namePrefix, pris, parameters);
      }
      else
      {
         // Emitting nothing would weld the body to its parent and silently cost the robot a degree
         // of freedom -- it would simulate, just not the robot that was asked for. Cross-four-bar
         // and revolute-twins joints need <equality connect> and fixed <tendon> respectively; until
         // the builder emits those, refuse the model.
         throw new UnsupportedOperationException("The MuJoCo MJCF builder cannot represent joint '" + joint.getName() + "' of type "
                                                 + joint.getClass().getSimpleName()
                                                 + ". Supported types are SixDoFJointDefinition (as the floating root), RevoluteJointDefinition"
                                                 + " and PrismaticJointDefinition.");
      }
   }

   private static void appendOneDofJoint(StringBuilder sb,
                                         String pad,
                                         String mjcfType,
                                         String namePrefix,
                                         OneDoFJointDefinition jointDef,
                                         MujocoSimulationParametersReadOnly parameters)
   {
      Tuple3DReadOnly axis = jointDef.getAxis();
      sb.append(pad).append("<joint name=\"").append(namePrefix).append(jointDef.getName())
        .append("\" type=\"").append(mjcfType)
        .append("\" axis=\"").append(axis.getX()).append(' ').append(axis.getY()).append(' ').append(axis.getZ()).append('"');

      // Damping
      if (jointDef.getDamping() > 0.0)
         sb.append(" damping=\"").append(jointDef.getDamping()).append('"');

      // Coulomb joint friction. No SCS2 engine reads stiction today, so for the URDFs in use
      // (Alex and Zulu both declare friction="0.0") this emits nothing.
      if (jointDef.getStiction() > 0.0)
         sb.append(" frictionloss=\"").append(jointDef.getStiction()).append('"');

      // Reflected rotor inertia. Emitted only when the joint carries its own value; otherwise the
      // joint inherits the <default> block's armature, which is the model-wide fallback.
      if (jointDef.getArmature() > 0.0)
         sb.append(" armature=\"").append(jointDef.getArmature()).append('"');

      // Position limits. solreflimit is deliberately left at MuJoCo's default rather than derived
      // from getKpSoftLimitStop()/getKdSoftLimitStop(): MuJoCo's negative-solref form specifies
      // stiffness per unit of acceleration, so those gains would need scaling by the joint's
      // effective inertia to mean the same thing. A default-stiffness stop is closer to the other
      // engines than no stop at all; gain-matching it is a follow-up.
      if (parameters.getEnforceJointLimits() && hasFinitePositionLimits(jointDef))
      {
         sb.append(" limited=\"true\" range=\"").append(jointDef.getPositionLowerLimit())
           .append(' ').append(jointDef.getPositionUpperLimit()).append('"');
      }

      // Total actuator force limit, clamping qfrc_actuator. This is what reproduces
      // SCS2OutputWriter's clamp of the summed torque inside the plant. It does not reach joints
      // the builder cannot give an actuator, whose torque still arrives through qfrc_applied.
      if (hasFiniteEffortLimits(jointDef))
      {
         sb.append(" actuatorfrclimited=\"true\" actuatorfrcrange=\"").append(jointDef.getEffortLowerLimit())
           .append(' ').append(jointDef.getEffortUpperLimit()).append('"');
      }

      sb.append("/>\n");
   }

   /** True when the joint declares a usable position range, matching {@code OneDoFJointDefinition}'s unset sentinels. */
   static boolean hasFinitePositionLimits(OneDoFJointDefinition jointDef)
   {
      double lower = jointDef.getPositionLowerLimit();
      double upper = jointDef.getPositionUpperLimit();
      return Double.isFinite(lower) && Double.isFinite(upper) && lower < upper;
   }

   /** True when the joint declares a usable effort range. */
   static boolean hasFiniteEffortLimits(OneDoFJointDefinition jointDef)
   {
      double lower = jointDef.getEffortLowerLimit();
      double upper = jointDef.getEffortUpperLimit();
      return Double.isFinite(lower) && Double.isFinite(upper) && lower < upper;
   }

   /** True if the transform has neither rotation nor translation (so pos/quat can be omitted). */
   static boolean isIdentity(RigidBodyTransformReadOnly transform)
   {
      return !transform.hasRotation() && !transform.hasTranslation();
   }

   private static final int MODEL_SUMMARY_MAX_GEOMS = 8;

   /** Logs the compiled model's effective option and per-geom contact values — what MuJoCo actually runs vs what was requested. */
   /**
    * Prints {@code mjModel.opt} READ BACK FROM THE NATIVE STRUCT, so it states what MuJoCo holds
    * rather than what SCS2 believes it set. Use it to settle whether an option edit actually landed:
    * nothing here passes through a YoVariable on the way out.
    * <p>
    * Two things it is deliberately explicit about, because both have caused wasted days here:
    * <ul>
    * <li>{@code timestep} is whatever {@code opt} holds at this write. Since {@code writeOptions}
    * runs at compile and the engine rewrites {@code opt.timestep} every TICK from the session DT, a
    * compile-time readback shows the pre-tick value rather than what MuJoCo steps with. Derive the
    * stepping timestep from the recording cadence instead.
    * <li>The {@code o_*} entries are annotated with whether {@code mjENBL_OVERRIDE} is actually set.
    * With the flag clear they are dead weight: contact uses the PER-GEOM solref/solimp instead, and
    * editing {@code o_solref}/{@code o_solimp} changes nothing at all.
    * </ul>
    */
   public static void logEffectiveOptions(Mujoco.mjModel model, String context)
   {
      Mujoco.mjOption opt = model.opt();
      int enableflags = opt.enableflags();
      int disableflags = opt.disableflags();
      boolean overrideActive = (enableflags & Mujoco.mjENBL_OVERRIDE) != 0;

      StringBuilder summary = new StringBuilder();
      summary.append(String.format("MuJoCo mjOption read back from the native model (%s):%n", context));
      // NOTE the timestep here is whatever opt holds at the moment of this write. writeOptions runs
      // at compile and on option edits, and the engine rewrites opt.timestep every TICK from the
      // session DT -- so a compile-time readback shows the pre-tick value, not what MuJoCo steps with.
      // Derive the stepping timestep from the recording cadence, not from this line.
      summary.append(String.format("  timestep=%s (as held at this write; see note) integrator=%d solver=%d cone=%d jacobian=%d%n",
                                   opt.timestep(),
                                   opt.integrator(),
                                   opt.solver(),
                                   opt.cone(),
                                   opt.jacobian()));
      summary.append(String.format("  impratio=%s tolerance=%s iterations=%d ls_iterations=%d noslip_iterations=%d%n",
                                   opt.impratio(),
                                   opt.tolerance(),
                                   opt.iterations(),
                                   opt.ls_iterations(),
                                   opt.noslip_iterations()));
      summary.append(String.format("  enableflags=0x%x [%s]  disableflags=0x%x [%s]%n",
                                   enableflags,
                                   describeEnableFlags(enableflags),
                                   disableflags,
                                   describeDisableFlags(disableflags)));
      summary.append(String.format("  o_* contact override is %s%n",
                                   overrideActive ? "ACTIVE: the values below replace solref/solimp/margin/friction on EVERY contact"
                                                  : "INACTIVE (mjENBL_OVERRIDE clear): the values below are IGNORED; contact uses per-geom solref/solimp"));
      summary.append(String.format("  o_margin=%s o_solref=(%s %s) o_solimp=(%s %s %s %s %s) o_friction=(%s %s %s %s %s)%n",
                                   opt.o_margin(),
                                   opt.o_solref(0),
                                   opt.o_solref(1),
                                   opt.o_solimp(0),
                                   opt.o_solimp(1),
                                   opt.o_solimp(2),
                                   opt.o_solimp(3),
                                   opt.o_solimp(4),
                                   opt.o_friction(0),
                                   opt.o_friction(1),
                                   opt.o_friction(2),
                                   opt.o_friction(3),
                                   opt.o_friction(4)));

      // The per-geom values are what contact actually uses when the override is off, so print the
      // first few: they are compile-time only and cannot be changed at runtime.
      int ngeom = (int) model.ngeom();
      DoublePointer geomSolref = model.geom_solref();
      DoublePointer geomSolimp = model.geom_solimp();
      for (int geomId = 0; geomId < Math.min(ngeom, MODEL_SUMMARY_MAX_GEOMS); geomId++)
      {
         BytePointer namePointer = Mujoco.mj_id2name(model, Mujoco.mjOBJ_GEOM, geomId);
         summary.append(String.format("  geom[%d] %s: solref=(%s %s) solimp=(%s %s %s %s %s)%n",
                                      geomId,
                                      namePointer == null || namePointer.isNull() ? "(unnamed)" : namePointer.getString(),
                                      geomSolref.get(2L * geomId),
                                      geomSolref.get(2L * geomId + 1),
                                      geomSolimp.get(5L * geomId),
                                      geomSolimp.get(5L * geomId + 1),
                                      geomSolimp.get(5L * geomId + 2),
                                      geomSolimp.get(5L * geomId + 3),
                                      geomSolimp.get(5L * geomId + 4)));
      }
      if (ngeom > MODEL_SUMMARY_MAX_GEOMS)
         summary.append(String.format("  ... %d more geoms elided%n", ngeom - MODEL_SUMMARY_MAX_GEOMS));

      LogTools.info(summary.toString());
   }

   private static String describeEnableFlags(int enableflags)
   {
      StringBuilder names = new StringBuilder();
      appendFlag(names, enableflags, Mujoco.mjENBL_OVERRIDE, "OVERRIDE");
      appendFlag(names, enableflags, Mujoco.mjENBL_ENERGY, "ENERGY");
      appendFlag(names, enableflags, Mujoco.mjENBL_FWDINV, "FWDINV");
      appendFlag(names, enableflags, Mujoco.mjENBL_INVDISCRETE, "INVDISCRETE");
      appendFlag(names, enableflags, Mujoco.mjENBL_SLEEP, "SLEEP");
      appendFlag(names, enableflags, Mujoco.mjENBL_DIAGEXACT, "DIAGEXACT");
      return names.length() == 0 ? "none" : names.toString();
   }

   private static String describeDisableFlags(int disableflags)
   {
      StringBuilder names = new StringBuilder();
      appendFlag(names, disableflags, Mujoco.mjDSBL_CONSTRAINT, "CONSTRAINT");
      appendFlag(names, disableflags, Mujoco.mjDSBL_EQUALITY, "EQUALITY");
      appendFlag(names, disableflags, Mujoco.mjDSBL_FRICTIONLOSS, "FRICTIONLOSS");
      appendFlag(names, disableflags, Mujoco.mjDSBL_LIMIT, "LIMIT");
      appendFlag(names, disableflags, Mujoco.mjDSBL_CONTACT, "CONTACT");
      appendFlag(names, disableflags, Mujoco.mjDSBL_SPRING, "SPRING");
      appendFlag(names, disableflags, Mujoco.mjDSBL_DAMPER, "DAMPER");
      appendFlag(names, disableflags, Mujoco.mjDSBL_GRAVITY, "GRAVITY");
      appendFlag(names, disableflags, Mujoco.mjDSBL_CLAMPCTRL, "CLAMPCTRL");
      appendFlag(names, disableflags, Mujoco.mjDSBL_WARMSTART, "WARMSTART");
      appendFlag(names, disableflags, Mujoco.mjDSBL_FILTERPARENT, "FILTERPARENT");
      appendFlag(names, disableflags, Mujoco.mjDSBL_ACTUATION, "ACTUATION");
      appendFlag(names, disableflags, Mujoco.mjDSBL_REFSAFE, "REFSAFE");
      appendFlag(names, disableflags, Mujoco.mjDSBL_SENSOR, "SENSOR");
      appendFlag(names, disableflags, Mujoco.mjDSBL_MIDPHASE, "MIDPHASE");
      appendFlag(names, disableflags, Mujoco.mjDSBL_EULERDAMP, "EULERDAMP");
      appendFlag(names, disableflags, Mujoco.mjDSBL_AUTORESET, "AUTORESET");
      appendFlag(names, disableflags, Mujoco.mjDSBL_NATIVECCD, "NATIVECCD");
      appendFlag(names, disableflags, Mujoco.mjDSBL_ISLAND, "ISLAND");
      appendFlag(names, disableflags, Mujoco.mjDSBL_MULTICCD, "MULTICCD");
      return names.length() == 0 ? "none" : names.toString();
   }

   private static void appendFlag(StringBuilder names, int flags, int bit, String name)
   {
      if ((flags & bit) == 0)
         return;
      if (names.length() > 0)
         names.append('|');
      names.append(name);
   }

   /**
    * Dumps the entire compiled {@code mjModel} through MuJoCo's own {@code mj_printModel}, i.e. every
    * field MuJoCo holds, written by MuJoCo rather than reformatted by us. {@code /dev/stdout} sends it
    * to the console; any other path writes a file, which is usually what you want since the dump is
    * tens of thousands of lines for a humanoid.
    */
   public static void printNativeModel(Mujoco.mjModel model, String filename)
   {
      LogTools.info("Dumping the full native mjModel via mj_printModel to {}", filename);
      Mujoco.mj_printModel(model, filename);
   }

   public static void logEffectiveModelSummary(Mujoco.mjModel model)
   {
      Mujoco.mjOption opt = model.opt();
      StringBuilder summary = new StringBuilder("MuJoCo compiled model summary:\n");
      summary.append(String.format("  option: timestep=%s integrator=%s solver=%s cone=%s jacobian=%s iterations=%d ls_iterations=%d tolerance=%s"
                                   + " impratio=%s noslip_iterations=%d%n",
                                   opt.timestep(),
                                   MujocoIntegrator.fromMujocoValue(opt.integrator()),
                                   MujocoSolver.fromMujocoValue(opt.solver()),
                                   MujocoCone.fromMujocoValue(opt.cone()),
                                   MujocoJacobian.fromMujocoValue(opt.jacobian()),
                                   opt.iterations(),
                                   opt.ls_iterations(),
                                   opt.tolerance(),
                                   opt.impratio(),
                                   opt.noslip_iterations()));

      int ngeom = (int) model.ngeom();
      int printed = Math.min(ngeom, MODEL_SUMMARY_MAX_GEOMS);
      DoublePointer friction = model.geom_friction();
      DoublePointer solref = model.geom_solref();
      DoublePointer solimp = model.geom_solimp();
      DoublePointer margin = model.geom_margin();
      DoublePointer gap = model.geom_gap();
      IntPointer condim = model.geom_condim();
      for (int geomId = 0; geomId < printed; geomId++)
      {
         BytePointer namePointer = Mujoco.mj_id2name(model, Mujoco.mjOBJ_GEOM, geomId);
         String name = namePointer == null || namePointer.isNull() ? "(unnamed)" : namePointer.getString();
         summary.append(String.format("  geom[%d] %s: friction=(%s %s %s) solref=(%s %s) solimp=(%s %s %s %s %s) condim=%d margin=%s gap=%s%n",
                                      geomId,
                                      name,
                                      friction.get(3 * geomId),
                                      friction.get(3 * geomId + 1),
                                      friction.get(3 * geomId + 2),
                                      solref.get(2L * geomId),
                                      solref.get(2L * geomId + 1),
                                      solimp.get(5L * geomId),
                                      solimp.get(5L * geomId + 1),
                                      solimp.get(5L * geomId + 2),
                                      solimp.get(5L * geomId + 3),
                                      solimp.get(5L * geomId + 4),
                                      condim.get(geomId),
                                      margin.get(geomId),
                                      gap.get(geomId)));
      }
      if (ngeom > printed)
         summary.append("  ... ").append(ngeom - printed).append(" more geoms elided\n");
      LogTools.info(summary.toString());
   }
}

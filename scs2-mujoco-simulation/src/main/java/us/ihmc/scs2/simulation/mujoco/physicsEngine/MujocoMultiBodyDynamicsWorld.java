package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.util.ArrayList;
import java.util.List;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjContact;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjOption;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.YoMujocoOptions;

/**
 * Owns the compiled `mjModel` and the runtime `mjData` for a single MuJoCo simulation.
 *
 * <p>v1 design: a MuJoCo simulation is one composite MJCF document that includes every robot's
 * URDF plus the terrain geoms. We assemble the MJCF text in {@link MujocoMultiBodyRobotFactory}
 * and call {@link #compile(String)} exactly once, then step the world from
 * {@link us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoPhysicsEngine#simulate}.
 *
 * <p>Adding robots after the world is compiled would require rebuilding `mjModel`, which MuJoCo
 * supports via `mjSpec` but is out of scope for v1.
 */
public class MujocoMultiBodyDynamicsWorld
{
   /**
    * {@code SCS2_MUJOCO_LOG_OPTIONS=1} logs {@code mjOption} read back from the native model after
    * every option write, which is how you confirm an edit actually landed. Noisy by design.
    */
   private static final boolean LOG_EFFECTIVE_OPTIONS = "1".equals(System.getenv("SCS2_MUJOCO_LOG_OPTIONS"));

   private mjModel model;
   private mjData data;
   private final List<MujocoMultiBodyRobot> robots = new ArrayList<>();
   private final List<MujocoTerrainObject> terrainObjects = new ArrayList<>();

   /**
    * Compiles the supplied MJCF text into {@code mjModel} + {@code mjData}.
    *
    * <p>{@code mj_loadXML} only accepts a file path, so the XML is written to {@code mjcfFile}
    * first (which {@link MujocoMultiBodyRobotFactory} typically places in a per-session working
    * directory next to the per-robot URDF includes).
    */
   private static final String VIRTUAL_MJCF_NAME = "world.xml";

   /**
    * Compiles the supplied MJCF text into {@code mjModel} + {@code mjData}.
    *
    * <p>The XML is handed to MuJoCo through a virtual file system rather than written to disk, so a
    * world whose collision shapes are all primitives touches the filesystem not at all. File-backed
    * meshes are still staged into the working directory, which the MJCF points at with an absolute
    * {@code meshdir} precisely because a VFS entry has no directory of its own.
    *
    * @param mjcfFileOrNull when non-null, the MJCF is also written here for inspection. The
    *       compile reads the in-memory copy either way.
    */
   public void compile(String mjcfXml, java.io.File mjcfFileOrNull)
   {
      if (model != null || data != null)
         throw new IllegalStateException("MuJoCo model already compiled. Multiple compiles per world are not supported in v1.");

      if (mjcfFileOrNull != null)
      {
         try
         {
            java.io.File parent = mjcfFileOrNull.getParentFile();
            if (parent != null && !parent.exists() && !parent.mkdirs())
               throw new java.io.IOException("Could not create " + parent);
            java.nio.file.Files.writeString(mjcfFileOrNull.toPath(), mjcfXml);
         }
         catch (java.io.IOException e)
         {
            throw new RuntimeException("Could not write MJCF to " + mjcfFileOrNull, e);
         }
      }

      BytePointer errorBuffer = new BytePointer(1000);
      byte[] mjcfBytes = mjcfXml.getBytes(java.nio.charset.StandardCharsets.UTF_8);
      Mujoco.mjVFS vfs = new Mujoco.mjVFS();
      try (BytePointer mjcfBuffer = new BytePointer(mjcfBytes.length))
      {
         Mujoco.mj_defaultVFS(vfs);
         mjcfBuffer.put(mjcfBytes);
         if (Mujoco.mj_addBufferVFS(vfs, VIRTUAL_MJCF_NAME, mjcfBuffer, mjcfBytes.length) != 0)
            throw new RuntimeException("Could not add the MJCF to MuJoCo's virtual file system");

         model = Mujoco.mj_loadXML(VIRTUAL_MJCF_NAME, vfs, errorBuffer, 1000);
      }
      finally
      {
         Mujoco.mj_deleteVFS(vfs);
      }

      if (model == null || model.isNull())
      {
         throw new RuntimeException("mj_loadXML failed: " + errorBuffer.getString());
      }
      data = Mujoco.mj_makeData(model);
      if (data == null || data.isNull())
         throw new RuntimeException("mj_makeData failed (out of memory?)");
   }

   public mjModel getModel()
   {
      return model;
   }

   public mjData getData()
   {
      return data;
   }

   public List<MujocoMultiBodyRobot> getRobots()
   {
      return robots;
   }

   public List<MujocoTerrainObject> getTerrainObjects()
   {
      return terrainObjects;
   }

   public void addMujocoRobot(MujocoMultiBodyRobot robot)
   {
      robots.add(robot);
   }

   public void addMujocoTerrainObject(MujocoTerrainObject terrainObject)
   {
      terrainObjects.add(terrainObject);
   }

   public void setGravity(Vector3DReadOnly gravity)
   {
      if (model == null)
         return;
      // mjOption.gravity is an mjtNum[3]. Mutate in place via JavaCPP pointer indexing.
      DoublePointer gravityPointer = model.opt().gravity();
      gravityPointer.put(0, gravity.getX());
      gravityPointer.put(1, gravity.getY());
      gravityPointer.put(2, gravity.getZ());
   }

   /**
    * Reset {@code mjData} to the compiled model's default state (qpos0, zero velocities, no
    * warm-start or contact carry-over). Used when the session re-initializes an already compiled
    * world; the per-robot initial joint state is re-seeded on top by the caller.
    */
   public void resetData()
   {
      if (model == null || data == null)
         return;
      Mujoco.mj_resetData(model, data);
   }

   /**
    * Copy the current {@code qpos}/{@code qvel} into keyframe 0, the empty {@code <key>} the MJCF
    * declares. Called once after the robots' initial joint state has been seeded, so that the
    * keyframe holds the pose the robots actually spawn in rather than the model's {@code qpos0}.
    */
   public void captureInitialKeyframe()
   {
      if (model == null || data == null || model.nkey() < 1)
         return;

      DoublePointer keyQpos = model.key_qpos();
      DoublePointer qpos = data.qpos();
      for (int i = 0; i < model.nq(); i++)
         keyQpos.put(i, qpos.get(i));

      DoublePointer keyQvel = model.key_qvel();
      DoublePointer qvel = data.qvel();
      for (int i = 0; i < model.nv(); i++)
         keyQvel.put(i, qvel.get(i));
   }

   /**
    * Reset to keyframe 0, i.e. the seeded initial state. Unlike a bare {@code mj_resetData} this
    * restores the spawn pose rather than {@code qpos0}, and it clears velocities, warm-start
    * accelerations and contact state at the same time.
    */
   public void resetToInitialKeyframe()
   {
      if (model == null || data == null)
         return;
      if (model.nkey() < 1)
         Mujoco.mj_resetData(model, data);
      else
         Mujoco.mj_resetDataKeyframe(model, data, 0);
   }

   public void step()
   {
      Mujoco.mj_step(model, data);
      // mj_step only computes cfrc_ext / cfrc_int when MJCF sensors require them. We read
      // cfrc_ext directly per-tick in MujocoRobot.updateSensors so the F/T sensor plumbing has
      // contact wrenches to integrate; force the post-constraint pass here unconditionally.
      Mujoco.mj_rnePostConstraint(model, data);
   }

   public void step(int substeps)
   {
      for (int i = 0; i < substeps; i++)
         Mujoco.mj_step(model, data);
      Mujoco.mj_rnePostConstraint(model, data);
   }

   private static int setBit(int flags, int bit, boolean set)
   {
      return set ? flags | bit : flags & ~bit;
   }

   private boolean warnedShortSolrefTimeconst = false;

   /**
    * Pushes all runtime-tunable options into {@code mjModel.opt}; effective on the next
    * {@code mj_step}. Physics thread only. {@code timestep} and {@code gravity} are engine-owned.
    */
   public void writeOptions(YoMujocoOptions options)
   {
      if (model == null)
         return;

      mjOption opt = model.opt();
      opt.impratio(options.impratio.getValue());
      opt.tolerance(options.tolerance.getValue());
      opt.ls_tolerance(options.ls_tolerance.getValue());
      opt.noslip_tolerance(options.noslip_tolerance.getValue());
      opt.ccd_tolerance(options.ccd_tolerance.getValue());
      opt.iterations(options.iterations.getValue());
      opt.ls_iterations(options.ls_iterations.getValue());
      opt.noslip_iterations(options.noslip_iterations.getValue());
      opt.ccd_iterations(options.ccd_iterations.getValue());
      opt.solver(options.solver.getEnumValue().toMujocoValue());
      opt.cone(options.cone.getEnumValue().toMujocoValue());
      opt.jacobian(options.jacobian.getEnumValue().toMujocoValue());
      opt.integrator(options.integrator.getEnumValue().toMujocoValue());

      // Only touch the bits this class manages; other flags may be owned elsewhere.
      int enableflags = opt.enableflags();
      enableflags = setBit(enableflags, Mujoco.mjENBL_OVERRIDE, options.enableOverride.getValue());
      enableflags = setBit(enableflags, Mujoco.mjENBL_ENERGY, options.enableEnergy.getValue());
      enableflags = setBit(enableflags, Mujoco.mjENBL_FWDINV, options.enableFwdinv.getValue());
      enableflags = setBit(enableflags, Mujoco.mjENBL_DIAGEXACT, options.enableDiagexact.getValue());
      opt.enableflags(enableflags);

      // Read-modify-write matters more here than for the enable flags: mjDSBL_FILTERPARENT is set
      // from the MJCF at compile time and is not mirrored in the options group, so writing the word
      // wholesale would silently turn parent-child collision filtering back on.
      int disableflags = opt.disableflags();
      disableflags = setBit(disableflags, Mujoco.mjDSBL_CONTACT, options.disableContact.getValue());
      disableflags = setBit(disableflags, Mujoco.mjDSBL_GRAVITY, options.disableGravity.getValue());
      disableflags = setBit(disableflags, Mujoco.mjDSBL_LIMIT, options.disableLimit.getValue());
      disableflags = setBit(disableflags, Mujoco.mjDSBL_EQUALITY, options.disableEquality.getValue());
      disableflags = setBit(disableflags, Mujoco.mjDSBL_ACTUATION, options.disableActuation.getValue());
      disableflags = setBit(disableflags, Mujoco.mjDSBL_WARMSTART, options.disableWarmstart.getValue());
      disableflags = setBit(disableflags, Mujoco.mjDSBL_AUTORESET, options.disableAutoReset.getValue());
      opt.disableflags(disableflags);

      opt.o_margin(options.o_margin.getValue());
      opt.o_solref(0, options.o_solref_timeconst.getValue());
      opt.o_solref(1, options.o_solref_dampratio.getValue());
      opt.o_solimp(0, options.o_solimp_dmin.getValue());
      opt.o_solimp(1, options.o_solimp_dmax.getValue());
      opt.o_solimp(2, options.o_solimp_width.getValue());
      opt.o_solimp(3, options.o_solimp_midpoint.getValue());
      opt.o_solimp(4, options.o_solimp_power.getValue());
      opt.o_friction(0, options.o_friction_slide.getValue());
      opt.o_friction(1, options.o_friction_slide.getValue());
      opt.o_friction(2, options.o_friction_spin.getValue());
      opt.o_friction(3, options.o_friction_roll.getValue());
      opt.o_friction(4, options.o_friction_roll.getValue());

      // MuJoCo's refsafe guard only clamps solref coming from MJCF, not runtime struct writes.
      double timeconst = options.o_solref_timeconst.getValue();
      if (options.enableOverride.getValue() && timeconst > 0.0 && timeconst < 2.0 * opt.timestep() && !warnedShortSolrefTimeconst)
      {
         warnedShortSolrefTimeconst = true;
         LogTools.warn("o_solref_timeconst ({}) is below MuJoCo's stability requirement of 2*timestep ({}); expect contact instability.",
                       timeconst,
                       2.0 * opt.timestep());
      }

      // Read the values straight back out of the native struct and log them, so a question about
      // whether an edit reached MuJoCo is answered by MuJoCo rather than by what we think we set.
      // Off unless asked: this fires on every option change.
      if (LOG_EFFECTIVE_OPTIONS)
         MujocoTools.logEffectiveOptions(model, "after writeOptions");
   }


   public double getTimestep()
   {
      return model.opt().timestep();
   }

   public void setTimestep(double dt)
   {
      // mjOption is a struct value; the generated wrapper exposes it via accessors. The setter for
      // timestep is auto-generated by JavaCPP.
      model.opt().timestep(dt);
   }

   public void dispose()
   {
      if (data != null && !data.isNull())
      {
         Mujoco.mj_deleteData(data);
         data = null;
      }
      if (model != null && !model.isNull())
      {
         Mujoco.mj_deleteModel(model);
         model = null;
      }
      robots.clear();
      terrainObjects.clear();
   }
}

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
   public void compile(String mjcfXml, java.io.File mjcfFile)
   {
      if (model != null || data != null)
         throw new IllegalStateException("MuJoCo model already compiled. Multiple compiles per world are not supported in v1.");

      try
      {
         java.nio.file.Files.writeString(mjcfFile.toPath(), mjcfXml);
      }
      catch (java.io.IOException e)
      {
         throw new RuntimeException("Could not write MJCF to " + mjcfFile, e);
      }

      BytePointer errorBuffer = new BytePointer(1000);
      model = Mujoco.mj_loadXML(mjcfFile.getAbsolutePath(), null, errorBuffer, 1000);
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

      // Only touch the two bits this class manages; other enable flags may be owned elsewhere.
      int enableflags = opt.enableflags();
      enableflags = options.enableOverride.getValue() ? enableflags | Mujoco.mjENBL_OVERRIDE : enableflags & ~Mujoco.mjENBL_OVERRIDE;
      enableflags = options.enableEnergy.getValue() ? enableflags | Mujoco.mjENBL_ENERGY : enableflags & ~Mujoco.mjENBL_ENERGY;
      opt.enableflags(enableflags);

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

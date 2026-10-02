package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;

/**
 * Pins the reference point of {@code cfrc_ext}'s moment, which {@link MujocoRobot#updateSensors}
 * depends on to place the foot force/torque wrench, and which therefore sets the centre of pressure
 * the state estimator plants a trusted foot at.
 *
 * <p>Like the rest of the com-based {@code c*} family (see {@link MujocoComFrameReferenceTest}) the
 * moment is referenced at {@code subtree_com[body_rootid[bodyId]]} -- the centre of mass of the whole
 * robot -- and not at the body's own centre of mass. Confusing the two biases a foot moment by
 * {@code (C - S) x F}, which this model measures at ~27 N*m on ~640 N of load, or roughly 40 mm of
 * centre-of-pressure error that wanders as the robot's centre of mass does.
 *
 * <p>Ground truth is the contact itself rather than another MuJoCo array: for a body touching the
 * ground at a single resolved point, {@code m_P = (p_contact - P) x F} at any reference point P.
 *
 * <p>Written against the raw bindings rather than a {@code SimulationSession} because it needs a
 * single, exactly-resolved contact; a compiled SCS2 collision model gives several.
 */
public class MujocoContactWrenchReferenceTest
{
   /** Exact: the comparison is an algebraic identity evaluated on one resolved contact. */
   private static final double TOLERANCE = 1.0e-9;

   private mjModel model;
   private mjData data;

   @BeforeAll
   public static void loadNatives()
   {
      assertTrue(MujocoNativeLibrary.load(), "MuJoCo native library failed to load");
   }

   @AfterEach
   public void shutdown()
   {
      if (data != null)
         Mujoco.mj_deleteData(data);
      if (model != null)
         Mujoco.mj_deleteModel(model);
      data = null;
      model = null;
   }

   /**
    * Heavy root high up, light foot far below resting on the ground: that puts the whole-robot
    * centre of mass about a metre from the foot, the same order as a real humanoid's foot-to-CoM
    * distance, so the two candidate reference points are far apart.
    */
   private static final String MJCF = """
         <mujoco>
           <option gravity="0 0 -9.81" timestep="0.001"/>
           <worldbody>
             <geom name="ground" type="plane" size="0 0 1" pos="0 0 0"/>
             <body name="pelvis" pos="0.13 -0.07 1.0">
               <freejoint name="root"/>
               <inertial pos="0 0 0" mass="40" diaginertia="1 1 1"/>
               <geom type="box" size="0.1 0.1 0.1" mass="0" contype="0" conaffinity="0"/>
               <body name="foot" pos="0.2 0.05 -0.95">
                 <joint name="ankle" type="hinge" axis="0 1 0" damping="20"/>
                 <inertial pos="0.01 0.005 0" mass="2" diaginertia="0.01 0.01 0.01"/>
                 <geom name="sole" type="sphere" size="0.05" mass="0"/>
               </body>
             </body>
           </worldbody>
         </mujoco>
         """;

   @Test
   public void testContactWrenchMomentIsReferencedAtSubtreeCom() throws Exception
   {
      java.nio.file.Path file = java.nio.file.Files.createTempFile("contactWrenchRef", ".xml");
      try
      {
         java.nio.file.Files.writeString(file, MJCF);
         byte[] error = new byte[1000];
         model = Mujoco.mj_loadXML(file.toString(), null, error, 1000);
         assertTrue(model != null && !model.isNull(), "MJCF failed to compile: " + new String(error).trim());
      }
      finally
      {
         java.nio.file.Files.deleteIfExists(file);
      }
      data = Mujoco.mj_makeData(model);
      Mujoco.mj_resetData(model, data);

      int footBodyId = nameToId(Mujoco.mjOBJ_BODY, "foot");
      int soleGeomId = nameToId(Mujoco.mjOBJ_GEOM, "sole");

      for (int i = 0; i < 2000; i++)
         Mujoco.mj_step(model, data);
      Mujoco.mj_forward(model, data);
      Mujoco.mj_rnePostConstraint(model, data);   // mj_forward does not fill cfrc_ext / cacc

      assertEquals(1, data.ncon(), "expected exactly one contact so the ground truth is unambiguous");

      var contact = data.contact().position(0);
      Vector3D contactPoint = new Vector3D(contact.pos().get(0), contact.pos().get(1), contact.pos().get(2));

      // mj_contactForce reports in the contact frame, whose rows are its axes; rotate out to world.
      DoublePointer contactForce = new DoublePointer(6);
      Mujoco.mj_contactForce(model, data, 0, contactForce);
      Vector3D force = new Vector3D();
      for (int column = 0; column < 3; column++)
      {
         double sum = 0.0;
         for (int row = 0; row < 3; row++)
            sum += contact.frame().get(row * 3 + column) * contactForce.get(row);
         force.setElement(column, sum);
      }
      // MuJoCo reports the force applied to geom2 by geom1; flip when the sole is geom1.
      if (contact.geom1() == soleGeomId)
         force.negate();

      Vector3D bodyOrigin = point(data.xpos(), footBodyId);
      Vector3D bodyOwnCom = point(data.xipos(), footBodyId);
      Vector3D subtreeCom = point(data.subtree_com(), model.body_rootid().get(footBodyId));

      // Guard the discriminating power of the model: if these coincide the test goes blind.
      Vector3D separation = new Vector3D();
      separation.sub(subtreeCom, bodyOwnCom);
      assertTrue(separation.norm() > 0.5,
                 "Test model is degenerate: whole-robot CoM and foot CoM are only " + separation.norm() + " m apart.");
      assertTrue(Math.abs(force.getZ()) > 100.0, "expected the foot to be carrying real load, got Fz=" + force.getZ());

      int base = footBodyId * 6;
      Vector3D reportedMoment = new Vector3D(data.cfrc_ext().get(base),
                                             data.cfrc_ext().get(base + 1),
                                             data.cfrc_ext().get(base + 2));

      Vector3D expectedAtSubtreeCom = momentAbout(contactPoint, subtreeCom, force);
      assertEquals(expectedAtSubtreeCom.getX(), reportedMoment.getX(), TOLERANCE, "cfrc_ext moment x is not referenced at subtree_com");
      assertEquals(expectedAtSubtreeCom.getY(), reportedMoment.getY(), TOLERANCE, "cfrc_ext moment y is not referenced at subtree_com");
      assertEquals(expectedAtSubtreeCom.getZ(), reportedMoment.getZ(), TOLERANCE, "cfrc_ext moment z is not referenced at subtree_com");

      // The body-CoM reading must be materially wrong, or this test stops discriminating.
      Vector3D expectedAtBodyCom = momentAbout(contactPoint, bodyOwnCom, force);
      Vector3D bias = new Vector3D();
      bias.sub(expectedAtBodyCom, reportedMoment);
      assertTrue(bias.norm() > 1.0,
                 "body-CoM and subtree-CoM references agree here (" + bias.norm() + " N*m), so this test no longer discriminates");

      // And the shift MujocoRobot.updateSensors applies must land on the body origin.
      Vector3D shift = new Vector3D();
      shift.sub(subtreeCom, bodyOrigin);
      Vector3D shifted = new Vector3D();
      shifted.cross(shift, force);
      shifted.add(reportedMoment);
      Vector3D expectedAtOrigin = momentAbout(contactPoint, bodyOrigin, force);
      assertEquals(expectedAtOrigin.getX(), shifted.getX(), TOLERANCE, "shift to body origin x");
      assertEquals(expectedAtOrigin.getY(), shifted.getY(), TOLERANCE, "shift to body origin y");
      assertEquals(expectedAtOrigin.getZ(), shifted.getZ(), TOLERANCE, "shift to body origin z");
   }

   /** m_P = (p_contact - P) x F */
   private static Vector3D momentAbout(Vector3D contactPoint, Vector3D referencePoint, Vector3D force)
   {
      Vector3D lever = new Vector3D();
      lever.sub(contactPoint, referencePoint);
      Vector3D moment = new Vector3D();
      moment.cross(lever, force);
      return moment;
   }

   private static Vector3D point(DoublePointer array, int index)
   {
      return new Vector3D(array.get(index * 3), array.get(index * 3 + 1), array.get(index * 3 + 2));
   }

   private int nameToId(int type, String name)
   {
      int id;
      try (BytePointer pointer = new BytePointer(name))
      {
         id = Mujoco.mj_name2id(model, type, pointer);
      }
      assertTrue(id >= 0, "not found in compiled model: " + name);
      return id;
   }
}

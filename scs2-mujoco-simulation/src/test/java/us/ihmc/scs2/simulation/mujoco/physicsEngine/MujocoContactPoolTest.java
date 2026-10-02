package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * A contact slot has to follow one physical contact. MuJoCo's contact ordering is an arena index
 * free to permute between steps, so a pool that indexes slots by it makes two slots swap the corner
 * they describe mid-stance -- which was observed in a walking simulation, where two {@code dist_*}
 * traces exchanged identity partway through a touchdown.
 */
public class MujocoContactPoolTest
{
   /** Pool capacity for these tests; the graphics assert one group per slot. */
   private static final int POOL_CAPACITY = 8;

   private SimulationSession session;

   @BeforeAll
   public static void loadNatives()
   {
      assertTrue(MujocoNativeLibrary.load(), "MuJoCo native library failed to load");
   }

   @AfterEach
   public void shutdown()
   {
      if (session != null)
         session.shutdownSession();
      session = null;
   }

   /**
    * A box resting flat on the ground holds four corner contacts. Each slot must keep describing the
    * same corner for the whole rest, which shows up as its position staying put and its age rising
    * monotonically.
    */
   @Test
   public void testSlotsKeepTheSameContactWhileItPersists()
   {
      createRestingBox();
      simulate(200); // Settle onto the ground.

      Map<String, Point3D> positionAtSettle = new HashMap<>();
      Map<String, Double> ageAtSettle = new HashMap<>();
      for (int slot = 0; slot < 8; slot++)
      {
         if (Double.isNaN(value("dist_" + slot)))
            continue;
         positionAtSettle.put("" + slot, new Point3D(value("pos_" + slot + "X"), value("pos_" + slot + "Y"), value("pos_" + slot + "Z")));
         ageAtSettle.put("" + slot, value("age_" + slot));
      }
      assertTrue(positionAtSettle.size() >= 3, "Expected the resting box to hold several contacts, got " + positionAtSettle.size());

      simulate(300);

      for (Map.Entry<String, Point3D> entry : positionAtSettle.entrySet())
      {
         String slot = entry.getKey();
         Point3D now = new Point3D(value("pos_" + slot + "X"), value("pos_" + slot + "Y"), value("pos_" + slot + "Z"));
         assertTrue(now.distance(entry.getValue()) < 0.01,
                    "Slot " + slot + " changed which contact it describes: was " + entry.getValue() + ", now " + now);
         assertTrue(value("age_" + slot) > ageAtSettle.get(slot),
                    "Slot " + slot + " age did not keep rising, so the contact was treated as new");
      }
   }

   /** A slot freed when its contact ends must reset its age, so age is a true persistence count. */
   @Test
   public void testAgeResetsWhenTheContactEnds()
   {
      createRestingBox();
      simulate(200);
      int occupied = 0;
      for (int slot = 0; slot < 8; slot++)
         if (!Double.isNaN(value("dist_" + slot)))
            occupied++;
      assertTrue(occupied > 0, "No contacts were recorded at all");

      // Launch the box clear of the ground; every contact ends.
      ((us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics) robot().getJoint("root")).getJointTwist()
                                                                                                                  .getLinearPart()
                                                                                                                  .setZ(6.0);
      simulate(60);

      for (int slot = 0; slot < 8; slot++)
      {
         assertEquals(0.0, value("age_" + slot), 0.0, "Slot " + slot + " kept a non-zero age after its contact ended");
         assertTrue(Double.isNaN(value("dist_" + slot)), "Slot " + slot + " kept a distance after its contact ended");
      }
   }

   private us.ihmc.scs2.simulation.robot.Robot robot()
   {
      return (us.ihmc.scs2.simulation.robot.Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private double value(String name)
   {
      YoVariable v = session.getRootRegistry().findVariable(name);
      assertTrue(v != null, "No such variable: " + name);
      return v.getValueAsDouble();
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   /**
    * As {@link #createRestingBox()} but the body's geom is a SPHERE, so MuJoCo sorts the sphere ahead
    * of the box terrain in the contact pair and the raw contact normal points DOWN. The box case
    * cannot catch a sign error that only appears under that reordering.
    */
   private void createRestingSphere()
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      parameters.setPerContactDiagnosticsCapacity(POOL_CAPACITY);
      session = new SimulationSession((frame, registry) -> new MujocoPhysicsEngine(frame, registry, parameters));
      session.setSessionDTSeconds(1.0e-3);

      VisualDefinitionFactory ground = new VisualDefinitionFactory();
      ground.addBox(10.0, 10.0, 0.1, ColorDefinitions.Grey());
      session.addTerrainObject(new TerrainObjectDefinition(ground.getVisualDefinitions().get(0),
                                                           new CollisionShapeDefinition(new RigidBodyTransform(),
                                                                                        new Box3DDefinition(10.0, 10.0, 0.1))));

      RobotDefinition robot = new RobotDefinition("sphere");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition root = new SixDoFJointDefinition("root");
      SixDoFJointState initial = new SixDoFJointState();
      initial.setConfiguration(null, new Point3D(0.0, 0.0, 0.16));
      root.setInitialJointState(initial);
      elevator.addChildJoint(root);
      RigidBodyDefinition body = new RigidBodyDefinition("body");
      body.setMass(5.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.05, 0.05, 0.05));
      body.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(), new Sphere3DDefinition(0.05)));
      root.setSuccessor(body);
      robot.setRootBodyDefinition(elevator);
      session.addRobot(robot);
      session.initializeBufferSize(16);
   }

   private void createRestingBox()
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      parameters.setPerContactDiagnosticsCapacity(POOL_CAPACITY);
      session = new SimulationSession((frame, registry) -> new MujocoPhysicsEngine(frame, registry, parameters));
      session.setSessionDTSeconds(1.0e-3);

      VisualDefinitionFactory ground = new VisualDefinitionFactory();
      ground.addBox(10.0, 10.0, 0.1, ColorDefinitions.Grey());
      session.addTerrainObject(new TerrainObjectDefinition(ground.getVisualDefinitions().get(0),
                                                           new CollisionShapeDefinition(new RigidBodyTransform(),
                                                                                        new Box3DDefinition(10.0, 10.0, 0.1))));

      RobotDefinition robot = new RobotDefinition("box");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition root = new SixDoFJointDefinition("root");
      SixDoFJointState initial = new SixDoFJointState();
      initial.setConfiguration(null, new Point3D(0.0, 0.0, 0.16));
      root.setInitialJointState(initial);
      elevator.addChildJoint(root);
      RigidBodyDefinition body = new RigidBodyDefinition("body");
      body.setMass(5.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.05, 0.05, 0.05));
      body.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(), new Box3DDefinition(0.3, 0.2, 0.1)));
      root.setSuccessor(body);
      robot.setRootBodyDefinition(elevator);
      session.addRobot(robot);
      session.initializeBufferSize(16);
   }

   /**
    * The graphical form of the pool. The contract that makes it correct with no show/hide logic is
    * that a free slot holds NaN, so its graphic is simply not drawn -- assert both the structure and
    * that contract, since a slot that cleared to zero instead would pile every unused contact at the
    * world origin.
    */
   @Test
   public void testContactYoGraphicsCoverEverySlotAndHideWhenFree()
   {
      createRestingBox();
      YoGraphicDefinition graphics = ((MujocoPhysicsEngine) session.getPhysicsEngine()).getContactYoGraphics();
      assertNotNull(graphics, "Expected contact yoGraphics with the pool enabled");
      assertTrue(graphics instanceof YoGraphicGroupDefinition, "Expected a group, got " + graphics.getClass().getSimpleName());

      // One child group per slot, each holding a point and a force arrow.
      List<YoGraphicDefinition> slotGroups = ((YoGraphicGroupDefinition) graphics).getChildren();
      assertEquals(POOL_CAPACITY, slotGroups.size(), "Expected one graphic group per pool slot");
      for (YoGraphicDefinition slotGroup : slotGroups)
         assertEquals(2, ((YoGraphicGroupDefinition) slotGroup).getChildren().size(), "Expected a point and an arrow per slot");

      // Before anything touches, every slot is free, so every drawn position must be NaN.
      simulate(1);
      assertTrue(Double.isNaN(value("pos_0X")), "A free slot's position must be NaN so its graphic is not drawn");
      assertTrue(Double.isNaN(value("normalForceVector_0X")), "A free slot's force vector must be NaN");

      // Once it lands, the occupied slot carries a real position and a force vector along the normal.
      simulate(600);
      assertTrue(Double.isFinite(value("pos_0Z")), "An occupied slot must carry a finite position");
      double forceZ = value("normalForceVector_0Z");
      double normalForce = value("normalForce_0");
      assertTrue(Double.isFinite(forceZ), "An occupied slot must carry a finite force vector");
      // The box rests on the ground, so the normal points up and the vector's length is the force.
      assertEquals(normalForce, Math.abs(forceZ), 1.0e-6 * Math.max(1.0, normalForce), "Force vector length should equal the normal force");
   }

   /**
    * The force arrow must point UP out of the ground for a resting body, whatever shape its geom is.
    * <p>
    * This is a regression test for a real defect: {@code mjContact.frame[0..2]} points from geom A to
    * geom B, and the A/B order is not physical -- MuJoCo sorts each pair to match its
    * collision-function table, keyed by geom type with the lower {@code mjGEOM_*} index first. A BOX
    * on a BOX terrain keeps the terrain as A (equal types, ordered by id) and the normal points up; a
    * SPHERE (type 2) on a BOX terrain (type 6) makes the SPHERE geom A, so the identical normal points
    * DOWN. The original graphic took the raw normal and so drew the arrows into the ground for sphere
    * feet while looking correct for box feet, which is exactly why this test uses a sphere.
    */
   @Test
   public void testForceArrowPointsOutOfTheGroundForBothGeomTypes()
   {
      createRestingBox();
      simulate(600);
      double boxForceZ = value("normalForceVector_0Z");
      double boxNormalForce = value("normalForce_0");
      assertTrue(boxForceZ > 0.0, "Box: force arrow should point up out of the ground, got Z = " + boxForceZ);
      assertEquals(boxNormalForce, Math.abs(boxForceZ), 1.0e-6 * Math.max(1.0, boxNormalForce), "Box: arrow length should be the normal force");
      session.shutdownSession();

      createRestingSphere();
      simulate(600);
      double sphereForceZ = value("normalForceVector_0Z");
      double sphereNormalForce = value("normalForce_0");
      // Without the world-body sign correction this reads about -sphereNormalForce.
      assertTrue(sphereForceZ > 0.0, "Sphere: force arrow should point up out of the ground, got Z = " + sphereForceZ);
      assertEquals(sphereNormalForce,
                   Math.abs(sphereForceZ),
                   1.0e-6 * Math.max(1.0, sphereNormalForce),
                   "Sphere: arrow length should be the normal force");
   }
}

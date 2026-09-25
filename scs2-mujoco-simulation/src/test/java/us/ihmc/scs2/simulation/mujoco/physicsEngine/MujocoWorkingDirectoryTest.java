package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;

/**
 * The MJCF is compiled out of a virtual file system, so a world of primitive collision shapes should
 * touch the filesystem not at all -- previously every session left a temp directory behind, one per
 * test across a CI run.
 */
public class MujocoWorkingDirectoryTest
{
   private static final String WORKING_DIRECTORY_PROPERTY = "scs2.mujoco.workingDirectory";

   private SimulationSession session;
   private String previousProperty;

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
      if (previousProperty == null)
         System.clearProperty(WORKING_DIRECTORY_PROPERTY);
      else
         System.setProperty(WORKING_DIRECTORY_PROPERTY, previousProperty);
      previousProperty = null;
   }

   private static RobotDefinition createRobot(boolean withInlineMesh)
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition("root");
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition body = new RigidBodyDefinition("body");
      body.setMass(1.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      if (withInlineMesh)
      {
         // A convex polytope becomes an inline vertex cloud in the MJCF, so it exercises the mesh
         // asset path without any file being staged.
         ConvexPolytope3D tetrahedron = EuclidPolytopeFactories.newIcosahedron(0.1);
         body.addCollisionShapeDefinition(new CollisionShapeDefinition(new ConvexPolytope3DDefinition(tetrahedron)));
      }
      else
      {
         body.addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(0.1, 0.1, 0.1)));
      }
      rootJoint.setSuccessor(body);

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, 1.0));
      rootJoint.setInitialJointState(initialState);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private void createSessionAndStep(boolean withInlineMesh)
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame,
                                                                                                rootRegistry,
                                                                                                new MujocoSimulationParameters()));
      session.addRobot(createRobot(withInlineMesh));
      session.setSessionDTSeconds(1.0e-3);
      session.setGravity(0.0, 0.0, -9.81);
      session.initializeBufferSize(1000);
      assertTrue(session.getSimulationSessionControls().simulateNow(10), "Simulation reported a failure");
   }

   private static long countStaleWorkingDirectories() throws Exception
   {
      Path tmp = Path.of(System.getProperty("java.io.tmpdir"));
      try (var entries = Files.list(tmp))
      {
         return entries.filter(Files::isDirectory).filter(path -> path.getFileName().toString().startsWith("scs2-mujoco-")).count();
      }
   }

   @Test
   public void testPrimitiveWorldCreatesNoDirectory() throws Exception
   {
      long before = countStaleWorkingDirectories();
      createSessionAndStep(false);
      assertEquals(before, countStaleWorkingDirectories(), "Compiling a primitive-only world left a working directory behind");
   }

   @Test
   public void testInlineMeshWorldCreatesNoDirectory() throws Exception
   {
      long before = countStaleWorkingDirectories();
      createSessionAndStep(true);
      assertEquals(before, countStaleWorkingDirectories(), "An inline mesh should not need a file staged");
   }

   /** The debugging affordance has to survive: ask for a directory and the MJCF is still written. */
   @Test
   public void testConfiguredWorkingDirectoryStillWritesTheMjcf() throws Exception
   {
      Path directory = Files.createTempDirectory("mujoco-workdir-test-");
      previousProperty = System.getProperty(WORKING_DIRECTORY_PROPERTY);
      System.setProperty(WORKING_DIRECTORY_PROPERTY, directory.toString());

      createSessionAndStep(false);

      File mjcfFile = new File(directory.toFile(), "world.xml");
      assertTrue(mjcfFile.isFile(), "Expected the MJCF to be written to the configured directory");
      String mjcf = Files.readString(mjcfFile.toPath());
      assertTrue(mjcf.contains("meshdir="), "The MJCF must carry an absolute meshdir, since a VFS entry has no directory of its own");
   }
}

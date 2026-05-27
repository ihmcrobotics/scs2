package us.ihmc.scs2.simulation.mujoco;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoPhysicsEngineFactory;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;

/**
 * Same scene as {@code SphereAtRestExperimentalSimulation}, driven through
 * {@link us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoPhysicsEngine} with the canonical
 * {@link PushRobotControllerSCS2} attached. Right-click -> Run; in the visualizer toolbar a
 * "PushRobot" button is added (via {@code addPushButtonToSCS}) for one-click pushes. The push
 * direction and magnitude are tunable in the YoVariable tree under
 * {@code Sphere_PushRobotControllerSCS2}.
 */
public class MujocoSphereFallDemo
{
   public static void main(String[] args)
   {
      double sphereRadius = 0.5;
      double sphereMass = 1.0;

      RobotDefinition sphereRobot = newSphereRobot("Sphere", sphereRadius, sphereMass);

      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(null, new Point3D(0, 0, sphereRadius + 0.1));
      sphereRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.appendTranslation(0, 0, -0.05);
      GeometryDefinition terrainGeometry = new Box3DDefinition(100.0, 100.0, 0.1);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(
            new VisualDefinition(terrainPose, terrainGeometry, new MaterialDefinition(ColorDefinitions.Grey())),
            new CollisionShapeDefinition(terrainPose, terrainGeometry));

      // MuJoCo timestep MUST equal the SCS2 session DT (otherwise MuJoCo runs at a different
      // speed than wall clock). SCS2's default session DT is 1e-4 s (10 kHz); we bump both
      // session DT and MuJoCo timestep to 1 ms here. With substeps=1 each session tick advances
      // MuJoCo exactly one mj_step.
      double dt = 0.001;
      MujocoSimulationParameters mujocoParams = new MujocoSimulationParameters();
      mujocoParams.setTimestep(dt);

      SimulationConstructionSet2 scs = new SimulationConstructionSet2(MujocoPhysicsEngineFactory.newMujocoPhysicsEngineFactory(mujocoParams));
      scs.getSimulationSession().setSessionDTSeconds(dt);
      // Default buffer is 8192 ticks. At 1 ms tick that's ~8 s of history. Bump to 65536 (~65 s)
      // so the user can rewind through an entire rolloff.
      scs.changeBufferSize(65536);

      Robot liveRobot = scs.addRobot(sphereRobot);
      scs.addTerrainObject(terrain);

      // Last arg is `visualScale` -- with `scaleLength=true` in PushRobotControllerSCS2's arrow
      // graphic, actual arrow length ~= visualScale * 0.9 * forceMagnitude. The default 0.005
      // gives a 3 cm arrow at 6.25 N, hidden inside the 0.5 m sphere. 0.2 yields ~1.1 m so the
      // arrow extends ~2x the sphere radius.
      PushRobotControllerSCS2 pushController = new PushRobotControllerSCS2(scs.getTime(),
                                                                            liveRobot,
                                                                            "Sphere",
                                                                            new Vector3D(0.0, 0.0, 0.0),
                                                                            0.2);
      // Default push tuning: impulse = magnitude * duration. Sphere mass is 1 kg, terrain edge is
      // 50 m from origin, so initial velocity v0 ~= 2.5 m/s gives ~20 s to roll off the world
      // (ignoring rolling friction, which on a sphere is very small in MuJoCo's defaults).
      // Impulse = 1 kg * 2.5 m/s = 2.5 N*s -> 12.5 N for 0.2 s.
      // Longer duration so the arrow is visible during a push at real-time. Push impulse stays
      // the same by halving force magnitude (1.0 s * 1.25 N = 0.2 s * 6.25 N = 1.25 N*s).
      pushController.setPushDuration(1.0);
      pushController.setPushForceMagnitude(1.25);
      pushController.setPushForceDirection(new Vector3D(1.0, 0.0, 0.0));
      pushController.addPushButtonToSCS(scs);
      // Register the force arrow with the visualizer so it actually renders.
      scs.addYoGraphic(pushController.getForceVizDefinition());

      // Throttle the sim loop so 1 s of sim time takes 1 s of wall time. Without this the engine
      // free-runs as fast as it can.
      scs.setRealTimeRateSimulation(true);
      scs.start(true, false, false);
   }

   private static RobotDefinition newSphereRobot(String name, double radius, double mass)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);

      RigidBodyDefinition sphereBody = new RigidBodyDefinition(name + "RigidBody");
      sphereBody.setMass(mass);
      double radiusOfGyration = 0.8 * radius;
      sphereBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                      radiusOfGyration,
                                                                                      radiusOfGyration,
                                                                                      radiusOfGyration));

      VisualDefinitionFactory visualFactory = new VisualDefinitionFactory();
      visualFactory.addSphere(radius, ColorDefinitions.Maroon());
      sphereBody.addVisualDefinitions(visualFactory.getVisualDefinitions());
      sphereBody.addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(radius)));

      rootJoint.setSuccessor(sphereBody);
      robotDefinition.setRootBodyDefinition(rootBody);
      return robotDefinition;
   }
}

package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

public class RollingObjectsExperimentalSimulation
{
   private static final String BALL_NAME = "ball";
   private static final String CAPSULE_NAME = "capsule";
   private static final String CYLINDER_NAME = "cylinder";
   private static final String BALL_BODY_NAME = BALL_NAME + "RigidBody";
   private static final String CAPSULE_BODY_NAME = CAPSULE_NAME + "RigidBody";
   private static final String CYLINDER_BODY_NAME = CYLINDER_NAME + "RigidBody";


   public RollingObjectsExperimentalSimulation()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setErrorReductionParameter(0.001);

      double ballRadius = 0.2;
      double ballMass = 1.0;
      double ballRadiusOfGyrationPercent = 1.0;

      double cylinderRadius = 0.2;
      double cylinderHeight = 0.5;
      double cylinderMass = 1.0;
      double cylinderRadiusOfGyrationPercent = 1.0;

      double capsuleRadius = 0.2;
      double capsuleHeight = 0.1 + 2.0 * capsuleRadius;
      double capsuleMass = 1.0;
      double capsuleRadiusOfGyrationPercent = 1.0;

      double initialVelocity = 1.0;

      ColorDefinition appearance = ColorDefinitions.DarkCyan();
      boolean addStripes = true;
      ColorDefinition stripesAppearance = ColorDefinitions.Gold();

      RobotDefinition ballRobot = ExampleExperimentalSimulationTools.newSphereRobot(BALL_NAME,
                                                                                    ballRadius,
                                                                                    ballMass,
                                                                                    ballRadiusOfGyrationPercent,
                                                                                    appearance,
                                                                                    addStripes,
                                                                                    stripesAppearance);
      RobotDefinition cylinderRobot = ExampleExperimentalSimulationTools.newCylinderRobot(CYLINDER_NAME,
                                                                                          cylinderRadius,
                                                                                          cylinderHeight,
                                                                                          cylinderMass,
                                                                                          cylinderRadiusOfGyrationPercent,
                                                                                          appearance,
                                                                                          addStripes,
                                                                                          stripesAppearance);
      RobotDefinition capsuleRobot = ExampleExperimentalSimulationTools.newCapsuleRobot(CAPSULE_NAME,
                                                                                        capsuleRadius,
                                                                                        capsuleHeight,
                                                                                        capsuleMass,
                                                                                        capsuleRadiusOfGyrationPercent,
                                                                                        appearance,
                                                                                        addStripes,
                                                                                        stripesAppearance);

      SixDoFJointState ballInitialState = new SixDoFJointState(null, new Point3D(-1.0, -2.0 * ballRadius - cylinderHeight, ballRadius * 1.02));
      ballInitialState.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      ballRobot.getRootJointDefinitions().get(0).setInitialJointState(ballInitialState);

      SixDoFJointState cylinderInitialState = new SixDoFJointState();
      cylinderInitialState.setConfiguration(new Pose3D(-1.0, 0.0, cylinderRadius * 1.02, 0.0, 0.0, Math.PI / 2.0));
      cylinderInitialState.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      cylinderRobot.getRootJointDefinitions().get(0).setInitialJointState(cylinderInitialState);

      SixDoFJointState capsuleInitialState = new SixDoFJointState();
      capsuleInitialState.setConfiguration(new Pose3D(-1.0, cylinderHeight + capsuleHeight + capsuleRadius, capsuleRadius * 1.02, 0.0, 0.0, Math.PI / 2.0));
      capsuleInitialState.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      capsuleRobot.getRootJointDefinitions().get(0).setInitialJointState(capsuleInitialState);

      ballRobot.getRigidBodyDefinition(BALL_BODY_NAME).addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(ballRadius)));
      cylinderRobot.getRigidBodyDefinition(CYLINDER_BODY_NAME)
                   .addCollisionShapeDefinition(new CollisionShapeDefinition(new Cylinder3DDefinition(cylinderHeight, cylinderRadius)));
      capsuleRobot.getRigidBodyDefinition(CAPSULE_BODY_NAME)
                  .addCollisionShapeDefinition(new CollisionShapeDefinition(new Capsule3DDefinition(cylinderHeight, cylinderRadius)));

      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      Box3DDefinition terrainGeometry = new Box3DDefinition(1000.0, 1000.0, 0.1);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.SlateBlue())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters));
      simulationSession.addRobot(ballRobot);
      simulationSession.addRobot(cylinderRobot);
      simulationSession.addRobot(capsuleRobot);
      simulationSession.addTerrainObject(terrain);
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new RollingObjectsExperimentalSimulation();
   }
}

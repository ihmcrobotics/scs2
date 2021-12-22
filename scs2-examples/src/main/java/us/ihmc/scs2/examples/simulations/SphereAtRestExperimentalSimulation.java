package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

public class SphereAtRestExperimentalSimulation
{
   public SphereAtRestExperimentalSimulation()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setErrorReductionParameter(0.01);

      double sphereRadius = 0.5;
      double sphereMass = 1.0;

      RobotDefinition sphereRobot = ExampleExperimentalSimulationTools.newSphereRobot("Sphere",
                                                                                      sphereRadius,
                                                                                      sphereMass,
                                                                                      0.8,
                                                                                      ColorDefinitions.Maroon(),
                                                                                      true,
                                                                                      ColorDefinitions.Gold());
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(null, new Point3D(0, 0, sphereRadius));
      sphereRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      sphereRobot.getRigidBodyDefinition("SphereRigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(sphereRadius)));

      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.appendTranslation(0, 0, -0.05);
      GeometryDefinition terrainGeometry = new Box3DDefinition(100.0, 100.0, 0.1);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.LightGreen())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters));
      simulationSession.addRobot(sphereRobot);
      simulationSession.addTerrainObject(terrain);
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new SphereAtRestExperimentalSimulation();
   }
}

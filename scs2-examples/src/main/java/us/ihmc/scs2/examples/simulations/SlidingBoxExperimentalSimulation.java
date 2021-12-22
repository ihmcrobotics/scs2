package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
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

public class SlidingBoxExperimentalSimulation
{
   public SlidingBoxExperimentalSimulation()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setErrorReductionParameter(0.001);

      Vector3D boxSize = new Vector3D(0.4, 0.4, 0.4);

      double groundPitch = Math.toRadians(34.0);

      RobotDefinition boxRobot = ExampleExperimentalSimulationTools.newBoxRobot("box", boxSize, 150.0, 0.8, ColorDefinitions.DarkCyan());
      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(new YawPitchRoll(0, groundPitch, 0), new Point3D(0, 0, 0.7 * boxSize.getZ()));
      initialState.setVelocity(new Vector3D(0, 0, 0), new Vector3D(0, 0, 0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialState);

      boxRobot.getRigidBodyDefinition("boxRigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSize)));

      GeometryDefinition terrainGeometry = new Box3DDefinition(100.0, 100.0, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.appendPitchRotation(groundPitch);
      terrainPose.appendTranslation(0, 0, -0.05);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.Lavender())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters));
      simulationSession.addRobot(boxRobot);
      simulationSession.addTerrainObject(terrain);
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new SlidingBoxExperimentalSimulation();
   }
}

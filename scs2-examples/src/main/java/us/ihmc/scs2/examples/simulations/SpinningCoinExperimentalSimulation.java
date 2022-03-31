package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

public class SpinningCoinExperimentalSimulation
{
   private static final String SPINNING_COIN = "SpinningCoin";
   private final double coinWidth = 0.00175; //quarter //0.1    
   private final double coinRadius = 0.01213; //0.5; //         
   private final double coinMass = 0.00567; //1.0; //
   private final double spinningAngularVelocity = 2.0 * Math.PI;

   public SpinningCoinExperimentalSimulation()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(0.0);
      contactParameters.setRestitutionThreshold(0.0);
      contactParameters.setErrorReductionParameter(1.0e-3);

      RobotDefinition robotDefinition = new RobotDefinition(SPINNING_COIN);

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      robotDefinition.setRootBodyDefinition(rootBody);

      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition("rootJoint");
      rootBody.addChildJoint(rootJoint);

      RigidBodyDefinition coinBody = new RigidBodyDefinition("coin");
      rootJoint.setSuccessor(coinBody);
      coinBody.setMass(coinMass);
      coinBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(coinMass, coinRadius / 2.0, coinRadius / 2.0, coinWidth / 2.0));

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.appendTranslation(0.0, 0.0, -coinWidth / 2.0);
      factory.addCylinder(coinWidth, coinRadius, ColorDefinitions.Purple());
      factory.identity();
      factory.appendTranslation(0.0, 0.0, coinWidth / 2.0);
      factory.addBox(coinRadius / 3.0, coinRadius / 3.0, coinWidth / 4.0, ColorDefinitions.AliceBlue());
      factory.appendTranslation(0.0, 0.0, -coinWidth - coinWidth / 4.0);
      factory.addBox(coinRadius / 3.0, coinRadius / 3.0, coinWidth / 4.0, ColorDefinitions.Gold());
      coinBody.addVisualDefinitions(factory.getVisualDefinitions());

      coinBody.addCollisionShapeDefinition(new CollisionShapeDefinition(new Cylinder3DDefinition(coinWidth, coinRadius)));

      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new YawPitchRoll(0, 0, 1.2), new Point3D(0.1, 0.1, coinRadius + 0.04));
      initialJointState.setVelocity(new Vector3D(0, spinningAngularVelocity, 0), null);
      rootJoint.setInitialJointState(initialJointState);

      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      GeometryDefinition terrainGeometry = new Box3DDefinition(1000, 1000, 0.1);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkGrey())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters));
      simulationSession.addRobot(robotDefinition);
      simulationSession.addTerrainObject(terrain);
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new SpinningCoinExperimentalSimulation();
   }
}

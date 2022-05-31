package us.ihmc.scs2.examples.simulations.bullet;

import static us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools.newSphereRobot;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationEnergyStatistics;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

public class FlyingCollidingSpheresExperimentalBulletSimulation
{
   public FlyingCollidingSpheresExperimentalBulletSimulation()
   {
      double radius1 = 0.2;
      double mass1 = 1.0;
      double radiusOfGyrationPercent1 = 1.0;
      ColorDefinition appearance1 = ColorDefinitions.DarkGreen();
      ColorDefinition stripesAppearance1 = ColorDefinitions.LightGreen();
      RobotDefinition sphereRobot1 = newSphereRobot("sphere1", radius1, mass1, radiusOfGyrationPercent1, appearance1, true, stripesAppearance1);

      double radius2 = 0.2;
      double mass2 = 1.0;
      double radiusOfGyrationPercent2 = 1.0;
      ColorDefinition appearance2 = ColorDefinitions.DarkRed();
      ColorDefinition stripesAppearance2 = ColorDefinitions.LightSteelBlue();
      RobotDefinition sphereRobot2 = newSphereRobot("sphere2", radius2, mass2, radiusOfGyrationPercent2, appearance2, true, stripesAppearance2);

      sphereRobot1.getRigidBodyDefinition("sphere1RigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(radius1)));
      sphereRobot2.getRigidBodyDefinition("sphere2RigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(radius2)));

      SixDoFJointState sphere1InitialState = new SixDoFJointState();
      sphere1InitialState.setConfiguration(null, new Point3D(-1.3, 1.0, 0.6));
      sphere1InitialState.setVelocity(null, new Vector3D(2.0, 1.0, 0));
      sphereRobot1.getRootJointDefinitions().get(0).setInitialJointState(sphere1InitialState);

      SixDoFJointState sphere2InitialState = new SixDoFJointState();
      sphere2InitialState.setConfiguration(null, new Point3D(+0.2, 1.0, 0.4));
      sphere2InitialState.setVelocity(null, new Vector3D(0.0, 0, 0));
      sphereRobot2.getRootJointDefinitions().get(0).setInitialJointState(sphere2InitialState);

      BulletMultiBodyParameters bulletMultiBodyParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      bulletMultiBodyParameters.setLinearDamping(0);
      bulletMultiBodyParameters.setMaxCoordinateVelocity(100000);
      bulletMultiBodyParameters.setUseRK4Integration(true);
      BulletMultiBodyJointParameters bulletMultiBodyJointParameter = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(bulletMultiBodyParameters, bulletMultiBodyJointParameter));
      //SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory());
      simulationSession.addRobot(sphereRobot1);
      simulationSession.addRobot(sphereRobot2);
      simulationSession.submitBufferSizeRequest(245760);
      simulationSession.setBufferRecordTickPeriod(8);
      //simulationSession.setSessionDTSeconds(0.05);

      SimulationEnergyStatistics.setupSimulationEnergyStatistics(simulationSession);

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new FlyingCollidingSpheresExperimentalBulletSimulation();
   }
}

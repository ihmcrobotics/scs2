package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.SimulationEnergyStatistics;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;

public class BulletFlyingBallSimulationTest
{
   private static final double EPSILON = 0.01;
   private static final boolean BULLET_PHYSICS_ENGINE = true;
   
   @Test
   public void testFlyingBall()
   {
      Point3D initialPosition = new Point3D(0, 0, 0);
      Vector3D initialVelocity = new Vector3D(0.0, 0.0, 0);

      double dt = 0.01;

      double ballRadius = 0.2;
      double ballMass = 2.0;
      String name = "sphere";

      RobotDefinition sphereRobot1 = new RobotDefinition(name);
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
      rigidBody.setMass(ballMass);
      rigidBody.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.1));
      rootJoint.setSuccessor(rigidBody);

      sphereRobot1.setRootBodyDefinition(rootBody);
      sphereRobot1.getRigidBodyDefinition(name + "RigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(ballRadius)));

      SixDoFJointState sphereInitialState = new SixDoFJointState();
      sphereInitialState.setConfiguration(null, initialPosition);
      sphereInitialState.setVelocity(null, initialVelocity);
      sphereRobot1.getRootJointDefinitions().get(0).setInitialJointState(sphereInitialState);
       
      SimulationSession simulationSession = null;
      if (BULLET_PHYSICS_ENGINE)
      {
         BulletMultiBodyParameters bulletMultiBodyParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
         BulletMultiBodyJointParameters bulletMultiBodyJointParameter = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
         bulletMultiBodyParameters.setLinearDamping(0);
         bulletMultiBodyParameters.setMaxCoordinateVelocity(10000000);
         bulletMultiBodyParameters.setUseRK4Integration(true);
         simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(bulletMultiBodyParameters, bulletMultiBodyJointParameter));
      }
      else
      {
         simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory());
      }
      
      simulationSession.addRobot(sphereRobot1);
      simulationSession.setSessionDTSeconds(dt);
      SimulationEnergyStatistics.setupSimulationEnergyStatistics(simulationSession);
      SimFloatingRootJoint floatingRootJoint = (SimFloatingRootJoint)simulationSession.getPhysicsEngine().getRobots().get(0).getJoint(name);
      
      double orbitalEnergy = 0;
      for (int i = 1; i <= 1000; i++)
      {
         simulationSession.runTick();

         Vector3D height = heightAfterSeconds(initialPosition, initialVelocity, dt * i, simulationSession.getGravity().getZ());

         assertEquals(floatingRootJoint.getJointPose().getX(), height.getX(), EPSILON, "X failed at iteration = " + i);
         assertEquals(floatingRootJoint.getJointPose().getY(), height.getY(), EPSILON, "Y failed at iteration = " + i);
         assertEquals(floatingRootJoint.getJointPose().getZ(), height.getZ(), 0.1, "Z failed at iteration = " + i);
         
         //orbital energy should remain constant
         if (orbitalEnergy == 0)
            orbitalEnergy = simulationSession.getRootRegistry().findVariable(name + "OrbitalEnergy").getValueAsDouble();
         assertEquals(simulationSession.getRootRegistry().findVariable(name + "OrbitalEnergy").getValueAsDouble(), orbitalEnergy, 0.1, "Orbital Energy failed at iteration = " + i);
      }
   }

   private Vector3D heightAfterSeconds(Point3D initialPosition, Vector3D initialVelocity, double seconds, double gravity)
   {
      //H(time) = -1/2 * g * t^2 + V(initial) * t + H(initial)
      Vector3D height = new Vector3D();
      
      height.setX(initialVelocity.getX() * seconds + initialPosition.getX());
      height.setY(initialVelocity.getY() * seconds + initialPosition.getY());
      height.setZ(0.5 * gravity * seconds * seconds + initialVelocity.getZ() * seconds + initialPosition.getZ());

      return height;
   }
}

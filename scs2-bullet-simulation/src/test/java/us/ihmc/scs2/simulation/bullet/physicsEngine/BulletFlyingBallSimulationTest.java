package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationEnergyStatistics;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Tests the ballistics equation:
 *
 * H(time) = -1/2 * g * t2 + V(initial) * t + H(initial)
 *
 * and asserts that the Orbital Energy remains constant.
 */
public class BulletFlyingBallSimulationTest
{
   private static final double EPSILON = 0.01;
   private static final boolean BULLET_PHYSICS_ENGINE = true;
   private static final boolean VISUALIZE = false;
   private static final int numberOfSimulationTicks = 1000;
   private YoPoint3D expectedPosition;
   private SimFloatingRootJoint floatingRootJoint;
   private YoDouble orbitalEnergyVariable;

   public SimulationSession createSession()
   {
      Point3D initialPosition = new Point3D(0.0, 0.0, 0.0);
      Vector3D initialVelocity = new Vector3D(0.0, 0.0, 0.0);

      double dt = 0.01;

      double ballRadius = 0.2;
      double ballMass = 2.0;
      String name = "sphere";

      RobotDefinition sphereRobot = new RobotDefinition(name);
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);

      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
      rigidBody.setMass(ballMass);
      rigidBody.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.1));
      rigidBody.addVisualDefinition(new VisualDefinition(new Sphere3DDefinition(0.05),
                                                         new MaterialDefinition(null, ColorDefinitions.Brown(), ColorDefinitions.LightGray(), null, 10)));
      rootJoint.setSuccessor(rigidBody);

      sphereRobot.setRootBodyDefinition(rootBody);
      sphereRobot.getRigidBodyDefinition(name + "RigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(ballRadius)));

      SixDoFJointState sphereInitialState = new SixDoFJointState();
      sphereInitialState.setConfiguration(null, initialPosition);
      sphereInitialState.setVelocity(null, initialVelocity);
      sphereRobot.getRootJointDefinitions().get(0).setInitialJointState(sphereInitialState);

      SimulationSession simulationSession = null;
      if (BULLET_PHYSICS_ENGINE)
      {
         BulletMultiBodyParameters bulletMultiBodyParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
         bulletMultiBodyParameters.setLinearDamping(0);
         bulletMultiBodyParameters.setMaxCoordinateVelocity(10000000);
         bulletMultiBodyParameters.setUseRK4Integration(true);
         BulletMultiBodyJointParameters bulletMultiBodyJointParameter = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
         simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(bulletMultiBodyParameters,
                                                                                                            bulletMultiBodyJointParameter));
      }
      else
      {
         simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory());
      }

      simulationSession.addRobot(sphereRobot);
      simulationSession.setSessionDTSeconds(dt);
      simulationSession.initializeBufferSize(2 * numberOfSimulationTicks);
      SimulationEnergyStatistics.setupSimulationEnergyStatistics(simulationSession);

      expectedPosition = new YoPoint3D("expectedSpherePosition", simulationSession.getRootRegistry());
      FrameVector3DReadOnly gravity = simulationSession.getGravity();

      floatingRootJoint = (SimFloatingRootJoint) simulationSession.getPhysicsEngine().getRobots().get(0).getJoint(name);

      orbitalEnergyVariable = (YoDouble) simulationSession.getRootRegistry().findVariable(name + "OrbitalEnergy");

      simulationSession.addAfterPhysicsCallback(time -> expectedPosition.set(heightAfterSeconds(initialPosition, initialVelocity, time, gravity.getZ())));
      return simulationSession;
   }

   @Test
   public void testFlyingBall() throws Throwable
   {
      SimulationSession simulationSession = createSession();

      simulationSession.addAfterPhysicsCallback(new TimeConsumer()
      {
         double initialOrbitalEnergy = 0;

         @Override
         public void accept(double time)
         {
            EuclidCoreTestTools.assertEquals(expectedPosition, floatingRootJoint.getJointPose().getPosition(), EPSILON);

            //orbital energy should remain constant
            if (initialOrbitalEnergy == 0)
               initialOrbitalEnergy = orbitalEnergyVariable.getValue();
            assertEquals(initialOrbitalEnergy, orbitalEnergyVariable.getValue(), 0.1, "Orbital Energy failed at time = " + time);
         }
      });

      MutableObject<Throwable> caughtException = new MutableObject<>(null);
      simulationSession.addRunThrowableListener(t -> caughtException.setValue(t));

      SessionVisualizerControls visualizerControls = null;
      if (VISUALIZE)
      {
         visualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession);
         visualizerControls.waitUntilVisualizerFullyUp();
      }

      try
      {
         simulationSession.getSimulationSessionControls().simulateNow(numberOfSimulationTicks);
         if (caughtException.getValue() != null)
            throw caughtException.getValue();
      }
      finally
      {
         if (VISUALIZE)
         {
            visualizerControls.waitUntilVisualizerDown();
         }
      }

      simulationSession.shutdownSession();
   }

   private static Vector3D heightAfterSeconds(Point3D initialPosition, Vector3D initialVelocity, double seconds, double gravity)
   {
      //H(time) = -1/2 * g * t^2 + V(initial) * t + H(initial)
      Vector3D height = new Vector3D();

      height.setX(initialVelocity.getX() * seconds + initialPosition.getX());
      height.setY(initialVelocity.getY() * seconds + initialPosition.getY());
      height.setZ(0.5 * gravity * seconds * seconds + initialVelocity.getZ() * seconds + initialPosition.getZ());

      return height;
   }
}

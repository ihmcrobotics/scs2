package us.ihmc.scs2.simulation.bullet.physicsEngine;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletContactSolverInfoParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;

public class BulletCoefficientOfRestitutionTest
{
   private static final int ITERATIONS = 100;
   private static final int NUMBER_OF_TRIES = 1000;
   private static final boolean BULLET_PHYSICS_ENGINE = true;
   private static final double EPSILON = 1e-3;

   private static final double DT = 0.1;
   private static final String BALL_NAME1 = "ball1";
   private static final String BALL_NAME2 = "ball2";
   private static final MomentOfInertiaDefinition MOMENT_OF_INERTIA = new MomentOfInertiaDefinition(0.1, 0.1, 0.1);
   private static final double BALL_RADIUS1 = 0.4;
   private static final double BALL_RADIUS2 = 0.2;
   private static final double BALL_MASS1 = 2;
   private static final double BALL_MASS2 = 2;

   private static Vector3D initialVelocity1 = new Vector3D();
   private static Vector3D initialVelocity2 = new Vector3D();
   private static Vector3D finalVelocity1 = new Vector3D();
   private static Vector3D finalVelocity2 = new Vector3D();
   private static Vector3D differenceInitialVelocity = new Vector3D();
   private static Vector3D differenceFinalVelocity = new Vector3D();
   private static Point3D initialPosition1 = new Point3D();
   private static Point3D initialPosition2 = new Point3D();

   @Test
   public void testFlyingCollidingSpheres()
   {
      Random random = new Random(1254257L);

      //The Coefficient of Restitution can be between 0 and 1 - Make sure the end points are tested
      testCoefficientOfRestitution(0, random, true);
      testCoefficientOfRestitution(1, random, true);

      testCoefficientOfRestitution(0, random, false);
      testCoefficientOfRestitution(1, random, false);

      for (int i = 0; i <= ITERATIONS; i++)
      {
         testCoefficientOfRestitution(random.nextDouble(), random, true);
      }

      for (int i = 0; i <= ITERATIONS; i++)
      {
         testCoefficientOfRestitution(random.nextDouble(), random, false);
      }
   }

   private static void testCoefficientOfRestitution(double coefficientOfRestitution, Random random, boolean testHeadOnCollision)
   {
      double x1 = random.nextDouble();
      double y1 = random.nextDouble();
      double z1 = random.nextDouble();

      //Make sure the balls are far enough apart and do not overlap
      double x2 = random.nextDouble() + 2 * (x1 + BALL_RADIUS1 + BALL_RADIUS2);
      double y2 = random.nextDouble() + 2 * (y1 + BALL_RADIUS1 + BALL_RADIUS2);
      double z2 = random.nextDouble() + 2 * (z1 + BALL_RADIUS1 + BALL_RADIUS2);

      initialPosition1.set(x1, y1, z1);
      initialPosition2.set(x2, y2, z2);

      if (testHeadOnCollision)
      {
         //Balls travel directly towards each other
         initialVelocity1.sub(initialPosition2, initialPosition1);
         initialVelocity1.scale(random.nextDouble());

         initialVelocity2.sub(initialPosition1, initialPosition2);
         initialVelocity2.scale(random.nextDouble());
      }
      else
      {
         //Balls travel in the same direction with one at a slower speed
         initialVelocity1.sub(initialPosition2, initialPosition1);

         initialVelocity2.set(initialVelocity1);
         
         initialVelocity2.scale(BALL_RADIUS2);
      }

      RobotDefinition sphereRobot1 = createSphereRobot(BALL_RADIUS1, BALL_MASS1, BALL_NAME1, MOMENT_OF_INERTIA, initialPosition1, initialVelocity1);

      RobotDefinition sphereRobot2 = createSphereRobot(BALL_RADIUS2, BALL_MASS2, BALL_NAME2, MOMENT_OF_INERTIA, initialPosition2, initialVelocity2);

      SimulationSession simulationSession = null;
      if (BULLET_PHYSICS_ENGINE)
      {
         BulletMultiBodyParameters bulletMultiBodyParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
         BulletMultiBodyJointParameters bulletMultiBodyJointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
         BulletContactSolverInfoParameters bulletContactSolverInfoParameters = BulletContactSolverInfoParameters.defaultBulletContactSolverInfoParameters();
         bulletMultiBodyParameters.setLinearDamping(0.0);
         bulletMultiBodyParameters.setAngularDamping(0.0);
         bulletMultiBodyJointParameters.setJointRestitution(coefficientOfRestitution);
         bulletContactSolverInfoParameters.setSplitImpulse(1);
         bulletContactSolverInfoParameters.setSplitImpulseTurnErp(1.0f);
         bulletContactSolverInfoParameters.setSplitImpulsePenetrationThreshold(-0.0000001f);
         bulletContactSolverInfoParameters.setErrorReductionForNonContactConstraints(0);
         bulletContactSolverInfoParameters.setErrorReductionForContactConstraints(0);

         simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(bulletMultiBodyParameters,
                                                                                                            bulletMultiBodyJointParameters,
                                                                                                            bulletContactSolverInfoParameters));
      }
      else
      {
         ContactParameters contactParameters = new ContactParameters();
         contactParameters.setCoefficientOfRestitution(coefficientOfRestitution);
         simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters));
      }

      simulationSession.addRobot(sphereRobot1);
      simulationSession.addRobot(sphereRobot2);
      simulationSession.setSessionDTSeconds(DT);
      simulationSession.setGravity(0.0f, 0.0f, 0.0f);

      SimFloatingRootJoint floatingRootJoint1 = (SimFloatingRootJoint) simulationSession.getPhysicsEngine().getRobots().get(0).getAllJoints().get(0);
      SimFloatingRootJoint floatingRootJoint2 = (SimFloatingRootJoint) simulationSession.getPhysicsEngine().getRobots().get(1).getAllJoints().get(0);

      finalVelocity1.set(initialVelocity1);
      finalVelocity2.set(initialVelocity2);

      int j = 0;
      while (j < NUMBER_OF_TRIES && initialVelocity1.epsilonEquals(finalVelocity1, EPSILON))
      {
         simulationSession.runTick();

         finalVelocity1.set(floatingRootJoint1.getSuccessor().getBodyFixedFrame().getTwistOfFrame().getLinearPart());
         finalVelocity2.set(floatingRootJoint2.getSuccessor().getBodyFixedFrame().getTwistOfFrame().getLinearPart());

         j++;
      }

      //Collision happened
      if (!initialVelocity1.epsilonEquals(finalVelocity1, EPSILON))
      {
         differenceInitialVelocity.set(initialVelocity1);
         differenceInitialVelocity.sub(initialVelocity2);

         differenceFinalVelocity.set(finalVelocity1);
         differenceFinalVelocity.sub(finalVelocity2);
         
         double calculatedCoefficientOfRestitution = differenceFinalVelocity.norm() / differenceInitialVelocity.norm();

         assertEquals(coefficientOfRestitution * coefficientOfRestitution, calculatedCoefficientOfRestitution, EPSILON);
      }
      else
      {
         System.out.println("No Collision");
      }
   }

   private static RobotDefinition createSphereRobot(double radius,
                                                    double mass,
                                                    String name,
                                                    MomentOfInertiaDefinition momentOfInertia,
                                                    Point3D initialPosition,
                                                    Vector3D initialVelocity)
   {
      RobotDefinition sphereRobot = new RobotDefinition(name);
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);

      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(momentOfInertia);
      rootJoint.setSuccessor(rigidBody);

      sphereRobot.setRootBodyDefinition(rootBody);
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Sphere3DDefinition(radius));
      sphereRobot.getRigidBodyDefinition(name + "RigidBody").addCollisionShapeDefinition(collisionShapeDefinition);

      SixDoFJointState sphereInitialState = new SixDoFJointState();
      sphereInitialState.setConfiguration(null, initialPosition);
      sphereInitialState.setVelocity(null, initialVelocity);
      sphereRobot.getRootJointDefinitions().get(0).setInitialJointState(sphereInitialState);

      return sphereRobot;
   }
}

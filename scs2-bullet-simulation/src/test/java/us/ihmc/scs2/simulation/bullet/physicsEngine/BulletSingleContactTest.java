package us.ihmc.scs2.simulation.bullet.physicsEngine;

import org.junit.jupiter.api.Test;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;

import static org.junit.jupiter.api.Assertions.*;
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
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;

public class BulletSingleContactTest
{
   private static final int NUMBER_OF_TRIES = 10000;
   private static final boolean BULLET_PHYSICS_ENGINE = true;
   private static final double EPSILON = 0.06;
   
   @Test
   public void testFlyingCollidingSpheres()
   {
      //Random random = new Random(12541257L);

      //double dt = 1.0 / 60.0;
      double dt = 0.001;
      String name1 = "sphere1";
      String name2 = "sphere2";
      double radius1 = 0.2;
      double radius2 = 0.2;
      double mass1 = 2;
      double mass2 = 2;
      double initialVelocity1 = -1.0;
      double initialVelocity2 = 2.0;
      
      double coefficientOfRestitution = 0.2;

      RobotDefinition sphereRobot1 = createSphereRobot(radius1,
                                                       mass1,
                                                       name1,
                                                       new MomentOfInertiaDefinition(0.1, 0.1, 0.1),
                                                       new Point3D(0.2, 3.0, 0.6),
                                                       new Vector3D(0.0, initialVelocity1, 0.0));
      
      RobotDefinition sphereRobot2 = createSphereRobot(radius2,
                                                       mass2,
                                                       name2,
                                                       new MomentOfInertiaDefinition(0.1, 0.1, 0.1),
                                                       new Point3D(0.2, -3.0, 0.6),
                                                       new Vector3D(0.0, initialVelocity2, 0.0));
      
      SimulationSession simulationSession = null;
      if (BULLET_PHYSICS_ENGINE)
      {
         BulletMultiBodyParameters bulletMultiBodyParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
         BulletMultiBodyJointParameters bulletMultiBodyJointParameter = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
         bulletMultiBodyParameters.setLinearDamping(0.0);
         bulletMultiBodyJointParameter.setJointRestitution(coefficientOfRestitution);
            
         simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(bulletMultiBodyParameters, bulletMultiBodyJointParameter));
      }
      else
      {
         ContactParameters contactParameters = new ContactParameters();
         contactParameters.setCoefficientOfRestitution(coefficientOfRestitution);
         simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters));
      }
      
      simulationSession.addRobot(sphereRobot1);
      simulationSession.addRobot(sphereRobot2);
      simulationSession.setSessionDTSeconds(dt);
      simulationSession.setGravity(0.0f, 0.0f, 0.0f);
      
      BulletPhysicsEngine bulletPhysicsEngine = null;
      if (BULLET_PHYSICS_ENGINE)
      {
          bulletPhysicsEngine = (BulletPhysicsEngine)simulationSession.getPhysicsEngine();
//         bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld().getBtMultiBodyDynamicsWorld().getSolverInfo().setSplitImpulse(0);
//         bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld().getBtMultiBodyDynamicsWorld().getSolverInfo().setSplitImpulseTurnErp(1.0f);
//         bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld().getBtMultiBodyDynamicsWorld().getSolverInfo().setSplitImpulsePenetrationThreshold(-0.0000001f);
      }
      
      SimFloatingRootJoint floatingRootJoint1 = (SimFloatingRootJoint)simulationSession.getPhysicsEngine().getRobots().get(0).getAllJoints().get(0);
      SimFloatingRootJoint floatingRootJoint2 = (SimFloatingRootJoint)simulationSession.getPhysicsEngine().getRobots().get(1).getAllJoints().get(0);
      
      Double finalVelocity1 = initialVelocity1;
      Double finalVelocity2 = initialVelocity2;
      
      int i = 0;
      while (i < NUMBER_OF_TRIES && initialVelocity1 == finalVelocity1)
      {
         simulationSession.runTick();
         
         finalVelocity1 = floatingRootJoint1.getSuccessor().getBodyFixedFrame().getTwistOfFrame().getLinearPart().getY();
         finalVelocity2 = floatingRootJoint2.getSuccessor().getBodyFixedFrame().getTwistOfFrame().getLinearPart().getY();
         
         i++;
      }
      
      //Collision happened
      if (initialVelocity1 != finalVelocity1)
      {
         System.out.println(i + " position 1: " + floatingRootJoint1.getJointPose().getPosition().getY() + " " + floatingRootJoint1.getSuccessor().getBodyFixedFrame().getTwistOfFrame().getLinearPartY());
         System.out.println(i + " position 2: " + floatingRootJoint2.getJointPose().getPosition().getY() + " " + floatingRootJoint2.getSuccessor().getBodyFixedFrame().getTwistOfFrame().getLinearPartY());
         System.out.println(mass1 * initialVelocity1 + mass2 * initialVelocity2);
         System.out.println(mass1 * finalVelocity1 + mass2 * finalVelocity2);
         System.out.println("v1: " + (mass1 * initialVelocity1 + mass2 * initialVelocity2 - mass2 * finalVelocity2) / mass1);
         double cR = Math.abs(finalVelocity2 - finalVelocity1) / Math.abs(initialVelocity1 - initialVelocity2);
         System.out.println("Cr: " + cR);
         System.out.println("v2: " + (cR * (initialVelocity1 - initialVelocity2) + finalVelocity1));
      
         System.out.println("v1: " + (mass1 * initialVelocity1 + mass2 * initialVelocity2 + mass2 * cR * (initialVelocity2 - initialVelocity1)) / (mass1 + mass2));
         
         assertEquals(cR, coefficientOfRestitution, EPSILON);
      }
      else
      {
         System.out.println("No Collision");
      }
   }

   private RobotDefinition createSphereRobot(double radius,
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
      
      SixDoFJointState sphere2InitialState = new SixDoFJointState();
      sphere2InitialState.setConfiguration(null, initialPosition);
      sphere2InitialState.setVelocity(null, initialVelocity);
      sphereRobot.getRootJointDefinitions().get(0).setInitialJointState(sphere2InitialState);
      
      return sphereRobot;
   }
}

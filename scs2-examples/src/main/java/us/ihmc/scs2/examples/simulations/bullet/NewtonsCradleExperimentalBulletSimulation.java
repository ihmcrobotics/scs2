package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletContactSolverInfoParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;

public class NewtonsCradleExperimentalBulletSimulation
{
   private static final String NEWTONS_CRADLE = "NewtonsCradle";
   private static final boolean VISUALIZE_WITH_DEBUG_DRAWING = true;
   private static final int numberOfBalls = 6;
   private static final double ballRadius = 0.05;

   private static final double stringLength = 0.6;
   private static final double stringRadius = 0.002;
   private static final double ballMass = 0.2;

   public NewtonsCradleExperimentalBulletSimulation()
   {
      SimulationSession simulationSession = createSession();
      if (VISUALIZE_WITH_DEBUG_DRAWING)
      {
         SessionVisualizer sessionVisualizer = BulletExampleSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);
         sessionVisualizer.getToolkit().getSession().runTick();
      }
      else
      {
         SessionVisualizer.startSessionVisualizer(simulationSession);
      }
   }

   public static SimulationSession createSession()
   {
      RobotDefinition robotDefinition = new RobotDefinition(NEWTONS_CRADLE);
      double ballRadiusOfGyration = ballRadius * 0.6;
      double pinJointHeight = 1.1 * stringLength;
      double pinJointSeparation = 2.0001 * ballRadius;

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      robotDefinition.setRootBodyDefinition(rootBody);

      MaterialDefinition ballMaterial = new MaterialDefinition(ColorDefinitions.LightGrey(),
                                                               ColorDefinitions.Silver(),
                                                               ColorDefinitions.LightBlue(),
                                                               ColorDefinitions.AntiqueWhite(),
                                                               30);

      for (int i = 0; i < numberOfBalls; i++)
      {
         Vector3D offset = new Vector3D(i * pinJointSeparation, 1.0, pinJointHeight);
         RevoluteJointDefinition revoluteJoint = new RevoluteJointDefinition("pin" + i);
         revoluteJoint.setAxis(Axis3D.Y);
         revoluteJoint.setTransformToParent(new RigidBodyTransform(new Quaternion(), offset));

         RigidBodyDefinition rigidBody = new RigidBodyDefinition(getBallBodyName(i));
         rigidBody.setMass(ballMass);
         rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(ballMass,
                                                                                        ballRadiusOfGyration,
                                                                                        ballRadiusOfGyration,
                                                                                        ballRadiusOfGyration));
         rigidBody.setCenterOfMassOffset(0.0, 0.0, -stringLength);

         VisualDefinitionFactory factory = new VisualDefinitionFactory();
         factory.appendTranslation(0.0, 0.0, -0.5 * stringLength);
         factory.addCylinder(stringLength, stringRadius, ColorDefinitions.DarkSlateGray());
         factory.appendTranslation(0, 0, -0.5 * stringLength);
         factory.addSphere(ballRadius, ballMaterial);
         rigidBody.addVisualDefinitions(factory.getVisualDefinitions());

         rigidBody.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, -stringLength)),
                                                                            new Sphere3DDefinition(ballRadius)));
         rigidBody.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(new Quaternion(),
                                                                                                   new Vector3D(0.0, 0.0, -0.5 * stringLength)),
                                                                            new Cylinder3DDefinition(stringLength, stringRadius)));

         if (i == 0 || i == 1)
            revoluteJoint.setInitialJointState(new OneDoFJointState(0.3));

         revoluteJoint.setSuccessor(rigidBody);
         rootBody.addChildJoint(revoluteJoint);
      }

      BulletMultiBodyParameters parameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      BulletMultiBodyJointParameters jointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
      jointParameters.setJointRestitution(1.0);
      BulletContactSolverInfoParameters contactSolverInfoParameters = BulletContactSolverInfoParameters.defaultBulletContactSolverInfoParameters();
      contactSolverInfoParameters.setErrorReductionForContactConstraints(0.035);

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(parameters,
                                                                                                                           jointParameters,
                                                                                                                           contactSolverInfoParameters));
      simulationSession.addRobot(robotDefinition);
      return simulationSession;
   }

   private static String getBallBodyName(int i)
   {
      return "ball" + i;
   }

   public static void main(String[] args)
   {
      new NewtonsCradleExperimentalBulletSimulation();
   }
}

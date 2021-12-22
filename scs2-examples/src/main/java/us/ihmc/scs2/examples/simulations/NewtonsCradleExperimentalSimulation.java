package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

public class NewtonsCradleExperimentalSimulation
{
   private static final String NEWTONS_CRADLE = "NewtonsCradle";
   private final int numberOfBalls = 6;
   private final double ballRadius = 0.05;

   private final double stringLength = 0.6;
   private final double stringRadius = 0.002;
   private final double ballMass = 0.2;

   public NewtonsCradleExperimentalSimulation()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfRestitution(1.0);
      contactParameters.setRestitutionThreshold(0.0);

      RobotDefinition robotDefinition = new RobotDefinition(NEWTONS_CRADLE);
      double ballRadiusOfGyration = ballRadius * 0.6;
      double pinJointHeight = 1.1 * stringLength;
      double pinJointSeparation = 2.0001 * ballRadius; // FIXME Note the 1.0e-4 epsilon here, this is to prevent things from blowing up. Need to figure out how to fix this.

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      robotDefinition.setRootBodyDefinition(rootBody);

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
         factory.addCylinder(stringLength, stringRadius, new MaterialDefinition(ColorDefinitions.Yellow()));
         ColorDefinition aliceBlue = ColorDefinitions.Red();
         aliceBlue.setAlpha(0.4);
         factory.appendTranslation(0, 0, -0.5 * stringLength);
         factory.addSphere(ballRadius, new MaterialDefinition(aliceBlue));
         rigidBody.addVisualDefinitions(factory.getVisualDefinitions());

         rigidBody.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, -stringLength)),
                                                                            new Sphere3DDefinition(ballRadius)));

         if (i == 0 || i == 1)
            revoluteJoint.setInitialJointState(new OneDoFJointState(0.3));

         revoluteJoint.setSuccessor(rigidBody);
         rootBody.addChildJoint(revoluteJoint);
      }

      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters));
      simulationSession.addRobot(robotDefinition);
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   private String getBallBodyName(int i)
   {
      return "ball" + i;
   }

   public static void main(String[] args)
   {
      new NewtonsCradleExperimentalSimulation();
   }
}

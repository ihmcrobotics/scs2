package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PrismaticSpringBulletSimulation
{
   private static final String SPRING_PENDULUM = "SpringPendulum";
   private static final boolean VISUALIZE_WITH_DEBUG_DRAWING = false;

   private final double ballRadius = 0.04;
   private final double stringLength = 0.4;
   private final double stringRadius = 0.005;
   private final double ballMass = 0.3;

   public PrismaticSpringBulletSimulation()
   {
      RobotDefinition robotDefinition = new RobotDefinition(SPRING_PENDULUM);
      double ballRadiusOfGyration = ballRadius * 0.6;
      double pinJointHeight = 1.1 * stringLength;

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      robotDefinition.setRootBodyDefinition(rootBody);

      Vector3D offset = new Vector3D(0.2, 0.2, pinJointHeight);
      PrismaticJointDefinition prismaticJoint = new PrismaticJointDefinition("spring");
      prismaticJoint.setAxis(Axis3D.Z);
      prismaticJoint.setTransformToParent(new RigidBodyTransform(new Quaternion(), offset));

      RigidBodyDefinition rigidBody = new RigidBodyDefinition("ball");
      rigidBody.setMass(ballMass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(ballMass,
                                                                                     ballRadiusOfGyration,
                                                                                     ballRadiusOfGyration,
                                                                                     ballRadiusOfGyration));
      rigidBody.setCenterOfMassOffset(0.0, 0.0, -stringLength);

      MaterialDefinition ballMaterial = new MaterialDefinition(ColorDefinitions.Silver(),
                                                               ColorDefinitions.White(),
                                                               ColorDefinitions.LightBlue(),
                                                               ColorDefinitions.AntiqueWhite(),
                                                               20);

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.appendTranslation(0.0, 0.0, -0.5 * stringLength);
      factory.addCylinder(stringLength, stringRadius, ColorDefinitions.DarkSlateGray());
      factory.appendTranslation(0, 0, -0.5 * stringLength);
      factory.addSphere(ballRadius, ballMaterial);
      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());

      rigidBody.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, -stringLength)),
                                                                         new Sphere3DDefinition(ballRadius)));
      rigidBody.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, -0.5 * stringLength)),
                                                                         new Cylinder3DDefinition(stringLength, stringRadius)));

      prismaticJoint.setInitialJointState(new OneDoFJointState(0.3));
      prismaticJoint.setPositionLimits(-0.5, 0.5);

      prismaticJoint.setSuccessor(rigidBody);
      rootBody.addChildJoint(prismaticJoint);

      robotDefinition.addControllerDefinition((controllerInput, controllerOutput) -> new Controller()
      {
         YoRegistry registry = new YoRegistry("SpringController");
         YoDouble stiffness = new YoDouble("springStiffness", registry);
         YoDouble desiredLength = new YoDouble("springDesiredLength", registry);
         OneDoFJointReadOnly sliderJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint("spring");
         OneDoFJointStateBasics sliderOutput = controllerOutput.getOneDoFJointOutput("spring");

         @Override
         public void initialize()
         {
            stiffness.set(1000.0);
            desiredLength.set(0);
         }

         @Override
         public void doControl()
         {
            double currentPosition = sliderJoint.getQ();
            double desiredForce = stiffness.getValue() * (desiredLength.getValue() - currentPosition);
            sliderOutput.setEffort(desiredForce);
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return registry;
         }
      });

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory());
      simulationSession.addRobot(robotDefinition);

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

   public static void main(String[] args)
   {
      new PrismaticSpringBulletSimulation();
   }
}

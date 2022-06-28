package us.ihmc.scs2.simulation.bullet.physicsEngine;

import org.bytedeco.bullet.BulletDynamics.btMultiBody;
import org.bytedeco.bullet.LinearMath.btVector3;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyParameters;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FallingSphereExperimentalBulletSimulation
{
   public FallingSphereExperimentalBulletSimulation()
   {
      double radius1 = 0.3;
      double mass1 = 2.0;
      double radiusOfGyrationPercent1 = 1.0;
      ColorDefinition appearance1 = ColorDefinitions.DarkGreen();
      ColorDefinition stripesAppearance1 = ColorDefinitions.LightGreen();
      RobotDefinition sphereRobot1 = newSphereRobot("sphere1", radius1, mass1, radiusOfGyrationPercent1, appearance1, true, stripesAppearance1);

      sphereRobot1.getRigidBodyDefinition("sphere1RigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(radius1)));

      SixDoFJointState sphere1InitialState = new SixDoFJointState();
      sphere1InitialState.setConfiguration(null, new Point3D(0, 0, 2.0));
      sphere1InitialState.setVelocity(null, new Vector3D(0.0, 0.0, 0));
      sphereRobot1.getRootJointDefinitions().get(0).setInitialJointState(sphere1InitialState);
      
      Robot robot = new Robot(sphereRobot1, ReferenceFrameTools.constructARootFrame("worldFrame"));
      
      BulletMultiBodyDynamicsWorld bulletMultiBodyDynamicsWorld = new BulletMultiBodyDynamicsWorld();
      YoRegistry physicsEngineRegistry = null;
      YoBulletMultiBodyParameters globalMultiBodyParameters = new YoBulletMultiBodyParameters("globalMultiBody", physicsEngineRegistry);
      YoBulletMultiBodyJointParameters globalMultiBodyJointParameters = new YoBulletMultiBodyJointParameters("globalMultiBodyJoint", physicsEngineRegistry);
      
      BulletMultiBodyParameters bulletMultiBodyParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      bulletMultiBodyParameters.setLinearDamping(0);
      BulletMultiBodyJointParameters bulletMultiBodyJointParameter = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
      bulletMultiBodyJointParameter.setJointRestitution(1.0);

      globalMultiBodyParameters.set(bulletMultiBodyParameters);
      globalMultiBodyJointParameters.set(bulletMultiBodyJointParameter);

      BulletMultiBodyRobot bulletMultiBodyRobot = BulletMultiBodyRobotFactory.newInstance(robot, globalMultiBodyParameters, globalMultiBodyJointParameters);
      
      bulletMultiBodyDynamicsWorld.addBulletMultiBodyRobot(bulletMultiBodyRobot);
      bulletMultiBodyDynamicsWorld.setGravity(new Vector3D(0.0, 0.0, 9.81));
      
      btMultiBody btMultiBody = bulletMultiBodyRobot.getBtMultiBody();
      
      double dt = 0.1;

      for (int i = 1; i < 100; i++)
      {
         bulletMultiBodyDynamicsWorld.stepSimulation((float) dt, 1, (float) dt);
         
         btVector3 position = btMultiBody.getBasePos();
         System.out.println(i + " " + position.z());
      }

      bulletMultiBodyDynamicsWorld.dispose();

   }
   
   public static RobotDefinition newSphereRobot(String name,
                                                double radius,
                                                double mass,
                                                double radiusOfGyrationPercent,
                                                ColorDefinition color,
                                                boolean addStripes,
                                                ColorDefinition stripesColor)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      rootJoint.setSuccessor(newSphereRigidBody(name + "RigidBody", radius, mass, radiusOfGyrationPercent, color, addStripes, stripesColor));
      robotDefinition.setRootBodyDefinition(rootBody);
      return robotDefinition;
   }
   
   public static RigidBodyDefinition newSphereRigidBody(String name,
                                                        double radius,
                                                        double mass,
                                                        double radiusOfGyrationPercent,
                                                        ColorDefinition color,
                                                        boolean addStripes,
                                                        ColorDefinition stripesColor)
   {
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name);
      double radiusOfGyration = radiusOfGyrationPercent * radius;
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass, radiusOfGyration, radiusOfGyration, radiusOfGyration));

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addSphere(radius, color);

      if (addStripes)
      {
         double stripePercent = 0.05;
         factory.addArcTorus(0.0, 2.0 * Math.PI, (1.01 - stripePercent) * radius, radius * stripePercent, stripesColor);
         factory.appendRotation(Math.PI / 2.0, Axis3D.X);
         factory.addArcTorus(0.0, 2.0 * Math.PI, (1.01 - stripePercent) * radius, radius * stripePercent, stripesColor);
      }

      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      return rigidBody;
   }

   public static void main(String[] args)
   {
      new FallingSphereExperimentalBulletSimulation();
   }
}

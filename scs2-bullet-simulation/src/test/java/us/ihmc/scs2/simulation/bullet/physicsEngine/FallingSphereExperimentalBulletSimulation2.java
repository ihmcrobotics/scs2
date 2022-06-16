package us.ihmc.scs2.simulation.bullet.physicsEngine;

import org.bytedeco.bullet.BulletDynamics.*;
import org.bytedeco.bullet.LinearMath.*;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FallingSphereExperimentalBulletSimulation2
{
   public FallingSphereExperimentalBulletSimulation2()
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
      
      BulletMultiBodyParameters bulletMultiBodyParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      bulletMultiBodyParameters.setLinearDamping(0);
      BulletMultiBodyJointParameters bulletMultiBodyJointParameter = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
      bulletMultiBodyJointParameter.setJointRestitution(1.0);
      
      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(bulletMultiBodyParameters, bulletMultiBodyJointParameter));
      simulationSession.addRobot(sphereRobot1);
      
      GeometryDefinition terrainGeometry = new Box3DDefinition(6, 6, 0.01);
      RigidBodyTransform terrainPose = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, 0.0));
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.LightSlateGray())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      simulationSession.addTerrainObject(terrain);

      simulationSession.submitBufferSizeRequest(245760);
      simulationSession.setBufferRecordTickPeriod(8);
      simulationSession.setSessionDTSeconds(0.1);
      
      BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine)simulationSession.getPhysicsEngine();
      BulletMultiBodyDynamicsWorld bulletMultiBodyDynamicsWorld = bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld();
      btMultiBody btMultiBody = bulletMultiBodyDynamicsWorld.getBtMultiBodyDynamicsWorld().getMultiBody(0);
      
      double dt = 0.1;
      Vector3D gravity = new Vector3D(0.0, .0, -9.81);

      for (int i = 1; i <= 10; i++)
      {
         //bulletMultiBodyDynamicsWorld.stepSimulation((float) dt, 1, (float) dt);   //Works
         //bulletPhysicsEngine.simulate(dt, dt, gravity);                            //Works
         
         simulationSession.runTick();                                                //EXCEPTION_ACCESS_VIOLATION
         
         btVector3 position = btMultiBody.getBasePos();
         System.out.println(i + " " + position.z());
      }
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
      new FallingSphereExperimentalBulletSimulation2();
   }
}

package us.ihmc.scs2.simulation.bullet.simulations;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.CollisionJNI;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletBasedPhysicsEngine;

public class BoxExperimentalSimulation
{
   
   public BoxExperimentalSimulation()
   {
      //ContactParameters contactParameters = new ContactParameters();
      //contactParameters.setMinimumPenetration(5.0e-5);
      //contactParameters.setCoefficientOfFriction(0.7);
      //contactParameters.setErrorReductionParameter(0.001);

      double boxXLength = 0.2;
      double boxYWidth = 0.12;
      double boxZHeight = 0.4;
      double boxMass = 1.0;
      double boxRadiusOfGyrationPercent = 0.8;

      double initialBoxRoll = -Math.PI / 64.0;
      double initialVelocity = 0.0;

      double groundWidth = 1.0;
      double groundLength = 1.0;

      RobotDefinition boxRobot = new RobotDefinition("box");

      RigidBodyDefinition rootBody = new RigidBodyDefinition("box" + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition("box");
      rootBody.addChildJoint(rootJoint);
      
      RigidBodyDefinition rigidBody = new RigidBodyDefinition("box");
      rigidBody.setMass(boxMass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(boxMass,
                                                                                     boxRadiusOfGyrationPercent * boxXLength,
                                                                                     boxRadiusOfGyrationPercent * boxYWidth,
                                                                                     boxRadiusOfGyrationPercent * boxZHeight));
      
      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addBox(boxXLength, boxYWidth, boxZHeight, new MaterialDefinition(ColorDefinitions.DarkCyan()));
      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      
      rootJoint.setSuccessor(rigidBody);
      boxRobot.setRootBodyDefinition(rootBody);
      
      //boxRobot.getRigidBodyDefinition("boxRigidBody")
      //        .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxXLength, boxYWidth, boxZHeight)));

      SixDoFJointState initialJointState = new SixDoFJointState(new YawPitchRoll(0, 0, initialBoxRoll),
                                                                new Point3D(0.0,
                                                                            groundWidth / 2.0 - 0.002,
                                                                            boxZHeight / 2.0 * 1.05 + boxYWidth / 2.0 * Math.sin(Math.abs(initialBoxRoll))));
      initialJointState.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);
      
      btBoxShape boxCollisionShape = new btBoxShape(new Vector3((float)boxXLength, (float)boxYWidth, (float)boxZHeight));

      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, -0.05));
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      SimulationSession simulationSession = new SimulationSession((frame, rootRegistry) -> new BulletBasedPhysicsEngine(frame, rootRegistry));
      simulationSession.addRobot(boxRobot);
      simulationSession.addTerrainObject(terrain);
      BulletBasedPhysicsEngine bulletPhysicEngine = (BulletBasedPhysicsEngine)simulationSession.getPhysicsEngine();
      btMotionState motionState = new btMotionState();
      bulletPhysicEngine.addRigidBodyRobot(boxRobot, boxCollisionShape, 0, motionState, true);
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new BoxExperimentalSimulation();
   }
}
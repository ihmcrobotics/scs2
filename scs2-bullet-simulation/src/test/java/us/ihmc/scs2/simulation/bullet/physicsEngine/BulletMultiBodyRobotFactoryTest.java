package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.BroadphaseNativeTypes;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCapsuleShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.collision.btConeShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btCylinderShape;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.dynamics.btMultibodyLink;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;
import com.badlogic.gdx.physics.bullet.linearmath.btQuaternion;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletMultiBodyRobotFactoryTest
{
   static
   {
      Bullet.init();
      LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());
   }

   private final YawPitchRollTransformDefinition inertiaPose = new YawPitchRollTransformDefinition();
   private final YawPitchRollTransformDefinition collisionShapePose = new YawPitchRollTransformDefinition();
   private CollisionShapeDefinition shapeDefinition = new CollisionShapeDefinition();
   private final static RigidBodyTransform collisionShapeRigidBodyTransform = new RigidBodyTransform();
   private static Matrix4 compoundShapeChildTransform = new Matrix4();
   private static final Vector3 boxVertex = new Vector3();
   private static final YawPitchRollTransformDefinition TransformToParent = new YawPitchRollTransformDefinition();
   private static final RigidBodyTransform parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid = new RigidBodyTransform();
   private static final RigidBodyTransform parentJointAfterFrameToLinkCenterOfMassTransformEuclid = new RigidBodyTransform();

   private final static double EPSILON = 1e-5;
   private static final int ITERATIONS = 1;

   @Test
   public void testNewInstance()
   {
      Random random = new Random(42187);
      String name = "TestRobot";

      YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());
      YoBulletMultiBodyParameters globalMultiBodyParameters = new YoBulletMultiBodyParameters("globalMultiBody", physicsEngineRegistry);
      globalMultiBodyParameters.set(BulletMultiBodyParameters.defaultBulletMultiBodyParameters());
      YoBulletMultiBodyJointParameters globalMultiBodyJointParameters = new YoBulletMultiBodyJointParameters("globalMultiJointBody", physicsEngineRegistry);
      globalMultiBodyJointParameters.set(BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters());

      //Verify a robot without a joint produces an exception error.
      Assertions.assertThrows(UnsupportedOperationException.class, () ->
      {
         RobotDefinition robotDefinition = new RobotDefinition(name + "RootBody");
         RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RigidBody");
         robotDefinition.setRootBodyDefinition(rootBody);
         Robot robotNoJoints = new Robot(robotDefinition, ReferenceFrameTools.constructARootFrame("worldFrame"));
         BulletMultiBodyRobotFactory.newInstance(robotNoJoints, globalMultiBodyParameters, globalMultiBodyJointParameters);
      });

      //Test creating BulletMultiBodyRobots from robots without a BaseCollider
      for (int i = 0; i < ITERATIONS; i++)
      {
         Robot robot = createTestRobotWithoutBaseCollider(random, name + i);

         BulletMultiBodyRobot bulletMultiBodyRobot = BulletMultiBodyRobotFactory.newInstance(robot, globalMultiBodyParameters, globalMultiBodyJointParameters);

         assertTrue(bulletMultiBodyRobot.getBtMultiBody().getBaseCollider() == null, "Assert btMultiBody does not have a Base Collider");

         assertBulletMultiBodyRobotCreatedCorrectly(robot, bulletMultiBodyRobot, globalMultiBodyParameters, globalMultiBodyJointParameters);

         globalMultiBodyParameters.set(nextRandomMultiBodyParameters(random));
         globalMultiBodyJointParameters.set(nextRandomMultiBodyJointParameters(random));
      }

      //Test creating BulletMultiBodyRobots from robots with a BaseCollider
      for (int i = 0; i < ITERATIONS; i++)
      {
         Robot robot = createTestRobotWithBaseCollider(random, name + i);

         BulletMultiBodyRobot bulletMultiBodyRobot = BulletMultiBodyRobotFactory.newInstance(robot, globalMultiBodyParameters, globalMultiBodyJointParameters);

         assertTrue(bulletMultiBodyRobot.getBtMultiBody().getBaseCollider() != null, "Assert btMultiBody does not have a Base Collider");

         assertBulletMultiBodyRobotCreatedCorrectly(robot, bulletMultiBodyRobot, globalMultiBodyParameters, globalMultiBodyJointParameters);
         
         globalMultiBodyParameters.set(nextRandomMultiBodyParameters(random));
         globalMultiBodyJointParameters.set(nextRandomMultiBodyJointParameters(random));
      }
   }
   
   @Test
   public void testNewInstanceRegressionTest()
   {
      YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());
      YoBulletMultiBodyParameters globalMultiBodyParameters = new YoBulletMultiBodyParameters("globalMultiBody", physicsEngineRegistry);
      globalMultiBodyParameters.set(BulletMultiBodyParameters.defaultBulletMultiBodyParameters());
      YoBulletMultiBodyJointParameters globalMultiBodyJointParameters = new YoBulletMultiBodyJointParameters("globalMultiJointBody", physicsEngineRegistry);
      globalMultiBodyJointParameters.set(BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters());
      
      Robot robot = createTestRobotWithKnownValues();
    
      BulletMultiBodyRobot bulletMultiBodyRobot = BulletMultiBodyRobotFactory.newInstance(robot, globalMultiBodyParameters, globalMultiBodyJointParameters);
      
      btMultiBody btMultibody = bulletMultiBodyRobot.getBtMultiBody();
      assertEquals(btMultibody.getNumLinks(), 1);
      assertEquals(bulletMultiBodyRobot.getBulletMultiBodyLinkCollider(0).getCollisionGroup(), 64);
      assertEquals(bulletMultiBodyRobot.getBulletMultiBodyLinkCollider(0).getCollisionGroupMask(), 899);
      assertEquals(bulletMultiBodyRobot.getBulletMultiBodyLinkCollider(1).getCollisionGroup(), 1);
      assertEquals(bulletMultiBodyRobot.getBulletMultiBodyLinkCollider(1).getCollisionGroupMask(), 3);
      
      Vector3 linkAxis = bulletMultiBodyRobot.getBtMultiBody().getLink(0).getAxisTop(0);
      assertEquals(linkAxis.x,0);
      assertEquals(linkAxis.y,0);
      assertEquals(linkAxis.z,1);

      btMultiBodyLinkCollider baseCollider = bulletMultiBodyRobot.getBulletMultiBodyLinkCollider(0).getBtMultiBodyLinkCollider();
      btMultiBodyLinkCollider linkCollider = bulletMultiBodyRobot.getBulletMultiBodyLinkCollider(1).getBtMultiBodyLinkCollider();
      
      btCompoundShape baseColliderCompoundShape = (btCompoundShape)baseCollider.getCollisionShape();
      assertEquals(baseColliderCompoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE);
      btCylinderShape cylinderShape = (btCylinderShape)baseColliderCompoundShape.getChildShape(0);
      assertEquals(cylinderShape.getRadius(), 0.11f);
      assertEquals(cylinderShape.getHalfExtentsWithMargin().z, 0.06f / 2.0f);
      assertEquals(baseColliderCompoundShape.getChildShape(1).getShapeType(), BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE);
      cylinderShape = (btCylinderShape)baseColliderCompoundShape.getChildShape(1);
      assertEquals(cylinderShape.getRadius(), 0.12f);
      assertEquals(cylinderShape.getHalfExtentsWithMargin().z, 0.04f / 2.0f);
      assertEquals(baseColliderCompoundShape.getChildShape(2).getShapeType(), BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE);
      cylinderShape = (btCylinderShape)baseColliderCompoundShape.getChildShape(2);
      assertEquals(cylinderShape.getRadius(), 0.16f);
      assertEquals(cylinderShape.getHalfExtentsWithMargin().z, 0.05f / 2.0f);
      btCompoundShape linkColliderCompoundShape = (btCompoundShape)linkCollider.getCollisionShape();
      assertEquals(linkColliderCompoundShape.getChildShape(0).getShapeType(), BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE);
      btBoxShape boxShape = (btBoxShape)linkColliderCompoundShape.getChildShape(0);
      boxShape.getVertex(0, boxVertex);
      assertEquals(Math.abs(boxVertex.x), (float) 0.03f / 2.0f, EPSILON);
      assertEquals(Math.abs(boxVertex.y), (float) 0.04f / 2.0f, EPSILON);
      assertEquals(Math.abs(boxVertex.z), (float) 0.02f / 2.0f, EPSILON);
      
      assertEquals(linkCollider.getFriction(), (float) globalMultiBodyJointParameters.getJointFriction());
      assertEquals(linkCollider.getRestitution(), (float) globalMultiBodyJointParameters.getJointRestitution());
      assertEquals(linkCollider.getHitFraction(), (float) globalMultiBodyJointParameters.getJointHitFraction());
      assertEquals(linkCollider.getRollingFriction(), (float) globalMultiBodyJointParameters.getJointRollingFriction());
      assertEquals(linkCollider.getSpinningFriction(), (float) globalMultiBodyJointParameters.getJointSpinningFriction());
      assertEquals(linkCollider.getContactProcessingThreshold(), (float) globalMultiBodyJointParameters.getJointContactProcessingThreshold());
      
      assertEquals(btMultibody.getBaseMass(), 9.609f);
      assertEquals(btMultibody.getLinkMass(0), 2.27f);
      
      
//      assertBulletMultiBodyRobotCreatedCorrectly(robot, bulletMultiBodyRobot, globalMultiBodyParameters, globalMultiBodyJointParameters);
   }

   @Test
   public void testBulletCollisionShapeLocalTransform()
   {
      inertiaPose.setTranslation(0.05, 0.08, 0.09);
      inertiaPose.setOrientation(0.01, 0.01, 0.01);
      collisionShapePose.setTranslation(inertiaPose.getX() - 0.01, inertiaPose.getY() - 0.01, inertiaPose.getZ() - 0.01);
      collisionShapePose.setOrientation(inertiaPose.getYaw() - 0.004, inertiaPose.getPitch() - 0.004, inertiaPose.getRoll() - 0.004);

      String robotName = "Robot1";
      Robot robot1 = createRobotToTestLocalTransform(robotName, inertiaPose, collisionShapePose);

      shapeDefinition = robot1.getRobotDefinition().getRigidBodyDefinition(robotName + "RigidBody").getCollisionShapeDefinitions().get(0);
      ReferenceFrame linkCenterOfMassFrame = robot1.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      Matrix4 bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition, linkCenterOfMassFrame);

      RigidBodyTransform rigidBodyTransformExpectedResults1 = new RigidBodyTransform(0.999984,
                                                                                     -0.003959719,
                                                                                     0.004039708,
                                                                                     0.009999639,
                                                                                     0.0039757174,
                                                                                     0.99998426,
                                                                                     -0.0039599594,
                                                                                     0.010000003,
                                                                                     -0.004023964,
                                                                                     0.0039759567,
                                                                                     0.999984,
                                                                                     0.010000359);

      assertMatrix4EqualsRigidBodyTransform(robotName, bulletCollisionShapeLocalTransform, rigidBodyTransformExpectedResults1);

      inertiaPose.setTranslation(0.07, 0.02, 0.01);
      inertiaPose.setOrientation(0.03, 0.02, 0.04);
      collisionShapePose.setTranslation(0.02, 0.06, 0.03);
      collisionShapePose.setOrientation(0.04, 0.02, 0.01);

      robotName = "Robot2";
      Robot robot2 = createRobotToTestLocalTransform(robotName, inertiaPose, collisionShapePose);

      shapeDefinition = robot2.getRobotDefinition().getRigidBodyDefinition(robotName + "RigidBody").getCollisionShapeDefinitions().get(0);
      linkCenterOfMassFrame = robot2.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition, linkCenterOfMassFrame);

      RigidBodyTransform rigidBodyTransformExpectedResults2 = new RigidBodyTransform(0.99995,
                                                                                     0.009989796,
                                                                                     -4.0080564E-4,
                                                                                     0.048750732,
                                                                                     -0.009997344,
                                                                                     0.9994941,
                                                                                     -0.030193394,
                                                                                     -0.042155657,
                                                                                     9.897699E-5,
                                                                                     0.030195892,
                                                                                     0.999544,
                                                                                     -0.018608237);

      assertMatrix4EqualsRigidBodyTransform(robotName, bulletCollisionShapeLocalTransform, rigidBodyTransformExpectedResults2);

      inertiaPose.setTranslation(0.0, 0.0, 0.0);
      inertiaPose.setOrientation(0.0, 0.0, 0.0);
      collisionShapePose.setTranslation(0.0, 0.0, 0.0);
      collisionShapePose.setOrientation(0.0, 0.0, 0.0);

      robotName = "Robot3";
      Robot robot3 = createRobotToTestLocalTransform(robotName, inertiaPose, collisionShapePose);

      shapeDefinition = robot3.getRobotDefinition().getRigidBodyDefinition(robotName + "RigidBody").getCollisionShapeDefinitions().get(0);
      linkCenterOfMassFrame = robot3.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition, linkCenterOfMassFrame);

      RigidBodyTransform rigidBodyTransformExpectedResults3 = new RigidBodyTransform(1.0, 0.0, 0.0, -0.0, 0.0, 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, -0.0);

      assertMatrix4EqualsRigidBodyTransform(robotName, bulletCollisionShapeLocalTransform, rigidBodyTransformExpectedResults3);
   }

   private static BulletMultiBodyParameters nextRandomMultiBodyParameters(Random random)
   {
      BulletMultiBodyParameters parameters = new BulletMultiBodyParameters(random.nextBoolean(),
                                                                           random.nextBoolean(),
                                                                           random.nextBoolean(),
                                                                           random.nextBoolean(),
                                                                           random.nextBoolean(),
                                                                           random.nextDouble(),
                                                                           random.nextDouble(),
                                                                           random.nextDouble(),
                                                                           random.nextDouble());
      return parameters;
   }
   
   private static BulletMultiBodyJointParameters nextRandomMultiBodyJointParameters(Random random)
   {
      BulletMultiBodyJointParameters jointParameters = new BulletMultiBodyJointParameters(random.nextBoolean(),
                                                                                          random.nextDouble(),
                                                                                          random.nextDouble(),
                                                                                          random.nextDouble(),
                                                                                          random.nextDouble(),
                                                                                          random.nextDouble(),
                                                                                          random.nextDouble());
      return jointParameters;
   }
   
   private void assertBulletMultiBodyRobotCreatedCorrectly(Robot robot,
                                                           BulletMultiBodyRobot bulletMultiBodyRobot,
                                                           YoBulletMultiBodyParameters globalMultiBodyParameters,
                                                           YoBulletMultiBodyJointParameters globalMultiBodyJointParameters)
   {
      btMultiBody btMultiBody = bulletMultiBodyRobot.getBtMultiBody();

      //TODO: need to add a test for MomentOfInertia after it is corrected

      JointBasics rootJoint = robot.getRootBody().getChildrenJoints().get(0);
      boolean hasBaseCollider = rootJoint instanceof SimFloatingRootJoint;

      RigidBodyDefinition rigidBodyDefinition = robot.getRobotDefinition().getRootBodyDefinition().getChildrenJoints().get(0).getSuccessor();
      int numberOfLinks = 0;
      for (JointBasics joint : robot.getRootBody().getChildrenJoints())
      {
         numberOfLinks += countJointsAndCreateIndexMap(joint);
      }

      assertEquals((float) rigidBodyDefinition.getMass(), btMultiBody.getBaseMass());
      assertEquals((hasBaseCollider ? numberOfLinks - 1 : numberOfLinks), btMultiBody.getNumDofs());
      assertEquals((hasBaseCollider ? numberOfLinks - 1 : numberOfLinks), btMultiBody.getNumLinks());
      assertEquals(numberOfLinks, bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().size());
      assertEquals(numberOfLinks, bulletMultiBodyRobot.getBulletMultiBodyLinkColliderArray().size());
      assertEquals(btMultiBody.getCanSleep(), globalMultiBodyParameters.getCanSleep());
      assertEquals(btMultiBody.hasSelfCollision(), globalMultiBodyParameters.getHasSelfCollision());
      assertEquals(btMultiBody.getUseGyroTerm(), globalMultiBodyParameters.getUseGyroTerm());
      assertEquals(btMultiBody.isUsingRK4Integration(), globalMultiBodyParameters.getUseRK4Integration());
      assertEquals(btMultiBody.getLinearDamping(), (float) globalMultiBodyParameters.getLinearDamping(), EPSILON);
      assertEquals(btMultiBody.getAngularDamping(), (float) globalMultiBodyParameters.getAngularDamping(), EPSILON);
      assertEquals(btMultiBody.getMaxAppliedImpulse(), (float) globalMultiBodyParameters.getMaxAppliedImpulse(), EPSILON);
      assertEquals(btMultiBody.getMaxCoordinateVelocity(), (float) globalMultiBodyParameters.getMaxCoordinateVelocity(), EPSILON);

      for (JointBasics joint : robot.getRootBody().getChildrenJoints())
      {
         if (!(joint instanceof SimFloatingRootJoint))
            assertJointAndLinkEqual(robot, bulletMultiBodyRobot, globalMultiBodyJointParameters, btMultiBody, joint, hasBaseCollider);
         
         testChildJoints(robot, bulletMultiBodyRobot, globalMultiBodyJointParameters, btMultiBody, joint, hasBaseCollider);
      }
   }

   private static void testChildJoints (Robot robot,
                                        BulletMultiBodyRobot bulletMultiBodyRobot,
                                        YoBulletMultiBodyJointParameters globalMultiBodyJointParameters,
                                        btMultiBody btMultiBody,
                                        JointBasics jointBasics,
                                        boolean hasBaseCollider)
   {
      for (JointBasics childJoint : jointBasics.getSuccessor().getChildrenJoints())
      {
         assertJointAndLinkEqual(robot, bulletMultiBodyRobot, globalMultiBodyJointParameters, btMultiBody, childJoint, hasBaseCollider);
         testChildJoints(robot, bulletMultiBodyRobot, globalMultiBodyJointParameters, btMultiBody, childJoint, hasBaseCollider);
       }
   }
   
   private static void assertJointAndLinkEqual(Robot robot,
                                               BulletMultiBodyRobot bulletMultiBodyRobot,
                                               YoBulletMultiBodyJointParameters globalMultiBodyJointParameters,
                                               btMultiBody btMultiBody,
                                               JointBasics jointBasics,
                                               boolean hasBaseCollider)
   {
      int index = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(jointBasics.getName());
      btMultiBodyLinkCollider linkCollider = btMultiBody.getLinkCollider(index);

      assertEquals(linkCollider.getFriction(), (float) globalMultiBodyJointParameters.getJointFriction());
      assertEquals(linkCollider.getRestitution(), (float) globalMultiBodyJointParameters.getJointRestitution());
      assertEquals(linkCollider.getHitFraction(), (float) globalMultiBodyJointParameters.getJointHitFraction());
      assertEquals(linkCollider.getRollingFriction(), (float) globalMultiBodyJointParameters.getJointRollingFriction());
      assertEquals(linkCollider.getSpinningFriction(), (float) globalMultiBodyJointParameters.getJointSpinningFriction());
      assertEquals(linkCollider.getContactProcessingThreshold(), (float) globalMultiBodyJointParameters.getJointContactProcessingThreshold());

      assertEquals(btMultiBody.getLink(index).getFlags(), (globalMultiBodyJointParameters.getJointDisableParentCollision() ? 1 : 0));
      JointDefinition jointDefinition = robot.getRobotDefinition().getJointDefinition(jointBasics.getName());
      RigidBodyDefinition jointRigidBodyDefinition = jointDefinition.getSuccessor();
      btMultiBodyLinkCollider btMultiBodyLinkCollider = bulletMultiBodyRobot.getBulletMultiBodyLinkCollider(index + (hasBaseCollider ? 1 : 0)).getBtMultiBodyLinkCollider();

      List<CollisionShapeDefinition> collisionShapes = jointRigidBodyDefinition.getCollisionShapeDefinitions();
      btCompoundShape compoundShape = (btCompoundShape) btMultiBodyLinkCollider.getCollisionShape();
      ReferenceFrame linkCenterOfMassFrame = jointBasics.getSuccessor().getBodyFixedFrame();
      assertCollisionShapesSame(collisionShapes, compoundShape, linkCenterOfMassFrame);

      //TODO: need to add a test for joint MomentOfInertia after it is corrected
      assertEquals((float) jointRigidBodyDefinition.getMass(), btMultiBody.getLinkMass(index));

      OneDoFJointDefinition oneDoFJointDefinition = (OneDoFJointDefinition) jointDefinition;
      Vector3D jointAxis = oneDoFJointDefinition.getAxis();
      btMultibodyLink link = btMultiBody.getLink(index);
      us.ihmc.euclid.tuple4D.Quaternion euclidRotationFromParent = new us.ihmc.euclid.tuple4D.Quaternion(jointDefinition.getTransformToParent()
                                                                                                                        .getRotation());
      euclidRotationFromParent.invert();
      assertQuaternionEqualsBtQuaternion(euclidRotationFromParent, link.getZeroRotParentToThis());

      jointBasics.getPredecessor().getBodyFixedFrame().getTransformToDesiredFrame(parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid,
                                                                                  jointBasics.getFrameBeforeJoint());
      parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid.invert();
      assertVector3DBasicsEqualsBtVector3(parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid.getTranslation(), link.getEVector());

      jointBasics.getFrameAfterJoint().getTransformToDesiredFrame(parentJointAfterFrameToLinkCenterOfMassTransformEuclid,
                                                                  jointBasics.getSuccessor().getBodyFixedFrame());
      parentJointAfterFrameToLinkCenterOfMassTransformEuclid.invert();
      assertVector3DBasicsEqualsBtVector3(parentJointAfterFrameToLinkCenterOfMassTransformEuclid.getTranslation(), link.getDVector());

      if (jointBasics instanceof SimRevoluteJoint)
      {
         Vector3 linkAxis = link.getAxisTop(0);
         assertVector3DEqualsVector3(jointAxis, linkAxis);
         assertEquals(link.getJointType(), btMultibodyLink.eFeatherstoneJointType.eRevolute);

      }
      else if (jointBasics instanceof SimPrismaticJoint)
      {
         Vector3 linkAxis = link.getAxisBottom(0);
         assertVector3DEqualsVector3(jointAxis, linkAxis);
      }
   }

   private static int countJointsAndCreateIndexMap(JointBasics joint)
   {
      int numberOfJoints = 1;
      for (JointBasics childrenJoint : joint.getSuccessor().getChildrenJoints())
      {
         numberOfJoints += countJointsAndCreateIndexMap(childrenJoint);
      }
      return numberOfJoints;
   }

   private static void assertMatrix4EqualsRigidBodyTransform(String name, Matrix4 bulletCollisionShapeLocalTransform, RigidBodyTransform rigidBodyTransform)
   {
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M00], rigidBodyTransform.getM00(), EPSILON, name + " - M00 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M01], rigidBodyTransform.getM01(), EPSILON, name + " - M01 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M02], rigidBodyTransform.getM02(), EPSILON, name + " - M02 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M03], rigidBodyTransform.getM03(), EPSILON, name + " - M03 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M10], rigidBodyTransform.getM10(), EPSILON, name + " - M10 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M11], rigidBodyTransform.getM11(), EPSILON, name + " - M11 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M12], rigidBodyTransform.getM12(), EPSILON, name + " - M12 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M13], rigidBodyTransform.getM13(), EPSILON, name + " - M13 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M20], rigidBodyTransform.getM20(), EPSILON, name + " - M20 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M21], rigidBodyTransform.getM21(), EPSILON, name + " - M21 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M22], rigidBodyTransform.getM22(), EPSILON, name + " - M22 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M23], rigidBodyTransform.getM23(), EPSILON, name + " - M23 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M30], rigidBodyTransform.getM30(), EPSILON, name + " - M30 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M31], rigidBodyTransform.getM31(), EPSILON, name + " - M31 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M32], rigidBodyTransform.getM32(), EPSILON, name + " - M32 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M33], rigidBodyTransform.getM33(), EPSILON, name + " - M33 is not as expected");
   }

   private static void assertQuaternionEqualsBtQuaternion(us.ihmc.euclid.tuple4D.Quaternion quaterion, btQuaternion btQuaternion)
   {
      assertEquals((float) quaterion.getX(), btQuaternion.getX(), EPSILON);
      assertEquals((float) quaterion.getY(), btQuaternion.getY(), EPSILON);
      assertEquals((float) quaterion.getZ(), btQuaternion.getZ(), EPSILON);
      assertEquals((float) quaterion.getS(), btQuaternion.getW(), EPSILON);
   }

   private static void assertVector3DEqualsVector3(Vector3D vector3D, Vector3 vector3)
   {
      assertEquals(vector3D.getX32(), vector3.x);
      assertEquals(vector3D.getY32(), vector3.y);
      assertEquals(vector3D.getZ32(), vector3.z);
   }

   private static void assertVector3DBasicsEqualsBtVector3(Vector3DBasics vector3DBasics, btVector3 btVector3)
   {
      assertEquals(vector3DBasics.getX32(), btVector3.getX());
      assertEquals(vector3DBasics.getY32(), btVector3.getY());
      assertEquals(vector3DBasics.getZ32(), btVector3.getZ());
   }

   private static void assertCollisionShapesSame(List<CollisionShapeDefinition> collisionShapes,
                                                 btCompoundShape compoundShape,
                                                 ReferenceFrame linkCenterOfMassFrame)
   {
      for (int j = 0; j < collisionShapes.size(); j++)
      {
         int shapeType;
         switch (collisionShapes.get(j).getGeometryDefinition().getName())
         {
            case "sphere":
               shapeType = BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
               break;
            case "cylinder":
               shapeType = BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE;
               break;
            case "box":
               shapeType = BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE;
               break;
            case "cone":
               shapeType = BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE;
               break;
            case "capsule":
               shapeType = BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE;
               break;
            default:
               shapeType = 9999;
               break;
         }

         if (shapeType == BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE)
         {
            Sphere3DDefinition sphereShape = (Sphere3DDefinition) collisionShapes.get(j).getGeometryDefinition();
            btSphereShape btSphereShape = (btSphereShape) compoundShape.getChildShape(j);

            assertEquals((float) sphereShape.getRadius(), btSphereShape.getRadius());

         }
         else if (shapeType == BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE)
         {
            Cylinder3DDefinition cylinderShape = (Cylinder3DDefinition) collisionShapes.get(j).getGeometryDefinition();
            btCylinderShape btCylinderShape = (btCylinderShape) compoundShape.getChildShape(j);

            assertEquals((float) cylinderShape.getRadius(), btCylinderShape.getRadius(), EPSILON);
            assertEquals(btCylinderShape.getHalfExtentsWithMargin().z, (float) cylinderShape.getLength() / 2.0f, EPSILON);

         }
         else if (shapeType == BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE)
         {
            Box3DDefinition boxShape = (Box3DDefinition) collisionShapes.get(j).getGeometryDefinition();
            btBoxShape btBoxShape = (btBoxShape) compoundShape.getChildShape(j);

            for (int k = 0; k < btBoxShape.getNumEdges(); k++)
            {
               btBoxShape.getVertex(j, boxVertex);

               assertEquals(Math.abs(boxVertex.x), (float) boxShape.getSizeX() / 2.0f, EPSILON);
               assertEquals(Math.abs(boxVertex.y), (float) boxShape.getSizeY() / 2.0f, EPSILON);
               assertEquals(Math.abs(boxVertex.z), (float) boxShape.getSizeZ() / 2.0f, EPSILON);
            }
         }
         else if (shapeType == BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE)
         {
            Cone3DDefinition coneShape = (Cone3DDefinition) collisionShapes.get(j).getGeometryDefinition();
            btConeShapeZ btConeShape = (btConeShapeZ) compoundShape.getChildShape(j);

            assertEquals(btConeShape.getRadius(), (float) coneShape.getRadius(), EPSILON);
            assertEquals(btConeShape.getHeight(), (float) coneShape.getHeight(), EPSILON);
         }
         else if (shapeType == BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE)
         {
            Capsule3DDefinition capsuleShape = (Capsule3DDefinition) collisionShapes.get(j).getGeometryDefinition();
            btCapsuleShapeZ btCapsuleShapeZ = (btCapsuleShapeZ) compoundShape.getChildShape(j);

            assertEquals(btCapsuleShapeZ.getRadius(), (float) capsuleShape.getRadiusX(), EPSILON);
            assertEquals(btCapsuleShapeZ.getRadius(), (float) capsuleShape.getRadiusY(), EPSILON);
            assertEquals(btCapsuleShapeZ.getRadius(), (float) capsuleShape.getRadiusZ(), EPSILON);
            assertEquals(btCapsuleShapeZ.getHalfHeight(), (float) capsuleShape.getLength() / 2.0f, EPSILON);
         }

         compoundShapeChildTransform = compoundShape.getChildTransform(j);
         collisionShapeRigidBodyTransform.setAndInvert(linkCenterOfMassFrame.getTransformToParent());
         collisionShapeRigidBodyTransform.multiply(new RigidBodyTransform(collisionShapes.get(j).getOriginPose().getRotation(),
                                                                          collisionShapes.get(j).getOriginPose().getTranslation()));

         assertMatrix4EqualsRigidBodyTransform(collisionShapes.get(j).getName(), compoundShapeChildTransform, collisionShapeRigidBodyTransform);
         assertEquals(compoundShape.getChildShape(j).getShapeType(), shapeType);
      }
   }

   private static Robot createRobotToTestLocalTransform(String name,
                                                        YawPitchRollTransformDefinition inertiaPose,
                                                        YawPitchRollTransformDefinition collisionShapePose)
   {
      double boxLength = 0.1;

      // Expressed in frame after joint
      RobotDefinition boxRobot = new RobotDefinition(name);
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");

      rigidBody.getInertiaPose().set(collisionShapePose);
      rootJoint.setSuccessor(rigidBody);
      boxRobot.setRootBodyDefinition(rootBody);

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(boxLength, boxLength, boxLength));
      collisionShapeDefinition.getOriginPose().set(inertiaPose);
      boxRobot.getRigidBodyDefinition(name + "RigidBody").addCollisionShapeDefinition(collisionShapeDefinition);

      return new Robot(boxRobot, ReferenceFrameTools.constructARootFrame("worldFrame"));
   }
   
   private static Robot createTestRobotWithKnownValues()
   {
      String name = "TestRobot";
      RobotDefinition testRobot = new RobotDefinition(name);
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name + "RootJoint");
      rootBody.addChildJoint(rootJoint);
      RigidBodyDefinition rigidBodyDefinition = new RigidBodyDefinition(name + "RigidBody");
      
      MomentOfInertiaDefinition momentOfInertiaRigidBody = new MomentOfInertiaDefinition();
      momentOfInertiaRigidBody.set(0.125568, 8.0E-4, -5.00733E-4, 8.0E-4, 0.0972042, -5.0E-4, -5.00733E-4, -5.0E-4, 0.117936);
      rigidBodyDefinition.setMass(9.609);
      rigidBodyDefinition.setMomentOfInertia(momentOfInertiaRigidBody);
      rigidBodyDefinition.getInertiaPose().setOrientation(0.0, -0.0,  0.0);
      rigidBodyDefinition.getInertiaPose().getTranslation().set(0.012,  0.0,  0.027);

      CollisionShapeDefinition collisionShapeDefinition1 = new CollisionShapeDefinition(new YawPitchRollTransformDefinition(0.046,  0.0,  0.01, 0.0, -0.0,  1.571)
                                                                                        , new Cylinder3DDefinition(0.06, 0.11));
      collisionShapeDefinition1.setCollisionGroup(899);
      collisionShapeDefinition1.setCollisionMask(64);
      rigidBodyDefinition.addCollisionShapeDefinition(collisionShapeDefinition1);
      
      CollisionShapeDefinition collisionShapeDefinition2 = new CollisionShapeDefinition(new YawPitchRollTransformDefinition(-0.03,  0.0,  0.01, 0.0, -0.0,  1.571), 
                                                                                        new Cylinder3DDefinition(0.04, 0.12));
      collisionShapeDefinition1.setCollisionGroup(899);
      collisionShapeDefinition1.setCollisionMask(64);
      rigidBodyDefinition.addCollisionShapeDefinition(collisionShapeDefinition2);
      
      CollisionShapeDefinition collisionShapeDefinition3 = new CollisionShapeDefinition(new YawPitchRollTransformDefinition(0.01,  0.042,  0.09, 0.0, -0.0,  0.0), 
                                                                                        new Cylinder3DDefinition(0.05, 0.16));
      collisionShapeDefinition1.setCollisionGroup(899);
      collisionShapeDefinition1.setCollisionMask(64);
      rigidBodyDefinition.addCollisionShapeDefinition(collisionShapeDefinition3);

      RevoluteJointDefinition revoluteJointDefinition = new RevoluteJointDefinition("RevoluteJoint");
      revoluteJointDefinition.setAxis(Axis3D.Z);

      TransformToParent.setTranslation(-0.013,  0.0,  0.0);
      TransformToParent.setOrientation(0.0,  0.0,  0.0);
      revoluteJointDefinition.setTransformToParent(TransformToParent);
      RigidBodyDefinition jointRigidBodyDefinition = new RigidBodyDefinition(name + "RevoluteJointBody");
      MomentOfInertiaDefinition momentOfInertiaJoint = new MomentOfInertiaDefinition();
      momentOfInertiaJoint.set(0.0039092, -5.04491E-8, -3.42157E-4, -5.04491E-8, 0.00341694, 4.87119E-7, -3.42157E-4, 4.87119E-7, 0.00174492);
      jointRigidBodyDefinition.setMass(2.27);
      jointRigidBodyDefinition.setMomentOfInertia(momentOfInertiaJoint);
      jointRigidBodyDefinition.getInertiaPose().setOrientation(0.0, 0.0,  0.0);
      jointRigidBodyDefinition.getInertiaPose().getTranslation().set(-0.0112984, -3.15366E-6,  0.0746835);
      
      CollisionShapeDefinition jointCollisionShapeDefinition = new CollisionShapeDefinition(new YawPitchRollTransformDefinition(0.0,  0.0, -0.02, 0.0,  0.0,  0.0)
                                                                                          , new Box3DDefinition(0.03, 0.04, 0.02));
      jointCollisionShapeDefinition.setCollisionGroup(3);
      jointCollisionShapeDefinition.setCollisionMask(1);
      jointRigidBodyDefinition.addCollisionShapeDefinition(jointCollisionShapeDefinition);
      revoluteJointDefinition.setSuccessor(jointRigidBodyDefinition);
      rigidBodyDefinition.addChildJoint(revoluteJointDefinition);
      
      rootJoint.setSuccessor(rigidBodyDefinition);
      testRobot.setRootBodyDefinition(rootBody);
      
      return new Robot(testRobot, ReferenceFrameTools.constructARootFrame("worldFrame"));
   }

   private static Robot createTestRobotWithoutBaseCollider(Random random, String name)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name + "RootBody");
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RigidBody");
      int numberOfJoints = random.nextInt(10) + 1;
      OneDoFJointDefinition[] joints = new OneDoFJointDefinition[numberOfJoints];
      RigidBodyDefinition[] jointBodies = new RigidBodyDefinition[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         int jointType = random.nextInt(2);
         switch (jointType)
         {
            case 0:
               joints[i] = new RevoluteJointDefinition(name + "Joint" + i);
               break;
            case 1:
               joints[i] = new PrismaticJointDefinition(name + "Joint" + i);
               break;
            default:
               break;
         }

         joints[i].setAxis(new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));

         TransformToParent.setTranslation(random.nextDouble(), random.nextDouble(), random.nextDouble());
         TransformToParent.setOrientation(random.nextDouble(), random.nextDouble(), random.nextDouble());
         joints[i].setTransformToParent(TransformToParent);
         jointBodies[i] = createRandomShape(name + "JointBody" + i, random);
         joints[i].setSuccessor(jointBodies[i]);

         double lowerLimit = random.nextDouble();
         double upperLimit = lowerLimit + random.nextDouble();
         joints[i].setPositionLowerLimit(lowerLimit);
         joints[i].setPositionUpperLimit(upperLimit);

         if (i == 0)
         {
            joints[i].getTransformToParent().getTranslation().set(new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
            rootBody.addChildJoint(joints[i]);
         }
         else
         {
            jointBodies[i - 1].addChildJoint(joints[i]);
         }
      }
      robotDefinition.setRootBodyDefinition(rootBody);

      return new Robot(robotDefinition, ReferenceFrameTools.constructARootFrame("worldFrame"));
   }

   private static Robot createTestRobotWithBaseCollider(Random random, String name)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name + "RootBody");
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition(name + "RigidBody");
      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("rootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      TransformToParent.setTranslation(random.nextDouble(), random.nextDouble(), random.nextDouble());
      TransformToParent.setOrientation(random.nextDouble(), random.nextDouble(), random.nextDouble());

      SixDoFJointState initialRootJointState = new SixDoFJointState(TransformToParent.getRotation(), TransformToParent.getTranslation());
      rootJointDefinition.setInitialJointState(initialRootJointState);
      OneDoFJointState initialPinJointState = new OneDoFJointState();
      initialPinJointState.setEffort(random.nextDouble());
      RigidBodyDefinition rootJointRigidBodyDefinition = createRandomShape(name + "RootJointBody", random);
      rootJointDefinition.setSuccessor(rootJointRigidBodyDefinition);

      int numberOfJoints = random.nextInt(10) + 1;
      OneDoFJointDefinition[] joints = new OneDoFJointDefinition[numberOfJoints];
      RigidBodyDefinition[] jointBodies = new RigidBodyDefinition[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         int jointType = random.nextInt(2);
         switch (jointType)
         {
            case 0:
               joints[i] = new RevoluteJointDefinition(name + "Joint" + i);
               break;
            case 1:
               joints[i] = new PrismaticJointDefinition(name + "Joint" + i);
               break;
            default:
               break;
         }

         joints[i].setAxis(new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));

         TransformToParent.setTranslation(random.nextDouble(), random.nextDouble(), random.nextDouble());
         TransformToParent.setOrientation(random.nextDouble(), random.nextDouble(), random.nextDouble());
         joints[i].setTransformToParent(TransformToParent);
         jointBodies[i] = createRandomShape(name + "JointBody" + i, random);
         joints[i].setSuccessor(jointBodies[i]);

         double lowerLimit = random.nextDouble();
         double upperLimit = lowerLimit + random.nextDouble();
         joints[i].setPositionLowerLimit(lowerLimit);
         joints[i].setPositionUpperLimit(upperLimit);

         if (i == 0)
         {
            joints[i].getTransformToParent().getTranslation().set(new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
            joints[i].setInitialJointState(initialPinJointState);
            rootJointRigidBodyDefinition.addChildJoint(joints[i]);
         }
         else
         {
            jointBodies[i - 1].addChildJoint(joints[i]);
         }
      }
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      return new Robot(robotDefinition, ReferenceFrameTools.constructARootFrame("worldFrame"));
   }

   private static RigidBodyDefinition createRandomShape(String name, Random random)
   {
      RigidBodyDefinition rigidBodyDefinition = new RigidBodyDefinition(name);
      rigidBodyDefinition.setMass(random.nextDouble());
      rigidBodyDefinition.getMomentOfInertia().setToDiagonal(random.nextDouble(), random.nextDouble(), random.nextDouble());
      rigidBodyDefinition.getInertiaPose().getTranslation().set(random.nextDouble(), random.nextDouble(), random.nextDouble());

      int shapeSelection = random.nextInt(5);
      GeometryDefinition toyGeometryDefinition;

      switch (shapeSelection)
      {
         case 0:
            toyGeometryDefinition = new Sphere3DDefinition(random.nextDouble());
            break;
         case 1:
            toyGeometryDefinition = new Cylinder3DDefinition(random.nextDouble(), random.nextDouble());
            break;
         case 2:
            toyGeometryDefinition = new Box3DDefinition(random.nextDouble(), random.nextDouble(), random.nextDouble());
            break;
         case 3:
            toyGeometryDefinition = new Cone3DDefinition(random.nextDouble(), random.nextDouble());
            break;
         default:
            toyGeometryDefinition = new Capsule3DDefinition(random.nextDouble(), random.nextDouble());
            break;
      }

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(rigidBodyDefinition.getInertiaPose(), toyGeometryDefinition);
      rigidBodyDefinition.addCollisionShapeDefinition(collisionShapeDefinition);

      return rigidBodyDefinition;
   }
}

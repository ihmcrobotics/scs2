package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import com.badlogic.gdx.math.Matrix4;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.simulation.robot.Robot;

public class BulletMultiBodyRobotFactoryTest
{
   private final YawPitchRollTransformDefinition inertiaPose = new YawPitchRollTransformDefinition();
   private final YawPitchRollTransformDefinition collisionShapePose = new YawPitchRollTransformDefinition();
   private CollisionShapeDefinition shapeDefinition = new CollisionShapeDefinition();
   private static final double EPSILON = 1e-5;

   @Test
   public void testBulletCollisionShapeLocalTransform()
   {
      inertiaPose.setTranslation(0.05, 0.08, 0.09);
      inertiaPose.setOrientation(0.01, 0.01, 0.01);
      collisionShapePose.setTranslation(inertiaPose.getX() - 0.01, inertiaPose.getY() - 0.01, inertiaPose.getZ() - 0.01);
      collisionShapePose.setOrientation(inertiaPose.getYaw() - 0.004, inertiaPose.getPitch() - 0.004, inertiaPose.getRoll() - 0.004);

      String robotName = "Robot1";
      Robot robot1 = createTestRobot(robotName, inertiaPose, collisionShapePose);

      shapeDefinition = robot1.getRobotDefinition().getRigidBodyDefinition(robotName + "RigidBody").getCollisionShapeDefinitions().get(0);
      ReferenceFrame linkCenterOfMassFrame = robot1.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      Matrix4 bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition, linkCenterOfMassFrame);

      assertCollisionShapeTransformAsExpected(robotName,
                                              bulletCollisionShapeLocalTransform,
                                              0.999984,
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
                                              0.010000359,
                                              0.0,
                                              0.0,
                                              0.0,
                                              1.0);

      inertiaPose.setTranslation(0.07, 0.02, 0.01);
      inertiaPose.setOrientation(0.03, 0.02, 0.04);
      collisionShapePose.setTranslation(0.02, 0.06, 0.03);
      collisionShapePose.setOrientation(0.04, 0.02, 0.01);

      robotName = "Robot2";
      Robot robot2 = createTestRobot(robotName, inertiaPose, collisionShapePose);

      shapeDefinition = robot2.getRobotDefinition().getRigidBodyDefinition(robotName + "RigidBody").getCollisionShapeDefinitions().get(0);
      linkCenterOfMassFrame = robot2.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition, linkCenterOfMassFrame);
      
      assertCollisionShapeTransformAsExpected(robotName,
                                              bulletCollisionShapeLocalTransform,
                                              0.99995,
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
                                              -0.018608237,
                                              0.0,
                                              0.0,
                                              0.0,
                                              1.0);

      inertiaPose.setTranslation(0.0, 0.0, 0.0);
      inertiaPose.setOrientation(0.0, 0.0, 0.0);
      collisionShapePose.setTranslation(0.0, 0.0, 0.0);
      collisionShapePose.setOrientation(0.0, 0.0, 0.0);

      robotName = "Robot3";
      Robot robot3 = createTestRobot(robotName, inertiaPose, collisionShapePose);

      shapeDefinition = robot3.getRobotDefinition().getRigidBodyDefinition(robotName + "RigidBody").getCollisionShapeDefinitions().get(0);
      linkCenterOfMassFrame = robot3.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition, linkCenterOfMassFrame);

      assertCollisionShapeTransformAsExpected(robotName,
                                              bulletCollisionShapeLocalTransform,
                                              1.0,
                                              0.0,
                                              0.0,
                                              -0.0,
                                              0.0,
                                              1.0,
                                              0.0,
                                              -0.0,
                                              0.0,
                                              0.0,
                                              1.0,
                                              -0.0,
                                              0.0,
                                              0.0,
                                              0.0,
                                              1.0);
   }

   private static void assertCollisionShapeTransformAsExpected(String robotName,
                                                               Matrix4 bulletCollisionShapeLocalTransform,
                                                               double m00,
                                                               double m01,
                                                               double m02,
                                                               double m03,
                                                               double m10,
                                                               double m11,
                                                               double m12,
                                                               double m13,
                                                               double m20,
                                                               double m21,
                                                               double m22,
                                                               double m23,
                                                               double m30,
                                                               double m31,
                                                               double m32,
                                                               double m33)
   {
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M00], m00, EPSILON, robotName + " - M00 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M01], m01, EPSILON, robotName + " - M01 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M02], m02, EPSILON, robotName + " - M02 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M03], m03, EPSILON, robotName + " - M03 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M10], m10, EPSILON, robotName + " - M10 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M11], m11, EPSILON, robotName + " - M11 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M12], m12, EPSILON, robotName + " - M12 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M13], m13, EPSILON, robotName + " - M13 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M20], m20, EPSILON, robotName + " - M20 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M21], m21, EPSILON, robotName + " - M21 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M22], m22, EPSILON, robotName + " - M22 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M23], m23, EPSILON, robotName + " - M23 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M30], m30, EPSILON, robotName + " - M30 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M31], m31, EPSILON, robotName + " - M31 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M32], m32, EPSILON, robotName + " - M32 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M33], m33, EPSILON, robotName + " - M33 is not as expected");
   }

   private static Robot createTestRobot(String name, YawPitchRollTransformDefinition inertiaPose, YawPitchRollTransformDefinition collisionShapePose)
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

}

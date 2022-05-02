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
   private final double epsilon = 1e-5;

   @Test
   public void testBulletCollisionShapeLocalTransform()
   {
      inertiaPose.setTranslation(0.05, 0.08, 0.09);
      inertiaPose.setOrientation(0.01, 0.01, 0.01);
      collisionShapePose.setTranslation(inertiaPose.getX() - 0.01, inertiaPose.getY() - 0.01, inertiaPose.getZ() - 0.01);
      collisionShapePose.setOrientation(inertiaPose.getYaw() - 0.004, inertiaPose.getPitch() - 0.004, inertiaPose.getRoll() - 0.004);

      Robot robot1 = createTestRobot(inertiaPose, collisionShapePose);

      shapeDefinition = robot1.getRobotDefinition().getRigidBodyDefinition("boxRigidBody").getCollisionShapeDefinitions().get(0);
      ReferenceFrame linkCenterOfMassFrame = robot1.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      Matrix4 bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition,
                                                                                                                  linkCenterOfMassFrame);

      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M00], 0.999984, epsilon, "Robot 1 - M00 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M01], -0.003959719, epsilon, "Robot 1 - M01 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M02], 0.004039708, epsilon, "Robot 1 - M02 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M03], 0.009999639, epsilon, "Robot 1 - M03 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M10], 0.0039757174, epsilon, "Robot 1 - M10 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M11], 0.99998426, epsilon, "Robot 1 - M11 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M12], -0.0039599594, epsilon, "Robot 1 - M12 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M13], 0.010000003, epsilon, "Robot 1 - M13 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M20], -0.004023964, epsilon, "Robot 1 - M20 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M21], 0.0039759567, epsilon, "Robot 1 - M21 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M22], 0.999984, epsilon, "Robot 1 - M22 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M23], 0.010000359, epsilon, "Robot 1 - M23 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M30], 0.0, epsilon, "Robot 1 - M30 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M31], 0.0, epsilon, "Robot 1 - M31 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M32], 0.0, epsilon, "Robot 1 - M32 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M33], 1.0, epsilon, "Robot 1 - M33 is not as expected");

      inertiaPose.setTranslation(0.07, 0.02, 0.01);
      inertiaPose.setOrientation(0.03, 0.02, 0.04);
      collisionShapePose.setTranslation(0.02, 0.06, 0.03);
      collisionShapePose.setOrientation(0.04, 0.02, 0.01);

      Robot robot2 = createTestRobot(inertiaPose, collisionShapePose);

      shapeDefinition = robot2.getRobotDefinition().getRigidBodyDefinition("boxRigidBody").getCollisionShapeDefinitions().get(0);
      linkCenterOfMassFrame = robot2.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition,
                                                                                                          linkCenterOfMassFrame);

      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M00], 0.99995, epsilon, "Robot 2 - M00 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M01], 0.009989796, epsilon, "Robot 2 - M01 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M02], -4.0080564E-4, epsilon, "Robot 2 - M02 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M03], 0.048750732, epsilon, "Robot 2 - M03 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M10], -0.009997344, epsilon, "Robot 2 - M10 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M11], 0.9994941, epsilon, "Robot 2 - M11 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M12], -0.030193394, epsilon, "Robot 2 - M12 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M13], -0.042155657, epsilon, "Robot 2 - M13 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M20], 9.897699E-5, epsilon, "Robot 2 - M20 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M21], 0.030195892, epsilon, "Robot 2 - M21 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M22], 0.999544, epsilon, "Robot 2 - M22 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M23], -0.018608237, epsilon, "Robot 2 - M23 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M30], 0.0, epsilon, "Robot 2 - M30 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M31], 0.0, epsilon, "Robot 2 - M31 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M32], 0.0, epsilon, "Robot 2 - M32 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M33], 1.0, epsilon, "Robot 2 - M33 is not as expected");

      inertiaPose.setTranslation(0.0, 0.0, 0.0);
      inertiaPose.setOrientation(0.0, 0.0, 0.0);
      collisionShapePose.setTranslation(0.0, 0.0, 0.0);
      collisionShapePose.setOrientation(0.0, 0.0, 0.0);

      Robot robot3 = createTestRobot(inertiaPose, collisionShapePose);

      shapeDefinition = robot3.getRobotDefinition().getRigidBodyDefinition("boxRigidBody").getCollisionShapeDefinitions().get(0);
      linkCenterOfMassFrame = robot3.getRootBody().getChildrenJoints().get(0).getSuccessor().getBodyFixedFrame();

      bulletCollisionShapeLocalTransform = BulletMultiBodyRobotFactory.bulletCollisionShapeLocalTransform(shapeDefinition,
                                                                                                          linkCenterOfMassFrame);

      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M00], 1.0, epsilon, "Robot 3 - M00 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M01], 0.0, epsilon, "Robot 3 - M01 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M02], 0.0, epsilon, "Robot 3 - M02 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M03], -0.0, epsilon, "Robot 3 - M03 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M10], 0.0, epsilon, "Robot 3 - M10 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M11], 1.0, epsilon, "Robot 3 - M11 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M12], 0.0, epsilon, "Robot 3 - M12 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M13], -0.0, epsilon, "Robot 3 - M13 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M20], 0.0, epsilon, "Robot 3 - M20 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M21], 0.0, epsilon, "Robot 3 - M21 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M22], 1.0, epsilon, "Robot 3 - M22 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M23], -0.0, epsilon, "Robot 3 - M23 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M30], 0.0, epsilon, "Robot 3 - M30 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M31], 0.0, epsilon, "Robot 3 - M31 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M32], 0.0, epsilon, "Robot 3 - M32 is not as expected");
      assertEquals(bulletCollisionShapeLocalTransform.val[Matrix4.M33], 1.0, epsilon, "Robot 3 - M33 is not as expected");
   }

   private static Robot createTestRobot(YawPitchRollTransformDefinition inertiaPose, YawPitchRollTransformDefinition collisionShapePose)
   {
      double boxLength = 0.1;

      // Expressed in frame after joint
      String name = "box";
      RobotDefinition boxRobot = new RobotDefinition("box");
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");

      rigidBody.getInertiaPose().set(collisionShapePose);
      rootJoint.setSuccessor(rigidBody);
      boxRobot.setRootBodyDefinition(rootBody);

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(boxLength, boxLength, boxLength));
      collisionShapeDefinition.getOriginPose().set(inertiaPose);
      boxRobot.getRigidBodyDefinition("boxRigidBody").addCollisionShapeDefinition(collisionShapeDefinition);

      return new Robot(boxRobot, ReferenceFrameTools.constructARootFrame("worldFrame"));
   }

}

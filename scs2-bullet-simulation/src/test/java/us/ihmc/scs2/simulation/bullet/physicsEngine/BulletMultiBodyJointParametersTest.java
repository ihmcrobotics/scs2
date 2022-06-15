package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.Random;

import org.bytedeco.bullet.BulletDynamics.btMultiBody;
import org.bytedeco.bullet.BulletDynamics.btMultiBodyLinkCollider;
import org.bytedeco.bullet.LinearMath.btQuaternion;
import org.bytedeco.bullet.LinearMath.btVector3;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;

public class BulletMultiBodyJointParametersTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testDefaultBulletMultiBodyJointParameters()
   {
      //create default BulletMultiBodyJointParameters
      BulletMultiBodyJointParameters jointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();

      //create a simple btMultiBody with one linkCollider
      btVector3 intertia = new btVector3();
      btVector3 parentComToCurrentCom = new btVector3(0f, 0.05f * 2.f, 0f);
      btVector3 currentPivotToCurrentCom = new btVector3(0f, -0.05f, 0f);
      btVector3 hingeJointAxis = new btVector3(1, 0, 0);
      btVector3 parentComToCurrentPivot = new btVector3(parentComToCurrentCom.getX() - currentPivotToCurrentCom.getX(),
                                                        parentComToCurrentCom.getY() - currentPivotToCurrentCom.getY(),
                                                        parentComToCurrentCom.getZ() - currentPivotToCurrentCom.getZ());
      btQuaternion rotParentToThis = new btQuaternion(0.f, 0.f, 0.f, 1.f);

      btMultiBody btMultiBody = new btMultiBody(1, 1.0f, intertia, false, true);
      btMultiBody.setupRevolute(0, 0, intertia, -1, rotParentToThis, hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom);
      btMultiBodyLinkCollider linkCollider = new btMultiBodyLinkCollider(btMultiBody, 0);

      //The default BulletMultiBodyJointParameters should be the same as a newly created linkCollider.
      assertJointParametersEquals(true,
                                  linkCollider.getFriction(),
                                  linkCollider.getRestitution(),
                                  linkCollider.getHitFraction(),
                                  linkCollider.getRollingFriction(),
                                  linkCollider.getSpinningFriction(),
                                  linkCollider.getContactProcessingThreshold(),
                                  jointParameters);
   }

   @Test
   public void testConstructor()
   {
      Random random = new Random(346742);

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean jointDisableParentCollision = random.nextBoolean();
         double jointFriction = random.nextDouble();
         double jointRestitution = random.nextDouble();
         double jointHitFration = random.nextDouble();
         double jointRollingFriction = random.nextDouble();
         double jointSpinningFriction = random.nextDouble();
         double jointContactProcessingThreshold = random.nextDouble();

         BulletMultiBodyJointParameters jointParameters = new BulletMultiBodyJointParameters(jointDisableParentCollision,
                                                                                             jointFriction,
                                                                                             jointRestitution,
                                                                                             jointHitFration,
                                                                                             jointRollingFriction,
                                                                                             jointSpinningFriction,
                                                                                             jointContactProcessingThreshold);

         assertJointParametersEquals(jointDisableParentCollision,
                                     jointFriction,
                                     jointRestitution,
                                     jointHitFration,
                                     jointRollingFriction,
                                     jointSpinningFriction,
                                     jointContactProcessingThreshold,
                                     jointParameters);
      }
   }

   @Test
   public void testSetters()
   {
      Random random = new Random(125846);

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean jointDisableParentCollision = random.nextBoolean();
         double jointFriction = random.nextDouble();
         double jointRestitution = random.nextDouble();
         double jointHitFration = random.nextDouble();
         double jointRollingFriction = random.nextDouble();
         double jointSpinningFriction = random.nextDouble();
         double jointContactProcessingThreshold = random.nextDouble();

         BulletMultiBodyJointParameters jointParameters = new BulletMultiBodyJointParameters();

         jointParameters.setJointDisableParentCollision(jointDisableParentCollision);
         jointParameters.setJointFriction(jointFriction);
         jointParameters.setJointRestitution(jointRestitution);
         jointParameters.setJointHitFraction(jointHitFration);
         jointParameters.setJointRollingFriction(jointRollingFriction);
         jointParameters.setJointSpinningFriction(jointSpinningFriction);
         jointParameters.setJointContactProcessingThreshold(jointContactProcessingThreshold);

         assertJointParametersEquals(jointDisableParentCollision,
                                     jointFriction,
                                     jointRestitution,
                                     jointHitFration,
                                     jointRollingFriction,
                                     jointSpinningFriction,
                                     jointContactProcessingThreshold,
                                     jointParameters);
      }
   }

   private static void assertJointParametersEquals(boolean jointDisableParentCollision,
                                                   double jointFriction,
                                                   double jointRestitution,
                                                   double jointHitFration,
                                                   double jointRollingFriction,
                                                   double jointSpinningFriction,
                                                   double jointContactProcessingThreshold,
                                                   BulletMultiBodyJointParameters jointParameters)
   {
      Assertions.assertEquals(jointDisableParentCollision, jointParameters.getJointDisableParentCollision());
      Assertions.assertEquals((float) jointFriction, (float) jointParameters.getJointFriction());
      Assertions.assertEquals((float) jointRestitution, (float) jointParameters.getJointRestitution());
      Assertions.assertEquals((float) jointHitFration, (float) jointParameters.getJointHitFraction());
      Assertions.assertEquals((float) jointRollingFriction, (float) jointParameters.getJointRollingFriction());
      Assertions.assertEquals((float) jointSpinningFriction, (float) jointParameters.getJointSpinningFriction());
      Assertions.assertEquals((float) jointContactProcessingThreshold, (float) jointParameters.getJointContactProcessingThreshold());
   }
}

package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;

import org.bytedeco.bullet.BulletDynamics.btMultiBody;
import org.bytedeco.bullet.LinearMath.btVector3;
import org.junit.jupiter.api.Test;

public class BulletMultiBodyParametersTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testDefaultBulletMultiBodyParameters()
   {
      //create default BulletMultiBodyParameters
      BulletMultiBodyParameters parameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();

      //create a simple btMultiBody 
      btVector3 intertia = new btVector3();
      btMultiBody btMultiBody = new btMultiBody(0, 1.0f, intertia, false, parameters.getCanSleep());

      //The default BulletMultiBodyParameters should be the same as a newly created btMultiBody
      assertParametersEqual(btMultiBody.getCanSleep(),
                            btMultiBody.hasSelfCollision(),
                            btMultiBody.getUseGyroTerm(),
                            btMultiBody.isUsingGlobalVelocities(),
                            btMultiBody.isUsingRK4Integration(),
                            btMultiBody.getLinearDamping(),
                            btMultiBody.getAngularDamping(),
                            btMultiBody.getMaxAppliedImpulse(),
                            btMultiBody.getMaxCoordinateVelocity(),
                            parameters);
   }

   @Test
   public void testConstructor()
   {
      Random random = new Random(9531271);

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean canSleep = random.nextBoolean();
         boolean hasSelfCollision = random.nextBoolean();
         boolean useGyroTerm = random.nextBoolean();
         boolean useGlobalVelocities = random.nextBoolean();
         boolean useRK4Integration = random.nextBoolean();
         double linearDamping = random.nextDouble();
         double angularDamping = random.nextDouble();
         double maxAppliedImpulse = random.nextDouble();
         double maxCoordinateVelocity = random.nextDouble();

         BulletMultiBodyParameters parameters = new BulletMultiBodyParameters(canSleep,
                                                                              hasSelfCollision,
                                                                              useGyroTerm,
                                                                              useGlobalVelocities,
                                                                              useRK4Integration,
                                                                              linearDamping,
                                                                              angularDamping,
                                                                              maxAppliedImpulse,
                                                                              maxCoordinateVelocity);

         assertParametersEqual(canSleep,
                               hasSelfCollision,
                               useGyroTerm,
                               useGlobalVelocities,
                               useRK4Integration,
                               linearDamping,
                               angularDamping,
                               maxAppliedImpulse,
                               maxCoordinateVelocity,
                               parameters);
      }
   }

   @Test
   public void testSetters()
   {
      Random random = new Random(458791);

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean canSleep = random.nextBoolean();
         boolean hasSelfCollision = random.nextBoolean();
         boolean useGyroTerm = random.nextBoolean();
         boolean useGlobalVelocities = random.nextBoolean();
         boolean useRK4Integration = random.nextBoolean();
         double linearDamping = random.nextDouble();
         double angularDamping = random.nextDouble();
         double maxAppliedImpulse = random.nextDouble();
         double maxCoordinateVelocity = random.nextDouble();

         BulletMultiBodyParameters parameters = new BulletMultiBodyParameters();

         parameters.setCanSleep(canSleep);
         parameters.setHasSelfCollision(hasSelfCollision);
         parameters.setUseGyroTerm(useGyroTerm);
         parameters.setUseGlobalVelocities(useGlobalVelocities);
         parameters.setUseRK4Integration(useRK4Integration);
         parameters.setLinearDamping(linearDamping);
         parameters.setAngularDamping(angularDamping);
         parameters.setMaxAppliedImpulse(maxAppliedImpulse);
         parameters.setMaxCoordinateVelocity(maxCoordinateVelocity);

         assertParametersEqual(canSleep,
                               hasSelfCollision,
                               useGyroTerm,
                               useGlobalVelocities,
                               useRK4Integration,
                               linearDamping,
                               angularDamping,
                               maxAppliedImpulse,
                               maxCoordinateVelocity,
                               parameters);
      }
   }

   private static void assertParametersEqual(boolean canSleep,
                                             boolean hasSelfCollision,
                                             boolean useGyroTerm,
                                             boolean useGlobalVelocities,
                                             boolean useRK4Integration,
                                             double linearDamping,
                                             double angularDamping,
                                             double maxAppliedImpulse,
                                             double maxCoordinateVelocity,
                                             BulletMultiBodyParameters parameters)
   {
      assertEquals(canSleep, parameters.getCanSleep());
      assertEquals(hasSelfCollision, parameters.getHasSelfCollision());
      assertEquals(useGyroTerm, parameters.getUseGyroTerm());
      assertEquals(useGlobalVelocities, parameters.getUseGlobalVelocities());
      assertEquals(useRK4Integration, parameters.getUseRK4Integration());
      assertEquals((float) linearDamping, (float) parameters.getLinearDamping());
      assertEquals((float) angularDamping, (float) parameters.getAngularDamping());
      assertEquals((float) maxAppliedImpulse, (float) parameters.getMaxAppliedImpulse());
      assertEquals((float) maxCoordinateVelocity, (float) parameters.getMaxCoordinateVelocity());
   }
}

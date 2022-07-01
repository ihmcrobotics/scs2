package us.ihmc.scs2.simulation.bullet.physicsEngine;

import org.junit.jupiter.api.Test;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;

public class BulletFlyingBallNoSCS2Test
{
   static
   {
      Bullet.init();
      LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());
   }
   
   private static final double EPSILON = 0.1;
   
   @Test
   public void testHeightAfterSeconds()
   {
      double dt = 0.1;
      float ballMass = 1;
      float ballRadius = 0.5f;
      double gravity = -9.81;
      Vector3 baseInertiaDiag = new Vector3();
      boolean isFixed = false;
      
      BulletMultiBodyDynamicsWorld bulletMultiBodyDynamicsWorld = new BulletMultiBodyDynamicsWorld();

      btCollisionShape childShape = new btSphereShape(ballRadius);
      childShape.calculateLocalInertia(ballMass, baseInertiaDiag);
      BulletMultiBodyRobot bulletMultiBody = new BulletMultiBodyRobot(0, ballMass, baseInertiaDiag, isFixed, false, null);

      Matrix4 startTrans = new Matrix4();
      startTrans.set(0, 0, 0, 0, 0, 0, 0);
      bulletMultiBody.getBtMultiBody().setBaseWorldTransform(startTrans);

      BulletMultiBodyLinkCollider linkCollider = new BulletMultiBodyLinkCollider(bulletMultiBody.getBtMultiBody(), -1, null);
      linkCollider.setCollisionShape(childShape);
      linkCollider.setCollisionGroupMask(1, -1);

      bulletMultiBody.addBulletMuliBodyLinkCollider(linkCollider);
      bulletMultiBody.getBtMultiBody().setBaseCollider(linkCollider.getBtMultiBodyLinkCollider());
      
      bulletMultiBody.getBtMultiBody().setLinearDamping(0);
      bulletMultiBody.getBtMultiBody().setMaxCoordinateVelocity(1000000);
      bulletMultiBody.getBtMultiBody().useRK4Integration(true);
      
      bulletMultiBody.getBtMultiBody().finalizeMultiDof();
      
      btMultiBody btMultiBody = bulletMultiBody.getBtMultiBody();
      
      bulletMultiBodyDynamicsWorld.addBulletMultiBodyRobot(bulletMultiBody);
      bulletMultiBodyDynamicsWorld.setGravity(new Vector3D(0.0, 0.0, gravity));
      
      Point3D initialPosition = new Point3D(0, 0, 0);
      Vector3D initialVelocity = new Vector3D(0, 0, 0);
      double seconds = dt;
      Point3D actual = new Point3D();
      Point3D expectedTest = new Point3D();
      
      Vector3 prevPosition = new Vector3(0, 0, 0);
      Vector3 prevVelocity = new Vector3(0, 0, 0);
      Vector3 gravityVector = new Vector3(0, 0, -9.81f);
      
      for (int i = 1; i < 2001; i++)
      {
         int output = bulletMultiBodyDynamicsWorld.stepSimulation((float)dt, 1, (float)dt);
         
         Vector3D expected = heightAfterSeconds(initialPosition, initialVelocity, seconds, gravity);
         Vector3 expectedFloat = heightAfterSecondsFloat(prevPosition, prevVelocity, (float)dt, gravityVector);
          
         BulletTools.toEuclid(btMultiBody.getBasePos(), actual); 
         BulletTools.toEuclid(expectedFloat, expectedTest);
         //EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedTest, actual, EPSILON);
         
         System.out.println(output + " " + i + " " + btMultiBody.getBasePos().z + " " + expected.getZ() + " : " + expectedFloat.z);
         
         prevPosition.set(btMultiBody.getBasePos().x, btMultiBody.getBasePos().y, btMultiBody.getBasePos().z);
         prevVelocity.set(btMultiBody.getBaseVel().x, btMultiBody.getBaseVel().y, btMultiBody.getBaseVel().z);
         
         seconds += dt;
      }
      
      bulletMultiBodyDynamicsWorld.dispose();
   }
   
   private static Vector3D heightAfterSeconds(Point3D initialPosition, Vector3D initialVelocity, double seconds, double gravity)
   {
      //H(time) = -1/2 * g * t^2 + V(initial) * t + H(initial)
      Vector3D height = new Vector3D();

      height.setX(initialVelocity.getX() * seconds + initialPosition.getX());
      height.setY(initialVelocity.getY() * seconds + initialPosition.getY());
      height.setZ(0.5 * gravity * seconds * seconds + initialVelocity.getZ() * seconds + initialPosition.getZ());

      return height;
   }
   
   private static Vector3 heightAfterSecondsFloat(Vector3 previousPosition, Vector3 previousVelocity, float seconds, Vector3 gravity)
   {
      //H(time) = -1/2 * g * t^2 + V(initial) * t + H(initial)
      Vector3 height = new Vector3();

      height.x = 0.5f * gravity.x * seconds * seconds + previousVelocity.x * seconds + previousPosition.x;
      height.y = 0.5f * gravity.y * seconds * seconds + previousVelocity.y * seconds + previousPosition.y;
      height.z = 0.5f * gravity.z * seconds * seconds + previousVelocity.z * seconds + previousPosition.z;

      return height;
   }
}

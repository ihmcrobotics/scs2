package us.ihmc.scs2.examples.simulations.bullet;

import java.util.ArrayList;

import org.bytedeco.bullet.BulletCollision.btCollisionShape;
import org.bytedeco.bullet.BulletCollision.btCompoundShape;
import org.bytedeco.bullet.BulletCollision.btSphereShape;
import org.bytedeco.bullet.BulletDynamics.btMultiBody;
import org.bytedeco.bullet.LinearMath.btTransform;
import org.bytedeco.bullet.LinearMath.btVector3;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyDynamicsWorld;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyLinkCollider;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyRobot;

public class SimpleSphereUsingBulletNoSCS2
{
   public static void main(String[] args)
   {
      double dt = 0.1;
      float ballMass = 1;
      float ballRadius = 0.5f;
      double gravity = -9.81;
      btVector3 baseInertiaDiag = new btVector3();
      boolean isFixed = false;

      BulletMultiBodyDynamicsWorld bulletMultiBodyDynamicsWorld = new BulletMultiBodyDynamicsWorld();

      btCompoundShape bulletCompoundShape = new btCompoundShape();
      btCollisionShape childShape = new btSphereShape(ballRadius);
      bulletCompoundShape.addChildShape(btTransform.getIdentity(), childShape);

      ArrayList<btCollisionShape> btCollisionShapes = new ArrayList<>();
      btCollisionShapes.add(childShape);

      childShape.calculateLocalInertia(ballMass, baseInertiaDiag);
      BulletMultiBodyRobot bulletMultiBody = new BulletMultiBodyRobot(0, ballMass, baseInertiaDiag, isFixed, false, null);

      btTransform startTrans = new btTransform();
      startTrans.setIdentity();
      bulletMultiBody.getBtMultiBody().setBaseWorldTransform(startTrans);

      BulletMultiBodyLinkCollider linkCollider = new BulletMultiBodyLinkCollider(bulletMultiBody.getBtMultiBody(), -1, null);
      linkCollider.setCollisionShape(bulletCompoundShape, btCollisionShapes);
      linkCollider.setCollisionGroupMask(1, -1);

      bulletMultiBody.addBulletMuliBodyLinkCollider(linkCollider);
      bulletMultiBody.getBtMultiBody().setBaseCollider(linkCollider.getBtMultiBodyLinkCollider());

      bulletMultiBody.getBtMultiBody().finalizeMultiDof();

      btMultiBody btMultiBody = bulletMultiBody.getBtMultiBody();

      bulletMultiBodyDynamicsWorld.addBulletMultiBodyRobot(bulletMultiBody);
      bulletMultiBodyDynamicsWorld.setGravity(new Vector3D(0.0, 0.0, gravity));

      for (int i = 1; i < 100; i++)
      {
         bulletMultiBodyDynamicsWorld.stepSimulation((float) dt, 1, (float) dt);

         btVector3 position = btMultiBody.getBasePos();
         System.out.println(i + " " + position.z());
      }

      bulletMultiBodyDynamicsWorld.dispose();
   }
}

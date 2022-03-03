package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.ContactResultCallback;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObjectWrapper;
import com.badlogic.gdx.physics.bullet.collision.btManifoldPoint;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public class BulletContactPairTest
{
   private final ArrayList<btManifoldPoint> contactPoints = new ArrayList<>();
   private final YoRegistry yoRegistry = new YoRegistry("InteractionForcesDemo");
   private final YoInteger numberOfContactPoints = new YoInteger("numberOfContactPoints", yoRegistry);
   private final YoDouble pointAX = new YoDouble("pointAX", yoRegistry);
   private final YoDouble pointAY = new YoDouble("pointAY", yoRegistry);
   private final YoDouble pointAZ = new YoDouble("pointAZ", yoRegistry);
   private final YoDouble pointBX = new YoDouble("pointBX", yoRegistry);
   private final YoDouble pointBY = new YoDouble("pointBY", yoRegistry);
   private final YoDouble pointBZ = new YoDouble("pointBZ", yoRegistry);
   private final YoDouble normalX = new YoDouble("normalX", yoRegistry);
   private final YoDouble normalY = new YoDouble("normalY", yoRegistry);
   private final YoDouble normalZ = new YoDouble("normalZ", yoRegistry);
   private final YoDouble distance = new YoDouble("distance", yoRegistry);
   private final YoDouble appliedImpulse = new YoDouble("appliedImpulse", yoRegistry);
   private final YoDouble appliedImpulseLateral1 = new YoDouble("appliedImpulseLateral1", yoRegistry);
   private final YoDouble appliedImpulseLateral2 = new YoDouble("appliedImpulseLateral2", yoRegistry);
   private final YoDouble combinedContactDamping1 = new YoDouble("combinedContactDamping1", yoRegistry);
   private final YoDouble combinedFriction = new YoDouble("combinedFriction", yoRegistry);
   private final YoDouble combinedRestitution = new YoDouble("combinedRestitution", yoRegistry);
   private final YoDouble combinedRollingFriction = new YoDouble("combinedRollingFriction", yoRegistry);
   private final YoDouble combinedSpinningFriction = new YoDouble("combinedSpinningFriction", yoRegistry);
   private final YoDouble contactCFM = new YoDouble("contactCFM", yoRegistry);
   private final YoDouble contactERP = new YoDouble("contactERP", yoRegistry);
   private final YoDouble contactMotion1 = new YoDouble("contactMotion1", yoRegistry);
   private final YoDouble contactMotion2 = new YoDouble("contactMotion2", yoRegistry);
   private final YoInteger contactPointFlags = new YoInteger("contactPointFlags", yoRegistry);
   private final YoDouble distance1 = new YoDouble("distance1", yoRegistry);
   private final YoDouble frictionCFM = new YoDouble("frictionCFM", yoRegistry);
   private final YoDouble lateralFrictionDirection1X = new YoDouble("lateralFrictionDirection1X", yoRegistry);
   private final YoDouble lateralFrictionDirection1Y = new YoDouble("lateralFrictionDirection1Y", yoRegistry);
   private final YoDouble lateralFrictionDirection1Z = new YoDouble("lateralFrictionDirection1Z", yoRegistry);
   private final YoDouble lateralFrictionDirection2X = new YoDouble("lateralFrictionDirection2X", yoRegistry);
   private final YoDouble lateralFrictionDirection2Y = new YoDouble("lateralFrictionDirection2Y", yoRegistry);
   private final YoDouble lateralFrictionDirection2Z = new YoDouble("lateralFrictionDirection2Z", yoRegistry);
   private final YoInteger lifeTime = new YoInteger("lifeTime", yoRegistry);

   public BulletContactPairTest(btMultiBodyDynamicsWorld multiBodyDynamicsWorld,
                                BulletInternalTickCallbackRegistry internalTickCallbackRegistry,
                                btCollisionObject collisionObjectA,
                                btCollisionObject collisionObjectB)
   {
      internalTickCallbackRegistry.getPostTickRunnables().add(() ->
      {
         ContactResultCallback contactResultCallback = new ContactResultCallback()
         {
            @Override
            public float addSingleResult(btManifoldPoint contactPoint,
                                         btCollisionObjectWrapper collisionObjectAWrapper,
                                         int partIdA,
                                         int indexA,
                                         btCollisionObjectWrapper collisionObjectBWrapper,
                                         int partIdB,
                                         int indexB)
            {
               contactPoints.add(contactPoint);
               return 0;
            }
         };
         contactPoints.clear();
         multiBodyDynamicsWorld.contactPairTest(collisionObjectA, collisionObjectB, contactResultCallback);
         numberOfContactPoints.set(contactPoints.size());
         for (btManifoldPoint contactPoint : contactPoints)
         {
            Vector3 pointOnA = new Vector3();
            contactPoint.getPositionWorldOnA(pointOnA);
            pointAX.set(pointOnA.x);
            pointAY.set(pointOnA.y);
            pointAZ.set(pointOnA.z);

            RigidBodyTransform arrowTransform = new RigidBodyTransform();
            Vector3 normalOnBGDX = new Vector3();
            contactPoint.getNormalWorldOnB(normalOnBGDX);
            normalX.set(normalOnBGDX.x);
            normalY.set(normalOnBGDX.y);
            normalZ.set(normalOnBGDX.z);
            Vector3D normalOnBEuclid = new Vector3D();
            BulletTools.toEuclid(normalOnBGDX, normalOnBEuclid);
            normalOnBEuclid.normalize();
            normalX.set(normalOnBEuclid.getX());
            normalY.set(normalOnBEuclid.getY());
            normalZ.set(normalOnBEuclid.getZ());
            Quaternion arrowOrientation = new Quaternion();
            EuclidGeometryTools.orientation3DFromZUpToVector3D(normalOnBEuclid, arrowOrientation);
            Vector3 pointOnB = new Vector3();
            contactPoint.getPositionWorldOnB(pointOnB);
            pointBX.set(pointOnB.x);
            pointBY.set(pointOnB.y);
            pointBZ.set(pointOnB.z);
            Point3D pointOnBEuclid = new Point3D();
            BulletTools.toEuclid(pointOnB, pointOnBEuclid);
            arrowTransform.set(arrowOrientation, pointOnBEuclid);

            float distance = contactPoint.getDistance();
            float appliedImpulse = contactPoint.getAppliedImpulse();
            float appliedImpulseLateral1 = contactPoint.getAppliedImpulseLateral1();
            float appliedImpulseLateral2 = contactPoint.getAppliedImpulseLateral2();
            float combinedContactDamping1 = contactPoint.getCombinedContactDamping1();
            float combinedFriction = contactPoint.getCombinedFriction();
            float combinedRestitution = contactPoint.getCombinedRestitution();
            float combinedRollingFriction = contactPoint.getCombinedRollingFriction();
            float combinedSpinningFriction = contactPoint.getCombinedSpinningFriction();
            float contactCFM = contactPoint.getContactCFM();
            float contactERP = contactPoint.getContactERP();
            float contactMotion1 = contactPoint.getContactMotion1();
            float contactMotion2 = contactPoint.getContactMotion2();
            int contactPointFlags = contactPoint.getContactPointFlags();
            float distance1 = contactPoint.getDistance1();
            float frictionCFM = contactPoint.getFrictionCFM();
            Vector3 lateralFrictionDirection1 = new Vector3();
            contactPoint.getLateralFrictionDir1(lateralFrictionDirection1);
            Vector3 lateralFrictionDirection2 = new Vector3();
            contactPoint.getLateralFrictionDir2(lateralFrictionDirection2);
            int lifeTime = contactPoint.getLifeTime();

            this.distance.set(distance);
            this.appliedImpulse.set(appliedImpulse);
            this.appliedImpulseLateral1.set(appliedImpulseLateral1);
            this.appliedImpulseLateral2.set(appliedImpulseLateral2);
            this.combinedContactDamping1.set(combinedContactDamping1);
            this.combinedFriction.set(combinedFriction);
            this.combinedRestitution.set(combinedRestitution);
            this.combinedRollingFriction.set(combinedRollingFriction);
            this.combinedSpinningFriction.set(combinedSpinningFriction);
            this.contactCFM.set(contactCFM);
            this.contactERP.set(contactERP);
            this.contactMotion1.set(contactMotion1);
            this.contactMotion2.set(contactMotion2);
            this.contactPointFlags.set(contactPointFlags);
            this.distance1.set(distance1);
            this.frictionCFM.set(frictionCFM);
            this.lateralFrictionDirection1X.set(lateralFrictionDirection1.x);
            this.lateralFrictionDirection1Y.set(lateralFrictionDirection1.y);
            this.lateralFrictionDirection1Z.set(lateralFrictionDirection1.z);
            this.lateralFrictionDirection2X.set(lateralFrictionDirection2.x);
            this.lateralFrictionDirection2Y.set(lateralFrictionDirection2.y);
            this.lateralFrictionDirection2Z.set(lateralFrictionDirection2.z);
            this.lifeTime.set(lifeTime);
         }
      });
   }
}

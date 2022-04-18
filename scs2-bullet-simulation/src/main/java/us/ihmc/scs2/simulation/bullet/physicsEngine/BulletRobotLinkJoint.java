package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.physics.bullet.dynamics.btMultibodyLink;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BulletRobotLinkJoint extends BulletRobotLinkBasics
{
   private final SimRevoluteJoint simRevoluteJoint;
   private YoDouble bulletLinkAppliedForceX;
   private YoDouble bulletLinkAppliedForceY;
   private YoDouble bulletLinkAppliedForceZ;
   private YoDouble bulletLinkAppliedTorqueX;
   private YoDouble bulletLinkAppliedTorqueY;
   private YoDouble bulletLinkAppliedTorqueZ;

   public BulletRobotLinkJoint(SimRevoluteJoint simRevoluteJoint,
                               int bulletJointIndex,
                               RigidBodyWrenchRegistry rigidBodyWrenchRegistry,
                               YoRegistry yoRegistry,
                               BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider)
   {
      super(simRevoluteJoint.getSuccessor(), rigidBodyWrenchRegistry, bulletMultiBodyLinkCollider);
      this.simRevoluteJoint = simRevoluteJoint;

      bulletLinkAppliedForceX = new YoDouble(simRevoluteJoint.getName() + "_btAppliedForceX", yoRegistry);
      bulletLinkAppliedForceY = new YoDouble(simRevoluteJoint.getName() + "_btAppliedForceY", yoRegistry);
      bulletLinkAppliedForceZ = new YoDouble(simRevoluteJoint.getName() + "_btAppliedForceZ", yoRegistry);
      bulletLinkAppliedTorqueX = new YoDouble(simRevoluteJoint.getName() + "_btAppliedTorqueX", yoRegistry);
      bulletLinkAppliedTorqueY = new YoDouble(simRevoluteJoint.getName() + "_btAppliedTorqueY", yoRegistry);
      bulletLinkAppliedTorqueZ = new YoDouble(simRevoluteJoint.getName() + "_btAppliedTorqueZ", yoRegistry);
   }

   public void copyDataFromSCSToBullet()
   {
      updateBulletLinkColliderTransformFromMecanoRigidBody();

      getBulletMultiBodyLinkCollider().getMultiBody().setJointPos(getBulletJointIndex(), (float) simRevoluteJoint.getQ());
      getBulletMultiBodyLinkCollider().getMultiBody().setJointVel(getBulletJointIndex(), (float) simRevoluteJoint.getQd());
      getBulletMultiBodyLinkCollider().getMultiBody().addJointTorque(getBulletJointIndex(), (float) simRevoluteJoint.getTau());
   }

   public void copyBulletJointDataToSCS(double dt)
   {
      float jointPosition = getBulletMultiBodyLinkCollider().getMultiBody().getJointPos(getBulletJointIndex());
      simRevoluteJoint.setQ(jointPosition);
      float jointPVel = getBulletMultiBodyLinkCollider().getMultiBody().getJointVel(getBulletJointIndex());
      simRevoluteJoint.setQdd((jointPVel - simRevoluteJoint.getQd()) / dt);
      simRevoluteJoint.setQd(jointPVel);

      btVector3 linkForce = getBulletMultiBodyLinkCollider().getMultiBody().getLink(getBulletJointIndex()).getAppliedConstraintForce();
      bulletLinkAppliedForceX.set(linkForce.getX());
      bulletLinkAppliedForceY.set(linkForce.getY());
      bulletLinkAppliedForceZ.set(linkForce.getZ());
      btVector3 linkTorque = getBulletMultiBodyLinkCollider().getMultiBody().getLink(getBulletJointIndex()).getAppliedConstraintTorque();
      bulletLinkAppliedTorqueX.set(linkTorque.getX());
      bulletLinkAppliedTorqueY.set(linkTorque.getY());
      bulletLinkAppliedTorqueZ.set(linkTorque.getZ());

      ReferenceFrame bodyFrame = getSimRigidBody().getBodyFixedFrame();
      ReferenceFrame expressedInFrame = getSimRigidBody().getBodyFixedFrame();
      Vector3DReadOnly torque = new Vector3D((double) linkTorque.getX(), (double) linkTorque.getY(), (double) linkTorque.getZ());
      Vector3DReadOnly force = new Vector3D((double) linkForce.getX(), (double) linkForce.getY(), (double) linkForce.getZ());
      getRigidBodyWrenchRegistry().addWrench(getSimRigidBody(), new Wrench(bodyFrame, expressedInFrame, torque, force));
   }

   public YoDouble getBulletLinkAppliedForceX()
   {
      return bulletLinkAppliedForceX;
   }

   public YoDouble getBulletLinkAppliedForceY()
   {
      return bulletLinkAppliedForceY;
   }

   public YoDouble getBulletLinkAppliedForceZ()
   {
      return bulletLinkAppliedForceZ;
   }

   public YoDouble getBulletLinkAppliedTorqueX()
   {
      return bulletLinkAppliedTorqueX;
   }

   public YoDouble getBulletLinkAppliedTorqueY()
   {
      return bulletLinkAppliedTorqueY;
   }

   public YoDouble getBulletLinkAppliedTorqueZ()
   {
      return bulletLinkAppliedTorqueZ;
   }
}

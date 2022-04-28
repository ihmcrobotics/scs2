package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BulletRobotLinkJoint extends BulletRobotLinkBasics
{
   private final SimOneDoFJointBasics simOneDofJoint;
   private final Vector3D force = new Vector3D();
   private final Vector3D torque = new Vector3D();
   private final Wrench wrenchToAdd;
   
   private YoDouble bulletLinkAppliedForceX;
   private YoDouble bulletLinkAppliedForceY;
   private YoDouble bulletLinkAppliedForceZ;
   private YoDouble bulletLinkAppliedTorqueX;
   private YoDouble bulletLinkAppliedTorqueY;
   private YoDouble bulletLinkAppliedTorqueZ;

   public BulletRobotLinkJoint(SimOneDoFJointBasics simOneDoFJoint,
                               int bulletJointIndex,
                               RigidBodyWrenchRegistry rigidBodyWrenchRegistry,
                               YoRegistry yoRegistry,
                               BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider)
   {
      super(simOneDoFJoint.getSuccessor(), rigidBodyWrenchRegistry, bulletMultiBodyLinkCollider);
      this.simOneDofJoint = simOneDoFJoint;
      
      ReferenceFrame bodyFrame = getSimRigidBody().getBodyFixedFrame();
      ReferenceFrame expressedInFrame = getSimRigidBody().getBodyFixedFrame();
      wrenchToAdd = new Wrench(bodyFrame, expressedInFrame);

      bulletLinkAppliedForceX = new YoDouble(simOneDoFJoint.getName() + "_btAppliedForceX", yoRegistry);
      bulletLinkAppliedForceY = new YoDouble(simOneDoFJoint.getName() + "_btAppliedForceY", yoRegistry);
      bulletLinkAppliedForceZ = new YoDouble(simOneDoFJoint.getName() + "_btAppliedForceZ", yoRegistry);
      bulletLinkAppliedTorqueX = new YoDouble(simOneDoFJoint.getName() + "_btAppliedTorqueX", yoRegistry);
      bulletLinkAppliedTorqueY = new YoDouble(simOneDoFJoint.getName() + "_btAppliedTorqueY", yoRegistry);
      bulletLinkAppliedTorqueZ = new YoDouble(simOneDoFJoint.getName() + "_btAppliedTorqueZ", yoRegistry);
   }

   public void copyDataFromSCSToBullet()
   {
      updateBulletLinkColliderTransformFromMecanoRigidBody();

      getBulletMultiBodyLinkCollider().setJointPos(simOneDofJoint.getQ());
      getBulletMultiBodyLinkCollider().setJointVel(simOneDofJoint.getQd());
      getBulletMultiBodyLinkCollider().addJointTorque(simOneDofJoint.getTau());
   }

   public void copyBulletJointDataToSCS(double dt)
   {
      float jointPosition = getBulletMultiBodyLinkCollider().getJointPos();
      simOneDofJoint.setQ(jointPosition);
      float jointPVel = getBulletMultiBodyLinkCollider().getJointVel();
      simOneDofJoint.setQdd((jointPVel - simOneDofJoint.getQd()) / dt);
      simOneDofJoint.setQd(jointPVel);

      getBulletMultiBodyLinkCollider().getAppliedConstraintForce(force);
      bulletLinkAppliedForceX.set(force.getX());
      bulletLinkAppliedForceY.set(force.getY());
      bulletLinkAppliedForceZ.set(force.getZ());
      
      getBulletMultiBodyLinkCollider().getAppliedConstraintTorque(torque);
      bulletLinkAppliedTorqueX.set(torque.getX());
      bulletLinkAppliedTorqueY.set(torque.getY());
      bulletLinkAppliedTorqueZ.set(torque.getZ());

      wrenchToAdd.set(torque, force);
      getRigidBodyWrenchRegistry().addWrench(getSimRigidBody(), wrenchToAdd);
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

package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;

public abstract class BulletRobotLinkBasics 
{
   private final RigidBodyDefinition rigidBodyDefinition;
   private SimRigidBodyBasics simRigidBody;
   private final HashMap<String, Integer> jointNameToBulletJointIndexMap;
   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry;
   private btMultiBodyLinkCollider bulletMultiBodyLinkCollider;
//   private AltBulletRobotLinkCollisionSet collisionSet;
   private int bulletJointIndex;
   private final ArrayList<BulletRobotLinkBasics> children = new ArrayList<>();
   private btMultiBody bulletMultiBody;
//   private ReferenceFrame frameAfterJoint;
   private final Matrix4 bulletColliderCenterOfMassTransformToWorldBullet = new Matrix4();
   private final RigidBodyTransform bulletColliderCenterOfMassTransformToWorldEuclid = new RigidBodyTransform();
//   private int collisionGroup = 2; // Multi bodies need to be in a separate collision group
//   private int collisionGroupMask = 1 + 2; // But allowed to interact with group 1, which is rigid and static bodies

   public BulletRobotLinkBasics(RigidBodyDefinition rigidBodyDefinition,
                                SimRigidBodyBasics simRigidBody,
                                HashMap<String, Integer> jointNameToBulletJointIndexMap,
                                RigidBodyWrenchRegistry rigidBodyWrenchRegistry,
                                btMultiBodyLinkCollider bulletMultiBodyLinkCollider)
   {
      this.rigidBodyDefinition = rigidBodyDefinition;
      this.simRigidBody = simRigidBody;
      this.jointNameToBulletJointIndexMap = jointNameToBulletJointIndexMap;
      this.rigidBodyWrenchRegistry = rigidBodyWrenchRegistry;
      this.bulletMultiBodyLinkCollider = bulletMultiBodyLinkCollider;

//      frameAfterJoint = simRigidBody.getParentJoint().getFrameAfterJoint();
   }

   public 
   void addChildLinks(YoRegistry yoRegistry, BulletMultiBodyRobot bulletMultiBodyRobot)
   {
	  setBulletMultiBody(bulletMultiBodyRobot.getBulletMultiBody());
      for (JointBasics childJoint : simRigidBody.getChildrenJoints())
      {
         for (JointDefinition childJointDefinition : rigidBodyDefinition.getChildrenJoints())
         {
            if (childJoint.getName().equals(childJointDefinition.getName()))
            {
               if (childJoint instanceof SimRevoluteJoint)
               {

            	  int bulletJointIndex =  bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(childJoint.getName());
            	  btMultiBodyLinkCollider test = bulletMultiBodyRobot.getBulletMultiBody().getLinkCollider(bulletJointIndex);
                  SimRevoluteJoint childSimRevoluteJoint = (SimRevoluteJoint) childJoint;
                  RevoluteJointDefinition childRevoluteJointDefinition = (RevoluteJointDefinition) childJointDefinition;
                  getChildren().add(new BulletRobotLinkJoint(childRevoluteJointDefinition,
                                                                childSimRevoluteJoint,
                                                                jointNameToBulletJointIndexMap,
                                                                rigidBodyWrenchRegistry,
                                                                yoRegistry, 
                                                                bulletMultiBodyRobot.getBulletMultiBody().getLinkCollider(bulletJointIndex),
                                                                bulletMultiBodyRobot));
               }
               else
               {
                  throw new RuntimeException("Implement joint type: " + childJoint.getClass().getSimpleName());
               }
            }
         }
      }
   }

//   public abstract void setup(BulletPhysicsEngine bulletPhysicsEngine);

//   public AltBulletRobotLinkCollisionSet createBulletCollisionShape()
//   {
//
//      // Set collisionGroup and collisionGroupMask the same as the first shape in the CollisionShapeDefinitions.
//      if (rigidBodyDefinition.getCollisionShapeDefinitions().size() > 0)
//      {
//         setCollisionGroupMask((int) rigidBodyDefinition.getCollisionShapeDefinitions().get(0).getCollisionGroup());
//         setCollisionGroup((int) rigidBodyDefinition.getCollisionShapeDefinitions().get(0).getCollisionMask());
//      }
//
//      return collisionSet = new AltBulletRobotLinkCollisionSet(rigidBodyDefinition.getCollisionShapeDefinitions(),
//                                                            frameAfterJoint,
//                                                            simRigidBody.getBodyFixedFrame());
//   }

//   public void createBulletCollider(BulletPhysicsEngine bulletPhysicsManager)
//   {
//      bulletMultiBodyLinkCollider = new btMultiBodyLinkCollider(bulletMultiBody, bulletJointIndex);
//      bulletMultiBodyLinkCollider.setCollisionShape(collisionSet.getBulletCompoundShape());
//      bulletMultiBodyLinkCollider.setFriction(0.7f);
//
//      bulletPhysicsManager.addMultiBodyCollisionShape(bulletMultiBodyLinkCollider, collisionGroup, collisionGroupMask);
//   }

   public void updateBulletLinkColliderTransformFromMecanoRigidBody()
   {
      simRigidBody.getBodyFixedFrame().getTransformToDesiredFrame(bulletColliderCenterOfMassTransformToWorldEuclid, SimulationSession.DEFAULT_INERTIAL_FRAME);
      BulletTools.toBullet(bulletColliderCenterOfMassTransformToWorldEuclid, bulletColliderCenterOfMassTransformToWorldBullet);
      bulletMultiBodyLinkCollider.setWorldTransform(bulletColliderCenterOfMassTransformToWorldBullet);
   }

   public abstract void copyDataFromSCSToBullet();

   public abstract void copyBulletJointDataToSCS(double dt);

   public void setBulletJointIndex(int bulletJointIndex)
   {
      this.bulletJointIndex = bulletJointIndex;
   }

   public void setBulletMultiBody(btMultiBody bulletMultiBody)
   {
      this.bulletMultiBody = bulletMultiBody;
   }

   public ArrayList<BulletRobotLinkBasics> getChildren()
   {
      return children;
   }

//   public HashMap<String, Integer> getJointNameToBulletJointIndexMap()
//   {
//      return jointNameToBulletJointIndexMap;
//   }

   public btMultiBody getBulletMultiBody()
   {
      return bulletMultiBody;
   }

   public RigidBodyDefinition getRigidBodyDefinition()
   {
      return rigidBodyDefinition;
   }

   public SimRigidBodyBasics getSimRigidBody()
   {
      return simRigidBody;
   }

   public int getBulletJointIndex()
   {
      return bulletJointIndex;
   }

   public btMultiBodyLinkCollider getBulletMultiBodyLinkCollider()
   {
      return bulletMultiBodyLinkCollider;
   }

   public RigidBodyTransform getbulletColliderCenterOfMassTransformToWorldEuclid()
   {
      return bulletColliderCenterOfMassTransformToWorldEuclid;
   }

//   public void setCollisionGroup(int collisionGroup)
//   {
//      this.collisionGroup = collisionGroup;
//   }
//
//   public void setCollisionGroupMask(int collisionGroupMask)
//   {
//      this.collisionGroupMask = collisionGroupMask;
//   }
//
//   public int getCollisionGroup()
//   {
//      return collisionGroup;
//   }
//
//   public int getCollisionGroupMask()
//   {
//      return collisionGroupMask;
//   }
}

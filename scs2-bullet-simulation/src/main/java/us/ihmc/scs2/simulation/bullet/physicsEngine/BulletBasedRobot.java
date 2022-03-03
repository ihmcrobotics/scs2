package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletBasedRobot extends RobotExtension 
{
   private final BulletBasedRobotPhysics robotPhysics;
   private final RigidBodyBasics rootRigidBody;
   private final BulletBasedRobotLink elevator;
   private final BulletBasedRobotLink pelvisLink;
   private final float pelvisLinkMass;
   private final SixDoFJoint pelvisSixDoFJoint;
   private int linkCountingIndex = 0;
   private int numberOfLinks;
   private final HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<>();
   private final ArrayList<BulletBasedRobotLink> allLinks = new ArrayList<>();
   private final ArrayList<BulletBasedRobotLink> afterElevatorLinks = new ArrayList<>();
   private final ArrayList<BulletBasedRobotLink> afterPelvisLinks = new ArrayList<>();
   private btMultiBody bulletMultiBody;
   private btMultiBodyLinkCollider baseCollider;
   private final Matrix4 bulletPelvisColliderCenterOfMassTransformToWorldBullet = new Matrix4();
   private final RigidBodyTransform bulletPelvisColliderCenterOfMassTransformToWorldEuclid = new RigidBodyTransform();
   private final RigidBodyTransform bulletPelvisAfterJointTransformToWorldEuclid = new RigidBodyTransform();
   private final RigidBodyTransform bulletPelvisCenterOfMassTransformToAfterParentJointEuclid = new RigidBodyTransform();
   private final YoRegistry yoRegistry;
   
   public BulletBasedRobot(Robot robot, YoRegistry physicsRegistry)
   {
      super(robot, physicsRegistry);
      robotPhysics = new BulletBasedRobotPhysics(this);
      createBulletPhysicsFrom(robot);
      
      yoRegistry = new YoRegistry(getRobotDefinition().getName() + getClass().getSimpleName());
      rootRigidBody = getRobotDefinition().newInstance(ReferenceFrame.getWorldFrame());
      rootRigidBody.updateFramesRecursively();
      JointDefinition pelvisSixDoFJointDefinition = getRobotDefinition().getRootJointDefinitions().get(0);
      numberOfLinks = countJoints(pelvisSixDoFJointDefinition) - 1; // which is also number of joints in this case
      elevator = new BulletBasedRobotLink(getRobotDefinition().getRootBodyDefinition(), rootRigidBody, jointNameToBulletJointIndexMap, yoRegistry);
      initializeLinkLists(elevator);
      pelvisLink = elevator.getChildren().get(0);
      pelvisLinkMass = (float) pelvisLink.getRigidBodyDefinition().getMass();
      pelvisSixDoFJoint = (SixDoFJoint) pelvisLink.getParentJoint();
      bulletPelvisCenterOfMassTransformToAfterParentJointEuclid.set(pelvisLink.getRigidBodyDefinition().getInertiaPose());
   }
   
   private void initializeLinkLists(BulletBasedRobotLink link)
   {
      boolean isElevator = link.getRigidBodyDefinition().getName().equals("elevator");
      boolean isPelvis = link.getRigidBodyDefinition().getName().equals("pelvis");
      allLinks.add(link);
      if (!isElevator)
      {
         afterElevatorLinks.add(link);
      }
      if (!isElevator && !isPelvis)
      {
         afterPelvisLinks.add(link);
      }

      for (BulletBasedRobotLink child : link.getChildren())
      {
         initializeLinkLists(child);
      }
   }
   
   private int countJoints(JointDefinition joint)
   {
      jointNameToBulletJointIndexMap.put(joint.getName(), linkCountingIndex - 1);
      ++linkCountingIndex;
      int numberOfJoints = 1;
      for (JointDefinition childrenJoint : joint.getSuccessor().getChildrenJoints())
      {
         numberOfJoints += countJoints(childrenJoint);
      }
      return numberOfJoints;
   }
   
   public void createBulletMultiBody(BulletBasedPhysicsEngine bulletPhysicsManager)
   {
      boolean fixedBase = false;
      boolean canSleep = false;

      Vector3 pelvisIntertia = new Vector3((float) pelvisLink.getRigidBodyDefinition().getMomentOfInertia().getM00(),
                                           (float) pelvisLink.getRigidBodyDefinition().getMomentOfInertia().getM11(),
                                           (float) pelvisLink.getRigidBodyDefinition().getMomentOfInertia().getM22());
      bulletMultiBody = new btMultiBody(numberOfLinks, pelvisLinkMass, pelvisIntertia, fixedBase, canSleep);
      bulletMultiBody.setHasSelfCollision(true);
      bulletMultiBody.setUseGyroTerm(true);
      bulletMultiBody.setLinearDamping(0.1f);
      bulletMultiBody.setAngularDamping(0.9f);

      pelvisLink.createBulletCollisionShape(bulletMultiBody, bulletPhysicsManager);
      baseCollider = pelvisLink.getBulletMultiBodyLinkCollider();
      bulletMultiBody.setBaseCollider(baseCollider);

      for (BulletBasedRobotLink link : afterPelvisLinks)
      {
         link.createBulletJoint(bulletMultiBody);
         link.createBulletCollisionShape(bulletMultiBody, bulletPhysicsManager);
      }
      bulletMultiBody.finalizeMultiDof();
   }

   public void setPelvisCenterOfMassTransformToWorld(RigidBodyTransform pelvisCenterOfMassTransformToWorld)
   {
      BulletTools.toBullet(pelvisCenterOfMassTransformToWorld, bulletPelvisColliderCenterOfMassTransformToWorldBullet);
      bulletMultiBody.setBaseWorldTransform(bulletPelvisColliderCenterOfMassTransformToWorldBullet);
      baseCollider.setWorldTransform(bulletPelvisColliderCenterOfMassTransformToWorldBullet);
      bulletPelvisAfterJointTransformToWorldEuclid.set(pelvisCenterOfMassTransformToWorld);
      bulletPelvisCenterOfMassTransformToAfterParentJointEuclid.inverseTransform(bulletPelvisAfterJointTransformToWorldEuclid);
      pelvisSixDoFJoint.setJointPosition(bulletPelvisAfterJointTransformToWorldEuclid.getTranslation());
      pelvisSixDoFJoint.setJointOrientation(bulletPelvisAfterJointTransformToWorldEuclid.getRotation());
      rootRigidBody.updateFramesRecursively();
   }

   public void initializeRobotToDefaultJointAngles()
   {
      for (JointBasics joint : rootRigidBody.childrenSubtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            getRobotDefinition().getJointDefinition(joint.getName()).getInitialJointState().getConfiguration(joint);
         }
      }
      rootRigidBody.updateFramesRecursively();
   }

   public void updateFromMecanoRigidBody()
   {
      updateFromMecanoRigidBody(pelvisLink);
   }

   private void updateFromMecanoRigidBody(BulletBasedRobotLink link)
   {
      link.updateFromMecanoRigidBody();

      for (BulletBasedRobotLink child : link.getChildren())
      {
         updateFromMecanoRigidBody(child);
      }
   }
   
   public void updateFromBulletData()
   {
      baseCollider.getWorldTransform(bulletPelvisColliderCenterOfMassTransformToWorldBullet);
      BulletTools.toEuclid(bulletPelvisColliderCenterOfMassTransformToWorldBullet, bulletPelvisColliderCenterOfMassTransformToWorldEuclid);
      bulletPelvisAfterJointTransformToWorldEuclid.set(bulletPelvisColliderCenterOfMassTransformToWorldEuclid);
      bulletPelvisCenterOfMassTransformToAfterParentJointEuclid.inverseTransform(bulletPelvisAfterJointTransformToWorldEuclid);
      pelvisSixDoFJoint.setJointPosition(bulletPelvisAfterJointTransformToWorldEuclid.getTranslation());
      pelvisSixDoFJoint.setJointOrientation(bulletPelvisAfterJointTransformToWorldEuclid.getRotation());
      // TODO: Calculate velocity & acceleration to pack Mecano stuff?

      for (BulletBasedRobotLink afterPelvisLink : afterPelvisLinks)
      {
         afterPelvisLink.updateJointAngleFromBulletData();
      }

      rootRigidBody.updateFramesRecursively();

      for (BulletBasedRobotLink afterElevatorLink : afterElevatorLinks)
      {
         afterElevatorLink.updateFrames();
      }
   }

   public void afterSimulate()
   {
      for (BulletBasedRobotLink afterPelvisLink : afterPelvisLinks)
      {
         afterPelvisLink.afterSimulate();
      }
   }

   public BulletBasedRobotLink getElevator()
   {
      return elevator;
   }

   public BulletBasedRobotLink getPelvisLink()
   {
      return pelvisLink;
   }

   public btMultiBody getBulletMultiBody()
   {
      return bulletMultiBody;
   }

   public float getPelvisLinkMass()
   {
      return pelvisLinkMass;
   }

   public ArrayList<BulletBasedRobotLink> getAfterPelvisLinks()
   {
      return afterPelvisLinks;
   }

   public ArrayList<BulletBasedRobotLink> getAfterElevatorLinks()
   {
      return afterElevatorLinks;
   }

   public ArrayList<BulletBasedRobotLink> getAllLinks()
   {
      return allLinks;
   }

   private void createBulletPhysicsFrom(Robot robot)
   {
      robot.getRobotDefinition();
      // Instantiate the bullet physics objects.
      // Initialize the jointPairingList.
   }

   public void updateSensors()
   {
      for (SimJointBasics joint : getRootBody().childrenSubtreeIterable())
      {
         joint.getAuxialiryData().update(robotPhysics.getPhysicsOutput());
      }
   }

}
